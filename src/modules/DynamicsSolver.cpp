#include <yandy/modules/DynamicsSolver.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <random>
#include <cmath>
#include <algorithm>

#include "yandy/core/Logger.hpp"

#define YANDY_URDF_PATH YANDY_CONFIG_PATH "urdf/yandy_urdf.urdf"


namespace yandy::modules
{
    DynamicsSolver::DynamicsSolver()
    {
        m_logger = core::create_logger("Solver", spdlog::level::info);
        m_logger->info("Initializing solver, loading urdf from {}", YANDY_URDF_PATH);
        try
        {
            pinocchio::urdf::buildModel(YANDY_URDF_PATH, m_model);
        }
        catch (const std::exception& e)
        {
            m_logger->error("Error loading URDF: {}", e.what());
            throw;
        }
        m_data = pinocchio::Data(m_model);
        if (m_model.existJointName("joint_5"))
        {
            m_ee_joint_id = m_model.getJointId("joint_5");
        }
        else
        {
            m_logger->error("No valid end-effector joint found, aborting...");
            throw std::runtime_error("No valid end-effector joint found");
        }
        if (m_model.existFrame("gripper_tcp"))
        {
            m_tcp_frame_id = m_model.getFrameId("gripper_tcp");
        }
        else
        {
            m_logger->error("No valid end-effector frame found.");
            throw std::runtime_error("No valid end-effector frame found");
        }
        if (m_model.existFrame("camera_optical_frame"))
        {
            m_camera_frame_id = m_model.getFrameId("camera_optical_frame");
        }
        else
        {
            m_logger->error("No valid camera-optical frame found.");
            throw std::runtime_error("No valid camera-optical frame found");
        }
        // loading store frames
        if (m_model.existFrame("store_frame_1") && m_model.existFrame("store_frame_2"))
        {
            m_store_frames[0] = m_model.getFrameId("store_frame_1");
            m_store_frames[1] = m_model.getFrameId("store_frame_2");
        }
        else
        {
            m_logger->error("No valid store frames found.");
            throw std::runtime_error("No valid store frames found");
        }


        f_ext_.resize(m_model.njoints, pinocchio::Force::Zero());

        m_logger->info(" Model loaded. Joints: {}, DoF: {}", m_model.njoints, m_model.nv);
    }

    void DynamicsSolver::updateKinematics(const common::VectorJ& q, const common::VectorJ& v)
    {
        m_current_q = q;
        m_current_v = v;
        pinocchio::forwardKinematics(m_model, m_data, m_current_q, m_current_v);
        pinocchio::updateFramePlacements(m_model, m_data);
    }

    common::VectorJ DynamicsSolver::computeRNEA(const common::VectorJ& acc_des, const common::Vector6& ext_wrench_world)
    {
        std::ranges::fill(f_ext_, pinocchio::Force::Zero());
        if (!ext_wrench_world.isZero())
        {
            // Pinocchio 的 f_ext 要求定义在“关节局部坐标系”下
            // 我们输入的是“世界坐标系”下的力，所以需要转换
            // 获取末端关节在世界坐标系下的位姿 (Rotation Matrix)
            // data.oMi[id] 存储了从 Local 到 World 的变换
            const auto& iso_world_to_local = m_data.oMi[m_ee_joint_id].inverse();

            // 将世界坐标系的力 (Force + Torque) 转换到局部坐标系
            // act() 是 Pinocchio 的空间变换函数
            const pinocchio::Force f_world(ext_wrench_world.head<3>(), ext_wrench_world.tail<3>());
            const pinocchio::Force f_local = iso_world_to_local.act(f_world);

            // 施加到对应关节
            f_ext_[m_ee_joint_id] = f_local;
        }
        // 运行 RNEA
        return pinocchio::rnea(m_model, m_data,
                               m_current_q, // 使用 updateKinematics 时存下的位置
                               m_current_v,
                               acc_des,
                               f_ext_);
    }

    common::VectorJ DynamicsSolver::computeGravity()
    {
        return pinocchio::rnea(m_model, m_data, m_current_q, common::VectorJ::Zero(), common::VectorJ::Zero());
    }

    Eigen::Isometry3d DynamicsSolver::getEndEffectorPose() const
    {
        pinocchio::SE3 se3 = m_data.oMf[m_tcp_frame_id];

        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = se3.translation();
        pose.linear() = se3.rotation();

        return pose;
    }

    Eigen::Isometry3d DynamicsSolver::getCameraPose() const
    {
        return Eigen::Isometry3d(m_data.oMf[m_camera_frame_id].toHomogeneousMatrix());
    }

    Eigen::Isometry3d DynamicsSolver::getStoreFrame(const size_t index) const
    {
        return Eigen::Isometry3d(m_data.oMf[m_store_frames[index]].toHomogeneousMatrix());
    }

    Eigen::Isometry3d DynamicsSolver::transformObjectToBase(const Eigen::Isometry3d& T_cam_obj) const
    {
        // T_base_cam: 此时刻相机的位姿 (由 getCameraPose 算出)
        const Eigen::Isometry3d T_base_cam = getCameraPose();
        return T_base_cam * T_cam_obj;
    }


    common::VectorJ DynamicsSolver::solveIK5(const Eigen::Isometry3d& target_pose,
                                             const common::VectorJ& q_guess, double tol, int max_iter)
    {
        constexpr int MAX_RETRIES = 3;
        common::VectorJ best_q = q_guess;
        double min_err = 1e9;

        // 转换目标位姿为 Pinocchio 的 SE3 格式
        const pinocchio::SE3 oMdes(target_pose.rotation(), target_pose.translation());
        // 获取当前末端位姿 (引用,该值会实时更新)
        const pinocchio::SE3& oMcurr = m_data.oMf[m_tcp_frame_id];
        // 算法参数
        // 关节速度增量 (dq)
        Eigen::VectorXd v(m_model.nv);

        for (auto restart = 0; restart < MAX_RETRIES; ++restart)
        {
            // 初始化当前关节角
            common::VectorJ q = (restart == 0) ? q_guess : generateRandomJointPositions(); // 第一次用猜测值，后面用随机值

            // 迭代循环
            for (int i = 0; i < max_iter; ++i)
            {
                // 计算正向运动学和关节雅可比
                pinocchio::computeJointJacobians(m_model, m_data, q);
                pinocchio::updateFramePlacements(m_model, m_data);

                // 计算当前位姿和目标位姿之间的 6D 误差 (在末端局部坐标系下)
                // logic: err = log(oMcurr^{-1} * oMdes)
                pinocchio::SE3 iMd = oMcurr.actInv(oMdes);
                Eigen::Matrix<double, 6, 1> err6 = pinocchio::log6(iMd).toVector();

                // 忽略绕局部 Z 轴的旋转误差
                err6(5) = 0;

                const double current_err_norm = err6.norm();
                if (current_err_norm < min_err)
                {
                    min_err = current_err_norm;
                    best_q = q; // 保存历史上误差最小的一组解
                }

                // 判断是否收敛（此时衡量 5D 误差）
                if (current_err_norm < tol)
                {
                    return q;
                }

                // 计算 6D 雅可比矩阵
                Eigen::Matrix<double, 6, common::JOINT_NUM> J6;
                pinocchio::computeFrameJacobian(m_model, m_data, q, m_tcp_frame_id, pinocchio::LOCAL, J6);

                // 同样要把雅可比矩阵中对应局部 Z 轴旋转的那一行清零
                // 不要试图通过改变关节来消除绕 Z 轴的旋转
                J6.row(5).setZero();

                // 动态阻尼系数
                // 误差越大，阻尼越大（防止发散）；误差越小，阻尼越小（加速收敛）
                constexpr double base_damping = 1e-3;
                const double adaptive_damping = base_damping + 0.05 * current_err_norm;

                // 加权软限位
                // 在关节逼近限位时，增加特定关节的对角线惩罚权重

                Eigen::VectorXd w_limit = Eigen::VectorXd::Zero(m_model.nv);

                for (int k = 0; k < m_model.nv; ++k)
                {
                    constexpr double max_penalty = 1.0;
                    constexpr double buffer_ratio = 0.1;
                    const double q_min = m_model.lowerPositionLimit[k];
                    const double q_max = m_model.upperPositionLimit[k];
                    const double q_range = q_max - q_min;
                    const double q_buffer = q_range * buffer_ratio;

                    if (q[k] > q_max - q_buffer) // 接近上限
                    {
                        // 二次函数惩罚：越靠近边缘，惩罚越大
                        const double penetration = (q[k] - (q_max - q_buffer)) / q_buffer;
                        w_limit[k] = max_penalty * (penetration * penetration);
                    }
                    else if (q[k] < q_min + q_buffer) // 接近下限
                    {
                        const double penetration = ((q_min + q_buffer) - q[k]) / q_buffer;
                        w_limit[k] = max_penalty * (penetration * penetration);
                    }
                }


                // 标准的阻尼最小二乘求解 (DLS)
                // 构建 H 矩阵并加入 动态阻尼 和 软限位惩罚
                Eigen::Matrix<double, common::JOINT_NUM, common::JOINT_NUM> H = J6.transpose() * J6;
                // H_ii = J^T J + lambda^2 + w_limit
                H.diagonal().array() += (adaptive_damping * adaptive_damping) + w_limit.array();

                Eigen::Vector<double, common::JOINT_NUM> g;
                g = J6.transpose() * err6;

                // 求解关节速度增量
                v = H.llt().solve(g);

                // 步长限制，防止奇异点附近发散
                if (constexpr double max_step = 0.5; v.norm() > max_step)
                {
                    v = v.normalized() * max_step;
                }

                // 更新关节角
                // q = q + v
                // pinocchio::integrate 处理了欧拉角/四元数等复杂情况，虽然对纯旋转轴简单的加法也可以
                pinocchio::integrate(m_model, q, v, q);

                // 关节限位夹钳 (Clamping)
                // 防止解算出超出物理极限的角度
                q = q.cwiseMax(m_model.lowerPositionLimit).cwiseMin(m_model.upperPositionLimit);
            }
        }

        // 对于 5 自由度机械臂，求解 6D 目标往往无法完美收敛（success=false）
        // 此时我们返回“最后一次收敛的结果”，让机械臂尽力而为
        // m_logger->debug("[IK] Failed to converge. Error: {}", err.norm())
        return best_q;
    }

    common::VectorJ DynamicsSolver::generateRandomJointPositions()
    {
        common::VectorJ q;
        // initialize to zeros (in case JOINT_NUM is large)
        q.setZero();

        // thread-local RNG to avoid reseeding each call and to be safe in multithreaded contexts
        thread_local std::mt19937_64 rng{std::random_device{}()};

        for (int i = 0; i < common::JOINT_NUM; ++i)
        {
            const double lower = m_model.lowerPositionLimit[i];
            const double upper = m_model.upperPositionLimit[i];
            std::uniform_real_distribution<double> dist(lower, upper);
            q[i] = dist(rng);
        }

        return q;
    }
}

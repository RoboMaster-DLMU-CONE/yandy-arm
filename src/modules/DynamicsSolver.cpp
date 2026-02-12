#include <yandy/modules/DynamicsSolver.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

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
            camera_frame_id_ = m_model.getFrameId("camera_optical_frame");
        }
        else
        {
            m_logger->error("No valid camera-optical frame found.");
            throw std::runtime_error("No valid camera-optical frame found");
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

    Eigen::Isometry3d DynamicsSolver::getCameraPose()
    {
        return Eigen::Isometry3d(m_data.oMf[camera_frame_id_].toHomogeneousMatrix());
    }

    Eigen::Isometry3d DynamicsSolver::transformObjectToBase(const Eigen::Isometry3d& T_cam_obj)
    {
        // T_base_cam: 此时刻相机的位姿 (由 getCameraPose 算出)
        const Eigen::Isometry3d T_base_cam = getCameraPose();
        return T_base_cam * T_cam_obj;
    }


    std::optional<common::VectorJ> DynamicsSolver::solveIK(const Eigen::Isometry3d& target_pose,
                                                           const common::VectorJ& q_guess, double tol, int max_iter,
                                                           bool position_only)
    {
        // 转换目标位姿为 Pinocchio 的 SE3 格式
        const pinocchio::SE3 oMdes(target_pose.rotation(), target_pose.translation());

        // 初始化当前关节角
        common::VectorJ q = q_guess;

        // 算法参数
        constexpr double damping = 1e-3; // 阻尼系数 (lambda)，防止奇异点飞车
        constexpr double step_size = 1.0; // 步长，通常设为 1.0

        // 临时变量 (定义在栈上以提高速度)
        Eigen::VectorXd v(m_model.nv); // 关节速度增量 (dq)

        bool success = false;

        // 迭代循环
        for (int i = 0; i < max_iter; ++i)
        {
            // 正运动学 (FK) 更新当前位姿
            pinocchio::forwardKinematics(m_model, m_data, q);
            pinocchio::updateFramePlacements(m_model, m_data);

            // 获取当前末端位姿
            const pinocchio::SE3& oMcurr = m_data.oMf[m_tcp_frame_id];

            if (position_only)
            {
                // [B] 仅跟踪位置 (世界坐标系)
                const Eigen::Vector3d err_pos = target_pose.translation() - oMcurr.translation();
                if (err_pos.norm() < tol)
                {
                    success = true;
                    break;
                }

                Eigen::Matrix<double, 6, common::JOINT_NUM> J6;
                pinocchio::computeFrameJacobian(m_model, m_data, q, m_tcp_frame_id,
                                                pinocchio::LOCAL_WORLD_ALIGNED, J6);
                const Eigen::Matrix<double, 3, common::JOINT_NUM> J = J6.topRows<3>();

                Eigen::Matrix<double, common::JOINT_NUM, common::JOINT_NUM> H;
                H = J.transpose() * J;
                H.diagonal().array() += damping * damping;

                Eigen::Vector<double, common::JOINT_NUM> g;
                g = J.transpose() * err_pos;
                v = H.llt().solve(g);
            }
            else
            {
                // [B] 计算误差 (在局部坐标系下的 Twist)
                // logic: err = log(oMcurr^{-1} * oMdes)
                pinocchio::SE3 iMd = oMcurr.actInv(oMdes);
                Eigen::Matrix<double, 6, 1> err = pinocchio::log6(iMd).toVector();

                if (err.norm() < tol)
                {
                    success = true;
                    break;
                }

                Eigen::Matrix<double, 6, common::JOINT_NUM> J;
                pinocchio::computeFrameJacobian(m_model, m_data, q, m_tcp_frame_id, pinocchio::LOCAL, J);

                Eigen::Matrix<double, common::JOINT_NUM, common::JOINT_NUM> H;
                H = J.transpose() * J;
                H.diagonal().array() += damping * damping;

                Eigen::Vector<double, common::JOINT_NUM> g;
                g = J.transpose() * err;

                v = H.llt().solve(g);
            }

            // [F] 更新关节角
            // q = q + v * step_size
            // pinocchio::integrate 处理了欧拉角/四元数等复杂情况，虽然对纯旋转轴简单的加法也可以
            pinocchio::integrate(m_model, q, v * step_size, q);

            // [G] 关节限位夹钳 (Clamping)
            // 防止解算出超出物理极限的角度
            for (int k = 0; k < common::JOINT_NUM; k++)
            {
                // 需要确保 URDF 里 lower/upper 填对了
                q[k] = std::max(m_model.lowerPositionLimit[k], std::min(m_model.upperPositionLimit[k], q[k]));
            }
        }

        if (success)
        {
            return q;
        }

        // 对于 5 自由度机械臂，求解 6D 目标往往无法完美收敛（success=false）
        // 此时我们返回“最后一次收敛的结果”，让机械臂尽力而为
        // m_logger->debug("[IK] Failed to converge. Error: {}", err.norm());
        return q;
    }
}

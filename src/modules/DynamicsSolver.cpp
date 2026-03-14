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
        : m_geom_data(m_geom_model) // Initialize with empty model, update later
    {
        m_logger = core::create_logger("Solver", spdlog::level::info);
        m_logger->info("Initializing solver, loading urdf from {}", YANDY_URDF_PATH);
        try
        {
            pinocchio::urdf::buildModel(YANDY_URDF_PATH, m_model);
            
            // Build geometry model for collision
            std::vector<std::string> package_dirs;
            // Assuming YANDY_CONFIG_PATH is a string literal ending with /
            std::string config_path = YANDY_CONFIG_PATH;
            package_dirs.push_back(config_path + "urdf");
            
            pinocchio::urdf::buildGeom(m_model, YANDY_URDF_PATH, pinocchio::COLLISION, m_geom_model, package_dirs);
            
            m_geom_model.addAllCollisionPairs();
            
            // Remove collision pairs between adjacent links (parent-child)
            for (int i = 0; i < m_geom_model.collisionPairs.size(); )
            {
                const auto& cp = m_geom_model.collisionPairs[i];
                const auto& obj1 = m_geom_model.geometryObjects[cp.first];
                const auto& obj2 = m_geom_model.geometryObjects[cp.second];
                
                const auto joint1 = obj1.parentJoint;
                const auto joint2 = obj2.parentJoint;
                
                bool remove = false;
                // Self-collision (same joint)
                if (joint1 == joint2) remove = true;
                
                // Adjacent collision
                if (m_model.parents[joint1] == joint2 || m_model.parents[joint2] == joint1) remove = true;
                
                if (remove)
                {
                    m_geom_model.removeCollisionPair(cp);
                    // Do not increment i, as removeCollisionPair shifts the vector
                }
                else
                {
                    i++;
                }
            }
            
            // Re-initialize geometry data with the populated model
            m_geom_data = pinocchio::GeometryData(m_geom_model);
            
            m_logger->info("Geometry model loaded. Collision pairs: {}", m_geom_model.collisionPairs.size());
        }
        catch (const std::exception& e)
        {
            m_logger->error("Error loading URDF: {}", e.what());
            throw;
        }
        m_data = pinocchio::Data(m_model);
        if (m_model.existJointName("joint_6"))
        {
            m_ee_joint_id = m_model.getJointId("joint_6");
        }
        else if (m_model.existJointName("joint_5"))
        {
             m_logger->warn("joint_6 not found, falling back to joint_5");
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


    tl::expected<common::VectorJ, common::VectorJ> DynamicsSolver::solveIK(const Eigen::Isometry3d& target_pose,
                                                                            const common::VectorJ& q_guess, double tol,
                                                                            int max_iter)
    {
        constexpr int MAX_RETRIES = 5;
        common::VectorJ best_q = q_guess;
        double min_err = 1e9;

        // 转换目标位姿为 Pinocchio 的 SE3 格式
        const pinocchio::SE3 oMdes(target_pose.rotation(), target_pose.translation());

        // 预分配计算变量
        Eigen::VectorXd v(m_model.nv);
        Eigen::Matrix<double, 6, common::JOINT_NUM> J(6, m_model.nv);
        Eigen::Matrix<double, common::JOINT_NUM, common::JOINT_NUM> H;
        Eigen::Vector<double, common::JOINT_NUM> g;

        for (auto restart = 0; restart < MAX_RETRIES; ++restart)
        {
            // 初始化当前关节角
            common::VectorJ q = (restart == 0) ? q_guess : generateRandomJointPositions();
            bool collision_detected = false;

            // 迭代循环
            for (int i = 0; i < max_iter; ++i)
            {
                // 计算正向运动学和关节雅可比
                pinocchio::computeJointJacobians(m_model, m_data, q);
                pinocchio::updateFramePlacements(m_model, m_data);

                // 计算当前位姿和目标位姿之间的 6D 误差 (在末端局部坐标系下)
                const pinocchio::SE3& oMcurr = m_data.oMf[m_tcp_frame_id];
                const pinocchio::SE3 iMd = oMcurr.actInv(oMdes);
                Eigen::Matrix<double, 6, 1> err = pinocchio::log6(iMd).toVector();

                const double current_err_norm = err.norm();
                if (current_err_norm < min_err)
                {
                    min_err = current_err_norm;
                    best_q = q;
                }

                // 判断是否收敛
                if (current_err_norm < tol)
                {
                    // Check collision
                    bool collision = pinocchio::computeCollisions(m_model, m_data, m_geom_model, m_geom_data, q, true);

                    if (!collision)
                    {
                        return q;
                    }
                    else
                    {
                        // Collision detected, force restart
                        collision_detected = true;
                        break; // Break inner loop to trigger restart
                    }
                }

                // 计算 6D 雅可比矩阵 (LOCAL frame to match log6 error)
                pinocchio::getFrameJacobian(m_model, m_data, m_tcp_frame_id, pinocchio::LOCAL, J);

                // 自适应阻尼
                constexpr double base_lambda = 1e-3;
                const double adaptive_lambda = base_lambda + 0.05 * current_err_norm;
                const double lambda_sq = adaptive_lambda * adaptive_lambda;

                // 标准 DLS 求解: (J^T * J + lambda^2 * I) * v = J^T * err
                H = J.transpose() * J;
                H.diagonal().array() += lambda_sq;
                g = J.transpose() * err;

                // 使用 LDLT 分解求解线性方程 (H 是对称正定的)
                v = H.ldlt().solve(g);

                // 步长限制，防止发散
                if (constexpr double max_step = 0.5; v.norm() > max_step)
                {
                    v = v.normalized() * max_step;
                }

                // 早停: 如果步长过小，说明已收敛到局部极小值
                if (v.norm() < 1e-6)
                {
                    break;
                }

                // 更新关节角: q = q + v
                pinocchio::integrate(m_model, q, v, q);

                // 简单的关节限位夹钳 (Clamping)
                // 相比之前的 Active Set 策略，直接夹钳在 6-DoF 下通常更稳定且不易陷入死锁
                q = q.cwiseMax(m_model.lowerPositionLimit).cwiseMin(m_model.upperPositionLimit);
            }

            // 如果第一次尝试（使用猜测值）结果尚可，则不再尝试随机重启
            // 避免在无法完美到达时跳变为随机解
            if (constexpr double loose_tol = 1e-2; restart == 0 && min_err < loose_tol && !collision_detected)
            {
                // 必须验证 best_q 是否碰撞
                bool best_q_collision = pinocchio::computeCollisions(m_model, m_data, m_geom_model, m_geom_data, best_q, true);
                if (!best_q_collision)
                {
                    return tl::unexpected(best_q);
                }
                // 如果撞了，什么也不做，继续跑下一轮 restart 循环寻找安全解
            }
        }

        return tl::unexpected(best_q);
    }

    bool DynamicsSolver::checkPathCollision(
        const common::VectorJ& q_start,
        const common::VectorJ& q_goal,
        int num_samples)
    {
        for (int i = 1; i <= num_samples; ++i)
        {
            const double t = static_cast<double>(i) / num_samples;
            const common::VectorJ q_sample = (1.0 - t) * q_start + t * q_goal;
            if (pinocchio::computeCollisions(m_model, m_data, m_geom_model, m_geom_data, q_sample, true))
            {
                return true;
            }
        }
        return false;
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

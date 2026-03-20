#include <yandy/modules/DynamicsSolver.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
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
            for (size_t i = 0; i < m_geom_model.collisionPairs.size(); )
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

        m_logger->info("Model loaded. Joints: {}, DoF: {} (Arm: {}, Gimbal: {})", 
                       m_model.njoints, m_model.nv, common::ARM_JOINT_NUM, common::GIMBAL_JOINT_NUM);
    }

    void DynamicsSolver::updateKinematics(const common::VectorArm& arm_q, const common::VectorArm& arm_v,
                                          const common::VectorGimbal& gimbal_q)
    {
        // 存储分离的状态
        m_current_arm_q = arm_q;
        m_current_arm_v = arm_v;
        m_current_gimbal_q = gimbal_q;
        
        // 组装完整的 9D 向量给 Pinocchio
        const common::VectorJ full_q = common::combineJoints(arm_q, gimbal_q);
        const common::VectorJ full_v = common::combineJoints(arm_v, common::VectorGimbal::Zero());
        
        pinocchio::forwardKinematics(m_model, m_data, full_q, full_v);
        pinocchio::updateFramePlacements(m_model, m_data);
    }

    common::VectorArm DynamicsSolver::computeRNEA(const common::VectorArm& acc_des, const common::Vector6& ext_wrench_world)
    {
        std::ranges::fill(f_ext_, pinocchio::Force::Zero());
        if (!ext_wrench_world.isZero())
        {
            // Pinocchio 的 f_ext 要求定义在"关节局部坐标系"下
            const auto& iso_world_to_local = m_data.oMi[m_ee_joint_id].inverse();
            const pinocchio::Force f_world(ext_wrench_world.head<3>(), ext_wrench_world.tail<3>());
            const pinocchio::Force f_local = iso_world_to_local.act(f_world);
            f_ext_[m_ee_joint_id] = f_local;
        }
        
        // 组装完整的 9D 向量
        const common::VectorJ full_q = common::combineJoints(m_current_arm_q, m_current_gimbal_q);
        const common::VectorJ full_v = common::combineJoints(m_current_arm_v, common::VectorGimbal::Zero());
        const common::VectorJ full_acc = common::combineJoints(acc_des, common::VectorGimbal::Zero());
        
        // 运行 RNEA，返回完整 9D 力矩
        const common::VectorJ full_tau = pinocchio::rnea(m_model, m_data, full_q, full_v, full_acc, f_ext_);
        
        // 只返回机械臂部分 (前 6 个元素)
        return common::extractArm(full_tau);
    }

    common::VectorArm DynamicsSolver::computeGravity()
    {
        const common::VectorJ full_q = common::combineJoints(m_current_arm_q, m_current_gimbal_q);
        const common::VectorJ zero_vel = common::VectorJ::Zero();
        const common::VectorJ full_tau = pinocchio::rnea(m_model, m_data, full_q, zero_vel, zero_vel);
        return common::extractArm(full_tau);
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
        const Eigen::Isometry3d T_base_cam = getCameraPose();
        return T_base_cam * T_cam_obj;
    }


    tl::expected<common::VectorArm, common::VectorArm> DynamicsSolver::solveIK(
        const Eigen::Isometry3d& target_pose,
        const common::VectorArm& arm_q_guess, 
        double tol,
        int max_iter)
    {
        constexpr int MAX_RETRIES = 5;
        common::VectorArm best_arm_q = arm_q_guess;
        double min_err = 1e9;

        // 转换目标位姿为 Pinocchio 的 SE3 格式
        const pinocchio::SE3 oMdes(target_pose.rotation(), target_pose.translation());

        // 预分配计算变量 (只针对 6 DoF 机械臂)
        Eigen::Matrix<double, common::ARM_JOINT_NUM, 1> v_arm;
        Eigen::Matrix<double, 6, common::JOINT_NUM> J_full;  // 完整 6x9 雅可比
        Eigen::Matrix<double, 6, common::ARM_JOINT_NUM> J_arm; // 机械臂部分 6x6 雅可比
        Eigen::Matrix<double, common::ARM_JOINT_NUM, common::ARM_JOINT_NUM> H;
        Eigen::Matrix<double, common::ARM_JOINT_NUM, 1> g;

        for (auto restart = 0; restart < MAX_RETRIES; ++restart)
        {
            // 初始化当前机械臂关节角
            common::VectorArm arm_q = (restart == 0) ? arm_q_guess : generateRandomArmPositions();
            bool collision_detected = false;

            // 迭代循环
            for (int i = 0; i < max_iter; ++i)
            {
                // 组装完整关节向量 (使用当前云台状态)
                const common::VectorJ full_q = common::combineJoints(arm_q, m_current_gimbal_q);
                
                // 计算正向运动学和关节雅可比
                pinocchio::computeJointJacobians(m_model, m_data, full_q);
                pinocchio::updateFramePlacements(m_model, m_data);

                // 计算当前位姿和目标位姿之间的 6D 误差 (在末端局部坐标系下)
                const pinocchio::SE3& oMcurr = m_data.oMf[m_tcp_frame_id];
                const pinocchio::SE3 iMd = oMcurr.actInv(oMdes);
                Eigen::Matrix<double, 6, 1> err = pinocchio::log6(iMd).toVector();

                const double current_err_norm = err.norm();
                if (current_err_norm < min_err)
                {
                    min_err = current_err_norm;
                    best_arm_q = arm_q;
                }

                // 判断是否收敛
                if (current_err_norm < tol)
                {
                    // Check collision (使用完整 9D 配置)
                    bool collision = pinocchio::computeCollisions(m_model, m_data, m_geom_model, m_geom_data, full_q, true);

                    if (!collision)
                    {
                        return arm_q;
                    }
                    else
                    {
                        collision_detected = true;
                        break;
                    }
                }

                // 计算完整 6xN 雅可比矩阵 (LOCAL frame to match log6 error)
                pinocchio::getFrameJacobian(m_model, m_data, m_tcp_frame_id, pinocchio::LOCAL, J_full);
                
                // 按正确的 Pinocchio 索引提取机械臂部分的雅可比列
                for (int j = 0; j < common::ARM_JOINT_NUM; ++j)
                {
                    J_arm.col(j) = J_full.col(common::ARM_Q_INDICES[j]);
                }

                // 自适应阻尼
                constexpr double base_lambda = 1e-3;
                const double adaptive_lambda = base_lambda + 0.05 * current_err_norm;
                const double lambda_sq = adaptive_lambda * adaptive_lambda;

                // 标准 DLS 求解: (J^T * J + lambda^2 * I) * v = J^T * err
                H = J_arm.transpose() * J_arm;
                H.diagonal().array() += lambda_sq;
                g = J_arm.transpose() * err;

                // 使用 LDLT 分解求解线性方程
                v_arm = H.ldlt().solve(g);

                // 步长限制
                if (constexpr double max_step = 0.5; v_arm.norm() > max_step)
                {
                    v_arm = v_arm.normalized() * max_step;
                }

                // 早停
                if (v_arm.norm() < 1e-6)
                {
                    break;
                }

                // 更新机械臂关节角
                arm_q += v_arm;

                // 关节限位夹钳 (使用正确的 Pinocchio 索引)
                for (int j = 0; j < common::ARM_JOINT_NUM; ++j)
                {
                    const int idx = common::ARM_Q_INDICES[j];
                    arm_q[j] = std::clamp(arm_q[j], m_model.lowerPositionLimit[idx], m_model.upperPositionLimit[idx]);
                }
            }

            // 如果第一次尝试结果尚可，则不再尝试随机重启
            if (constexpr double loose_tol = 1e-2; restart == 0 && min_err < loose_tol && !collision_detected)
            {
                const common::VectorJ best_full_q = common::combineJoints(best_arm_q, m_current_gimbal_q);
                bool best_q_collision = pinocchio::computeCollisions(m_model, m_data, m_geom_model, m_geom_data, best_full_q, true);
                if (!best_q_collision)
                {
                    return tl::unexpected(best_arm_q);
                }
            }
        }

        return tl::unexpected(best_arm_q);
    }

    bool DynamicsSolver::checkPathCollision(
        const common::VectorArm& arm_q_start,
        const common::VectorArm& arm_q_goal,
        int num_samples)
    {
        for (int i = 1; i <= num_samples; ++i)
        {
            const double t = static_cast<double>(i) / num_samples;
            const common::VectorArm arm_q_sample = (1.0 - t) * arm_q_start + t * arm_q_goal;
            // 使用当前云台状态进行碰撞检测
            const common::VectorJ full_q = common::combineJoints(arm_q_sample, m_current_gimbal_q);
            if (pinocchio::computeCollisions(m_model, m_data, m_geom_model, m_geom_data, full_q, true))
            {
                return true;
            }
        }
        return false;
    }

    common::VectorArm DynamicsSolver::generateRandomArmPositions()
    {
        common::VectorArm arm_q;
        arm_q.setZero();

        thread_local std::mt19937_64 rng{std::random_device{}()};

        for (int i = 0; i < common::ARM_JOINT_NUM; ++i)
        {
            const int idx = common::ARM_Q_INDICES[i];
            const double lower = m_model.lowerPositionLimit[idx];
            const double upper = m_model.upperPositionLimit[idx];
            std::uniform_real_distribution<double> dist(lower, upper);
            arm_q[i] = dist(rng);
        }

        return arm_q;
    }
}

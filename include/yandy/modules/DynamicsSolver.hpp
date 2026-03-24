#ifndef YANDY_ARM_DYNAMICSSOLVER_HPP
#define YANDY_ARM_DYNAMICSSOLVER_HPP

#include <yandy/common/Types.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/collision/collision.hpp>
#include <spdlog/spdlog.h>
#include <tl/expected.hpp>

namespace yandy::modules
{
    class DynamicsSolver
    {
    public:
        DynamicsSolver();

        DynamicsSolver(const DynamicsSolver&) = delete;
        DynamicsSolver& operator=(const DynamicsSolver&) = delete;

        // 更新运动学 (FK)
        // 每次控制循环开始时调用一次，计算所有关节的位置、速度、Jacobian
        // arm_q, arm_v: 机械臂关节状态 (6 DoF)
        // gimbal_q: 云台关节位置 (3 DoF)，作为已知环境状态
        void updateKinematics(const common::VectorArm& arm_q, const common::VectorArm& arm_v,
                              const common::VectorGimbal& gimbal_q);

        // 计算完整逆动力学 (RNEA) - 只返回机械臂力矩
        // acc_des: 机械臂期望加速度 (来自轨迹规划)
        // ext_wrench_world: 作用在末端的世界坐标系下的外力
        // 返回: 机械臂所需关节力矩 tau (6 DoF)
        common::VectorArm computeRNEA(const common::VectorArm& acc_des, const common::Vector6& ext_wrench_world);

        // 仅计算机械臂重力补偿 (简化版 RNEA)
        common::VectorArm computeGravity();

        // 获取末端执行器位姿 (用于视觉对齐)
        // 返回: 4x4 变换矩阵 (Isometry3d)
        Eigen::Isometry3d getEndEffectorPose() const;

        // 获取相机光心相对于基座的位姿
        // 调用前必须先调用 updateKinematics()
        Eigen::Isometry3d getCameraPose() const;

        Eigen::Isometry3d getStoreFrame(size_t index) const;

        // 将 PnP 结果转换到基座坐标系
        // T_cam_obj: PnP 算出来的 (物体在相机系)
        Eigen::Isometry3d transformObjectToBase(const Eigen::Isometry3d& T_cam_obj) const;

        /**
         * @brief 数值法逆运动学求解 (仅机械臂 6 DoF)
         *
         * @param target_pose 期望的末端位姿 (基座坐标系)
         * @param arm_q_guess 猜测的机械臂初始角度
         * @param tol         位置误差容忍度 (单位: m 或 rad)
         * @param max_iter    最大迭代次数
         * @return 收敛时返回机械臂关节角；未收敛时在 error 中返回最佳近似解
         *
         * 注意: IK 求解时使用当前云台状态 (m_current_gimbal_q) 进行碰撞检测
         */
        tl::expected<common::VectorArm, common::VectorArm> solveIK(
            const Eigen::Isometry3d& target_pose,
            const common::VectorArm& arm_q_guess,
            double tol = 1e-4,
            int max_iter = 100
        );

        /**
         * @brief 5DoF 逆运动学求解 (位置 + 逼近方向，yaw 自由)
         *
         * 用于圆柱形矿石抓取场景：只约束末端位置和逼近方向（末端 Z 轴对齐目标法向），
         * yaw（绕末端 Z 轴旋转）自由浮动，利用这个自由度最小化关节移动量。
         *
         * @param target_position   目标位置 (基座坐标系)
         * @param approach_direction 逼近方向/目标法向量 (单位向量，基座坐标系)
         * @param arm_q_guess       猜测的机械臂初始角度
         * @param tol               误差容忍度 (位置: m, 方向: rad)
         * @param max_iter          最大迭代次数
         * @return 收敛时返回机械臂关节角；未收敛时在 error 中返回最佳近似解
         */
        tl::expected<common::VectorArm, common::VectorArm> solveIK5DoF(
            const Eigen::Vector3d& target_position,
            const Eigen::Vector3d& approach_direction,
            const common::VectorArm& arm_q_guess,
            double tol = 1e-4,
            int max_iter = 100
        );

        /**
         * @brief 检查两个机械臂构型之间的直线路径是否存在碰撞
         *
         * 在 arm_q_start 和 arm_q_goal 之间线性插值 num_samples 个点，
         * 使用当前云台状态进行碰撞检测。
         *
         * @return true 表示路径上存在碰撞
         */
        bool checkPathCollision(
            const common::VectorArm& arm_q_start,
            const common::VectorArm& arm_q_goal,
            int num_samples = 5  // 减少采样点，降低假阳性
        );

        // 获取 Pinocchio 模型和数据
        pinocchio::Model& getModel() { return m_model; }
        pinocchio::Data& getData() { return m_data; }
        const pinocchio::GeometryModel& getGeometryModel() const { return m_geom_model; }

        // 启用/禁用与 base_link 相关的碰撞对（用于存取矿等会与底座发生干涉的场景）
        void setBaseLinkCollisionEnabled(bool enabled);

        // 获取当前云台状态 (用于 TrajectoryPlanner)
        const common::VectorGimbal& getCurrentGimbalQ() const { return m_current_gimbal_q; }

    private:
        common::VectorArm generateRandomArmPositions();

        pinocchio::Model m_model;
        pinocchio::Data m_data;
        pinocchio::GeometryModel m_geom_model;
        pinocchio::GeometryData m_geom_data;

        std::vector<size_t> m_base_link_pair_indices; // 涉及 base_link 的碰撞对索引（缓存）

        // 分离存储机械臂和云台状态
        common::VectorArm m_current_arm_q{common::VectorArm::Zero()};
        common::VectorArm m_current_arm_v{common::VectorArm::Zero()};
        common::VectorGimbal m_current_gimbal_q{common::VectorGimbal::Zero()};

        pinocchio::JointIndex m_ee_joint_id{static_cast<pinocchio::JointIndex>(-1)}; // 末端关节在 Pinocchio 模型中的索引
        pinocchio::FrameIndex m_tcp_frame_id{static_cast<pinocchio::FrameIndex>(-1)};
        pinocchio::FrameIndex m_camera_frame_id{static_cast<pinocchio::FrameIndex>(-1)};
        pinocchio::FrameIndex m_store_frames[2]{
            static_cast<pinocchio::FrameIndex>(-1), static_cast<pinocchio::FrameIndex>(-1)
        };

        // 预分配外力容器，避免实时分配内存
        pinocchio::container::aligned_vector<pinocchio::Force> f_ext_;
        Eigen::Matrix<double, 6, 1> m_err;
        Eigen::VectorXd m_v;

        std::shared_ptr<spdlog::logger> m_logger;
    };
}


#endif //YANDY_ARM_DYNAMICSSOLVER_HPP

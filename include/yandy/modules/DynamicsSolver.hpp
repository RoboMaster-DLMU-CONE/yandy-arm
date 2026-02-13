#ifndef YANDY_ARM_DYNAMICSSOLVER_HPP
#define YANDY_ARM_DYNAMICSSOLVER_HPP

#include <yandy/common/Types.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <spdlog/spdlog.h>
#include <optional>

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
        void updateKinematics(const common::VectorJ& q, const common::VectorJ& v);

        // 计算完整逆动力学 (RNEA)
        // acc_des: 期望加速度 (来自轨迹规划)
        // ext_wrench_world: 作用在末端的世界坐标系下的外力 (例如矿石重力 [Fx,Fy,Fz,0,0,0])
        // 返回: 所需关节力矩 tau
        common::VectorJ computeRNEA(const common::VectorJ& acc_des, const common::Vector6& ext_wrench_world);

        // 仅计算重力补偿 (简化版 RNEA)
        // 相当于 computeRNEA(0, 0)
        common::VectorJ computeGravity();

        // 获取末端执行器位姿 (用于视觉对齐)
        // 返回: 4x4 变换矩阵 (Isometry3d)
        Eigen::Isometry3d getEndEffectorPose() const;

        // 获取相机光心相对于基座的位姿
        // 调用前必须先调用 updateKinematics(q, v)
        Eigen::Isometry3d getCameraPose();

        // 将 PnP 结果转换到基座坐标系
        // T_cam_obj: PnP 算出来的 (物体在相机系)
        Eigen::Isometry3d transformObjectToBase(const Eigen::Isometry3d& T_cam_obj);

        /**
            * @brief 数值法逆运动学求解 (CLIK算法)
            *
            * @param target_pose 期望的末端位姿 (基座坐标系)
            * @param q_guess     猜测的初始角度 (通常传当前角度，传空则使用零位)
            * @param tol         位置误差容忍度 (单位: m 或 rad)
            * @param max_iter    最大迭代次数
            * @param position_only
            * @return std::optional<VectorJ> 如果收敛返回关节角，否则返回 nullopt
            */
        std::optional<common::VectorJ> solveIK(
            const Eigen::Isometry3d& target_pose,
            const common::VectorJ& q_guess,
            double tol = 1e-4,
            int max_iter = 100,
            bool position_only = false
        );

        // 获取 Pinocchio 模型和数据
        pinocchio::Model& getModel() { return m_model; }
        pinocchio::Data& getData() { return m_data; }

    private:
        pinocchio::Model m_model;
        pinocchio::Data m_data;

        common::VectorJ m_current_q{};
        common::VectorJ m_current_v{};

        pinocchio::JointIndex m_ee_joint_id{static_cast<pinocchio::JointIndex>(-1)}; // 末端关节在 Pinocchio 模型中的索引
        pinocchio::FrameIndex m_tcp_frame_id{static_cast<pinocchio::FrameIndex>(-1)};
        pinocchio::FrameIndex camera_frame_id_{static_cast<pinocchio::FrameIndex>(-1)};

        // 预分配外力容器，避免实时分配内存
        pinocchio::container::aligned_vector<pinocchio::Force> f_ext_;
        Eigen::Matrix<double, 6, 1> m_err;
        Eigen::VectorXd m_v;

        std::shared_ptr<spdlog::logger> m_logger;
    };
}


#endif //YANDY_ARM_DYNAMICSSOLVER_HPP

#ifndef YANDY_ARM_DYNAMICSSOLVER_HPP
#define YANDY_ARM_DYNAMICSSOLVER_HPP

#include <yandy/common/Types.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <spdlog/spdlog.h>

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

    private:
        pinocchio::Model m_model;
        pinocchio::Data m_data;

        common::VectorJ m_current_q{};
        common::VectorJ m_current_v{};

        int ee_joint_id_; // 末端关节在 Pinocchio 模型中的索引

        // 预分配外力容器，避免实时分配内存
        pinocchio::container::aligned_vector<pinocchio::Force> f_ext_;

        std::shared_ptr<spdlog::logger> m_logger;
    };
}


#endif //YANDY_ARM_DYNAMICSSOLVER_HPP

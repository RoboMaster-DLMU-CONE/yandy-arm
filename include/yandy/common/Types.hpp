#ifndef YANDY_ARM_TYPES_HPP
#define YANDY_ARM_TYPES_HPP

#include <eigen3/Eigen/Dense>
#include <RPL/Meta/PacketTraits.hpp>

struct __attribute__((packed)) YandyControlPack
{
    float x; // m
    float y; // m
    float z; // m
    float roll; // rad
    float pitch; // rad
    float yaw; // rad
};

template <>
struct RPL::Meta::PacketTraits<YandyControlPack> : PacketTraitsBase<PacketTraits<YandyControlPack>>
{
    static constexpr uint16_t cmd = 0x0604;
    static constexpr size_t size = sizeof(YandyControlPack);
};

namespace yandy::common
{
    // 关节数量
    constexpr int JOINT_NUM = 5;

    using VectorJ = Eigen::Matrix<double, JOINT_NUM, 1>;
    using Vector6 = Eigen::Matrix<double, 6, 1>; // 用于空间力向量 (Fx,Fy,Fz, Tx,Ty,Tz)

    // 机械臂状态
    struct JointState
    {
        Eigen::Vector<double, JOINT_NUM> q; // 位置 (rad)
        Eigen::Vector<double, JOINT_NUM> v; // 速度 (rad/s)
        Eigen::Vector<double, JOINT_NUM> tau; // 反馈力矩 (Nm)
    };

    // 机械臂控制指令
    struct JointCommand
    {
        Eigen::Vector<double, JOINT_NUM> q_des{}; // 期望位置
        Eigen::Vector<double, JOINT_NUM> v_des{}; // 期望速度
        Eigen::Vector<double, JOINT_NUM> kp{}; // 刚度
        Eigen::Vector<double, JOINT_NUM> kd{}; // 阻尼
        Eigen::Vector<double, JOINT_NUM> tau_ff{}; // 前馈力矩 (核心)
    };
}

#endif //YANDY_ARM_TYPES_HPP

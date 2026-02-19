#ifndef YANDY_ARM_TYPES_HPP
#define YANDY_ARM_TYPES_HPP

#include <eigen3/Eigen/Dense>
#include <RPL/Meta/PacketTraits.hpp>

enum class YandyControlCmd : uint8_t
{
    CMD_NONE = 0x00, // 心跳/无操作

    // === 系统级指令 ===
    CMD_ERROR = 0x01, // 急停 (Emergency Stop) - 最高优先级
    CMD_SWITCH_ENABLE = 0x02, // 启用/禁用输出 (Enable Toggle)
    CMD_RESET = 0x03, // 复位 (Reset Error / Clear Accumulators)

    // === 模式切换指令 ===
    CMD_SWITCH_FETCH = 0x10, // 进入/退出 抓取模式 (Fetch Toggle)
    CMD_SWITCH_STORE = 0x11, // 进入/退出 存取矿模式 (Store Toggle)

    // === 手动操作指令 ===
    CMD_SWITCH_GRIP = 0x20, // 手动切换夹爪 (Gripper Toggle)

    // === 调试/修正指令 ===
    CMD_TOGGLE_HELD = 0x80, // 强制修改“持有矿石”状态
    CMD_INC_STORE = 0x81, // 强制库存 +1
    CMD_DEC_STORE = 0x82 // 强制库存 -1
};

inline auto format_as(YandyControlCmd c)
{
    return static_cast<uint8_t>(c);
}

struct __attribute__((packed)) YandyControlPack
{
    float x; // m
    float y; // m
    float z; // m
    float roll; // rad
    float pitch; // rad
    float yaw; // rad

    YandyControlCmd cmd;
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

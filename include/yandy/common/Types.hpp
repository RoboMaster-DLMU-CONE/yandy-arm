#ifndef YANDY_ARM_TYPES_HPP
#define YANDY_ARM_TYPES_HPP

#include <RPL/Meta/PacketTraits.hpp>
#include <eigen3/Eigen/Dense>
#include <string_view>

enum class YandyControlCmd : uint8_t {
  CMD_NONE = 0x00, // 心跳/无操作

  // === 系统级指令 ===
  CMD_ERROR = 0x01,         // 急停 (Emergency Stop) - 最高优先级
  CMD_SWITCH_ENABLE = 0x02, // 启用/禁用输出 (Enable Toggle)
  CMD_RESET = 0x03,         // 复位 (Reset Error / Clear Accumulators)

  // === 模式切换指令 ===
  CMD_SWITCH_FETCH = 0x10, // 进入/退出 抓取模式 (Fetch Toggle)
  CMD_SWITCH_STORE = 0x11, // 进入/退出 存取矿模式 (Store Toggle)

  // === 手动操作指令 ===
  CMD_SWITCH_GRIP = 0x20, // 手动切换夹爪 (Gripper Toggle)

  // === 抓取流程内部指令 (由 Robot 自动触发) ===
  CMD_POSE_STABLE = 0x30,   // 视觉测量稳定 (Seeking -> PreGrasp)
  CMD_PREGRASP_DONE = 0x31, // 到达预抓取点 (PreGrasp -> Approaching)
  CMD_GRASP_DONE = 0x32,    // 完成抓取 (Approaching -> exit)

  // === 调试/修正指令 ===
  CMD_TOGGLE_HELD = 0x80, // 强制修改“持有矿石”状态
  CMD_INC_STORE = 0x81,   // 强制库存 +1
  CMD_DEC_STORE = 0x82    // 强制库存 -1
};

enum class YandyState : uint8_t {
  Unknown,
  Disabled,
  Manual,
  Fetching,
  Store,
  Error
};

constexpr std::string_view format_as(const YandyState s) {
  switch (s) {
  case YandyState::Unknown:
    return "Unknown";
  case YandyState::Disabled:
    return "Disabled";
  case YandyState::Manual:
    return "Manual";
  case YandyState::Fetching:
    return "Fetching";
  case YandyState::Store:
    return "Store";
  case YandyState::Error:
    return "Error";
  default:
    return "Unknown";
  }
}

constexpr std::string_view format_as(const YandyControlCmd c) {
  switch (c) {
  case YandyControlCmd::CMD_NONE:
    return "NONE";
  case YandyControlCmd::CMD_ERROR:
    return "ERROR";
  case YandyControlCmd::CMD_SWITCH_ENABLE:
    return "SWITCH_ENABLE";
  case YandyControlCmd::CMD_RESET:
    return "RESET";
  case YandyControlCmd::CMD_SWITCH_FETCH:
    return "SWITCH_FETCH";
  case YandyControlCmd::CMD_SWITCH_STORE:
    return "SWITCH_STORE";
  case YandyControlCmd::CMD_SWITCH_GRIP:
    return "SWITCH_GRIP";
  case YandyControlCmd::CMD_POSE_STABLE:
    return "POSE_STABLE";
  case YandyControlCmd::CMD_PREGRASP_DONE:
    return "PREGRASP_DONE";
  case YandyControlCmd::CMD_GRASP_DONE:
    return "GRASP_DONE";
  case YandyControlCmd::CMD_TOGGLE_HELD:
    return "TOGGLE_HELD";
  case YandyControlCmd::CMD_INC_STORE:
    return "INC_STORE";
  case YandyControlCmd::CMD_DEC_STORE:
    return "DEC_STORE";
  default:
    return "UNKNOWN";
  }
}

struct __attribute__((packed)) YandyControlPack {
  float x;     // m
  float y;     // m
  float z;     // m
  float ee_qw; // 末端姿态四元数 W
  float ee_qx; // 末端姿态四元数 X
  float ee_qy; // 末端姿态四元数 Y
  float ee_qz; // 末端姿态四元数 Z
  float gimbal_z;
  float gimbal_yaw;
  float gimbal_pitch;

  // === 底盘传感器数据 (来自下位机) ===
  // IMU 比力 (specific force = a_real - g), 底盘 body frame, 单位 m/s²
  // 静止水平时: ax≈0, ay≈0, az≈+9.81 (Z轴朝上)
  float ax; // X 方向线性比力
  float ay; // Y 方向线性比力
  float az; // Z 方向线性比力

  // 底盘角速度, 底盘 body frame, 单位 rad/s
  float gx; // 绕 X 轴角速度 (roll rate)
  float gy; // 绕 Y 轴角速度 (pitch rate)
  float gz; // 绕 Z 轴角速度 (yaw rate)

  // 底盘姿态四元数 (传感器融合估计, world → chassis)
  float qw; // 四元数 W 分量 (实部)
  float qx; // 四元数 X 分量
  float qy; // 四元数 Y 分量
  float qz; // 四元数 Z 分量

  YandyControlCmd cmd;
};
static_assert(sizeof(YandyControlPack) == 81,
              "YandyControlPack size mismatch — update Rust packet.rs and "
              "lower computer firmware");

template <>
struct RPL::Meta::PacketTraits<YandyControlPack>
    : PacketTraitsBase<PacketTraits<YandyControlPack>> {
  static constexpr uint16_t cmd = 0x0604;
  static constexpr size_t size = sizeof(YandyControlPack);
};

namespace yandy::common {
// 关节数量定义
constexpr int ARM_JOINT_NUM = 6;    // 机械臂关节数 (由上位机控制)
constexpr int GIMBAL_JOINT_NUM = 3; // 云台关节数 (状态从下位机读取)
constexpr int JOINT_NUM = ARM_JOINT_NUM + GIMBAL_JOINT_NUM; // 总关节数

// Pinocchio 关节顺序索引 (由 URDF 树结构决定):
// q[0] = joint_1, q[1..3] = gimbal_joint_1/2/3, q[4..8] = joint_2..6
// 因此: arm_indices = [0, 4, 5, 6, 7, 8], gimbal_indices = [1, 2, 3]
constexpr int ARM_Q_INDICES[ARM_JOINT_NUM] = {0, 4, 5, 6, 7, 8};
constexpr int GIMBAL_Q_INDICES[GIMBAL_JOINT_NUM] = {1, 2, 3};

// 类型别名
using VectorArm =
    Eigen::Matrix<double, ARM_JOINT_NUM, 1>; // 机械臂关节向量 (6D)
using VectorGimbal =
    Eigen::Matrix<double, GIMBAL_JOINT_NUM, 1>;      // 云台关节向量 (3D)
using VectorJ = Eigen::Matrix<double, JOINT_NUM, 1>; // 完整关节向量 (9D)
using Vector6 =
    Eigen::Matrix<double, 6, 1>; // 用于空间力向量 (Fx,Fy,Fz, Tx,Ty,Tz)

// 云台状态 (只读，来自下位机)
struct GimbalState {
  VectorGimbal q{VectorGimbal::Zero()}; // 位置 (rad): [z, yaw, pitch]
  VectorGimbal v{VectorGimbal::Zero()}; // 速度 (rad/s)
};

// 机械臂状态 (6 DoF)
struct ArmState {
  VectorArm q{VectorArm::Zero()};   // 位置 (rad)
  VectorArm v{VectorArm::Zero()};   // 速度 (rad/s)
  VectorArm tau{VectorArm::Zero()}; // 反馈力矩 (Nm)
};

// 机械臂控制指令 (6 DoF)
struct ArmCommand {
  VectorArm q_des{VectorArm::Zero()};  // 期望位置
  VectorArm v_des{VectorArm::Zero()};  // 期望速度
  VectorArm kp{VectorArm::Zero()};     // 刚度
  VectorArm kd{VectorArm::Zero()};     // 阻尼
  VectorArm tau_ff{VectorArm::Zero()}; // 前馈力矩 (核心)
};

// 辅助函数: 组装完整关节向量 (按 Pinocchio 顺序)
// arm = [j1, j2, j3, j4, j5, j6] -> full[0,4,5,6,7,8]
// gimbal = [z, yaw, pitch] -> full[1,2,3]
inline VectorJ combineJoints(const VectorArm &arm, const VectorGimbal &gimbal) {
  VectorJ full;
  for (int i = 0; i < ARM_JOINT_NUM; ++i) {
    full[ARM_Q_INDICES[i]] = arm[i];
  }
  for (int i = 0; i < GIMBAL_JOINT_NUM; ++i) {
    full[GIMBAL_Q_INDICES[i]] = gimbal[i];
  }
  return full;
}

// 辅助函数: 从完整向量中提取机械臂部分 (按 Pinocchio 顺序)
inline VectorArm extractArm(const VectorJ &full) {
  VectorArm arm;
  for (int i = 0; i < ARM_JOINT_NUM; ++i) {
    arm[i] = full[ARM_Q_INDICES[i]];
  }
  return arm;
}

// 辅助函数: 从完整向量中提取云台部分 (按 Pinocchio 顺序)
inline VectorGimbal extractGimbal(const VectorJ &full) {
  VectorGimbal gimbal;
  for (int i = 0; i < GIMBAL_JOINT_NUM; ++i) {
    gimbal[i] = full[GIMBAL_Q_INDICES[i]];
  }
  return gimbal;
}

} // namespace yandy::common

#endif // YANDY_ARM_TYPES_HPP

#include <yandy/core/Logger.hpp>
#include <yandy/modules/ArmHW.hpp>

#include <one/PID/PidController.hpp>
#include <one/can/CanDriver.hpp>
#include <one/motor/dji/DjiMotor.hpp>
#include <one/motor/dm/DmMotor.hpp>

#include <toml++/toml.hpp>

// Simulation Includes
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/parsers/urdf.hpp>

using one::pid::PidChain;
using one::pid::PidController;
using one::pid::PidParams;
using namespace one::motor;
using dji::GM6020_Voltage;
using dji::M2006;
using dji::M3508;
using dm::J10010L;
using dm::J4310;
using dm::J4340;
using dm::J8009;
using one::can::CanDriver;
using namespace one::motor::units::literals;

using std::make_unique;

#define JOINT_CONFIG_PATH YANDY_CONFIG_PATH "joint.toml"

namespace yandy::modules {
namespace detail {
// ---------------------------------------------------------------------
// RealArmHW Implementation
// ---------------------------------------------------------------------
class RealArmHW : public IArmHW {
public:
  explicit RealArmHW(one::can::CanDriver &can) : m_driver(can) {
    m_logger = core::create_logger("RealArmHW", spdlog::level::info);
    parse_config();
  }

  void read(common::ArmState &state) override {
    for (int i = 0; i < common::JOINT_NUM; ++i) {
      double raw_pos{}; // rad
      double raw_vel{}; // rad/s
      double raw_tau{}; // Nm
      // Check if motor is initialized
      if (!m_motors[i])
        continue;

      auto status_opt = m_motors[i]->getStatusVariant();
      if (!status_opt.has_value())
        continue;
      auto status = status_opt.value();
      std::visit(
          [&]<typename T>(const T &payload) {
            using StatusType = std::decay_t<T>;
            if constexpr (std::is_same_v<StatusType,
                                         one::motor::dm::MotorStatus>) {
              raw_pos = payload.position.numerical_value_in(rad);
              raw_vel = payload.velocity.numerical_value_in(rad / s);
              raw_tau = payload.torque.numerical_value_in(N * m);
            } else {
              raw_pos = payload.reduced_angle.numerical_value_in(rad);
              raw_vel = payload.reduced_angular.numerical_value_in(rad / s);
            }
          },
          status);

      // q_model = Dir * (q_motor - Offset)
      state.q[i] = m_dirs[i] * (raw_pos - m_offsets[i]);

      // v_model = Dir * v_motor
      state.v[i] = m_dirs[i] * raw_vel;

      // tau_model = Dir * tau_motor
      state.tau[i] = m_dirs[i] * raw_tau;
    }
  }

  void write(const common::ArmCommand &cmd) override {
    for (int i = 0; i < common::JOINT_NUM; ++i) {
      if (!m_motors[i])
        continue;

      double target_pos = (m_dirs[i] * cmd.q_des[i]) + m_offsets[i];
      double target_vel = m_dirs[i] * cmd.v_des[i];
      double ff_torque = m_dirs[i] * cmd.tau_ff[i];

      (void)m_motors[i]->setUnitRefs(target_pos * rad, target_vel * rad / s,
                                     ff_torque * N * m);
    }
  }

  void enable() override {
    m_logger->info("Enabling all Arm motors...");
    for (int i = 0; i < common::JOINT_NUM; ++i) {
      if (!m_motors[i])
        continue;
      if (auto res = m_motors[i]->enable(); !res.has_value()) {
        m_logger->error("Failed to enable motor {}: {}", i + 1,
                        res.error().message);
      }
    }
  }

  void disable() override {
    m_logger->info("Disabling all Arm motors...");
    for (int i = 0; i < common::JOINT_NUM; ++i) {
      if (!m_motors[i])
        continue;
      if (auto res = m_motors[i]->disable(); !res.has_value()) {
        m_logger->error("Failed to disable motor {}: {}", i + 1,
                        res.error().message);
      }
    }
  }

private:
  void parse_config() {
    m_logger->info(
        "Initializing Real Arm Hardware, loading configs from: {}...",
        JOINT_CONFIG_PATH);

    auto tbl = toml::parse_file(JOINT_CONFIG_PATH);
    parse_dm_motor(tbl["joint_1"], 0);
    parse_dm_motor(tbl["joint_2"], 1);
    parse_dm_motor(tbl["joint_3"], 2);
    parse_gm6020_motor(tbl["joint_4"], 3);
    parse_dm_motor(tbl["joint_5"], 4);
    parse_m2006_motor(tbl["joint_6"], 5);
  }

  void parse_dm_motor(const toml::v3::node_view<toml::v3::node> joint_node,
                      const size_t joint_index) {
    uint16_t can_id{}, master_id{};
    float kp{}, kd{};
    can_id = joint_node["can_id"].value<uint16_t>().value();
    master_id = joint_node["master_id"].value<uint16_t>().value();
    m_dirs[joint_index] = joint_node["dir"].value<float>().value();
    m_offsets[joint_index] = joint_node["offset"].value<float>().value();

    kp = joint_node["mit_pid"]["kp"].value<float>().value();
    kd = joint_node["mit_pid"]["kd"].value<float>().value();
    switch (joint_index) {
    case 0:
      // J1 : 4340
      m_motors[joint_index] = std::make_unique<J4340>(
          m_driver, dm::Param{can_id, master_id, dm::MITMode{kp, kd}});
      break;
    case 1:
      // J2 : 10010L
      m_motors[joint_index] = std::make_unique<J10010L>(
          m_driver, dm::Param{can_id, master_id, dm::MITMode{kp, kd}});
      break;
    case 2:
      // J3 : 8009
      m_motors[joint_index] = std::make_unique<J8009>(
          m_driver, dm::Param{can_id, master_id, dm::MITMode{kp, kd}});
      break;
    case 4:
      // J5 : 4310
      m_motors[joint_index] = std::make_unique<J4310>(
          m_driver, dm::Param{can_id, master_id, dm::MITMode{kp, kd}});
      break;
    default:;
    }
    m_logger->info("J{} DM motor parsed: can_id: {}, master_id: {}, dir: {}, "
                   "offset: {}, MIT params: {}, {}",
                   joint_index + 1, can_id, master_id, m_dirs[joint_index],
                   m_offsets[joint_index], kp, kd);
  }

  void parse_gm6020_motor(toml::v3::node_view<toml::v3::node> joint_node,
                          size_t joint_index) {
    float kp{}, kd{};
    const uint8_t id = joint_node["id"].value<uint8_t>().value();
    m_dirs[joint_index] = joint_node["dir"].value<float>().value();
    m_offsets[joint_index] = joint_node["offset"].value<float>().value();
    kp = joint_node["mit_pid"]["kp"].value<float>().value();
    kd = joint_node["mit_pid"]["kd"].value<float>().value();

    m_motors[joint_index] = std::make_unique<GM6020_Voltage>(
        m_driver, dji::Param{id, dji::MITMode{kp, kd}});

    m_logger->info("J{} GM6020 motor parsed: dir: {}, offset: {}",
                   joint_index + 1, m_dirs[joint_index],
                   m_offsets[joint_index]);
  }

  void parse_m2006_motor(toml::v3::node_view<toml::v3::node> joint_node,
                         size_t joint_index) {
    float kp{}, kd{};
    const uint8_t id = joint_node["id"].value<uint8_t>().value();
    m_dirs[joint_index] = joint_node["dir"].value<float>().value();
    m_offsets[joint_index] = joint_node["offset"].value<float>().value();
    kp = joint_node["mit_pid"]["kp"].value<float>().value();
    kd = joint_node["mit_pid"]["kd"].value<float>().value();

    m_motors[joint_index] =
        std::make_unique<M2006>(m_driver, dji::Param{id, dji::MITMode{kp, kd}});

    m_logger->info("J{} M2006 motor parsed: dir: {}, offset: {}",
                   joint_index + 1, m_dirs[joint_index],
                   m_offsets[joint_index]);
  }

  one::can::CanDriver &m_driver;
  std::array<std::unique_ptr<IMotor>, common::JOINT_NUM> m_motors;
  std::array<float, common::JOINT_NUM> m_dirs{};
  std::array<float, common::JOINT_NUM> m_offsets{};
  std::shared_ptr<spdlog::logger> m_logger;
};

// ---------------------------------------------------------------------
// FakeArmHW Implementation
// Simulates full 9D robot internally, but only exposes 6D arm joints
// ---------------------------------------------------------------------
class FakeArmHW : public IArmHW {
public:
  FakeArmHW() {
    m_logger = core::create_logger("FakeArmHW", spdlog::level::info);
    const std::string urdf_path = YANDY_CONFIG_PATH "urdf/yandy_urdf.urdf";
    m_logger->info("Initializing Fake Arm Hardware, loading URDF from: {}",
                   urdf_path);

    pinocchio::urdf::buildModel(urdf_path, m_sim_model);
    m_sim_data = pinocchio::Data(m_sim_model);

    // 初始化完整 9D 状态
    m_full_q = common::VectorJ::Zero();
    m_full_v = common::VectorJ::Zero();

    // 初始姿态 (机械臂)：避开奇异点，同时落在关节限位内
    m_full_q.head<common::ARM_JOINT_NUM>() << 0.0, 0.6, 1.5, 0.0, 0.8, 0.0;
    // 云台关节初始为 0
    m_full_q.tail<common::GIMBAL_JOINT_NUM>().setZero();

    // 初始化 last_cmd (6D)，避免第一帧飞车
    m_last_cmd.q_des = common::extractArm(m_full_q);
    m_last_cmd.kp.fill(20.0); // 提高刚度
    m_last_cmd.kd.fill(1.0);
  }

  void read(common::ArmState &out_state) override {
    // 只返回机械臂部分 (前 6 个关节)
    out_state.q = common::extractArm(m_full_q);
    out_state.v = common::extractArm(m_full_v);
    out_state.tau = m_arm_tau;
  }

  void write(const common::ArmCommand &cmd) override { m_last_cmd = cmd; }

  void step(double dt) override {
    // 计算重力 (完整 9D 模型)
    const common::VectorJ tau_grav =
        pinocchio::rnea(m_sim_model, m_sim_data, m_full_q,
                        common::VectorJ::Zero(), common::VectorJ::Zero());

    // 计算机械臂电机力矩 (MIT Mode) - 只针对前 6 个关节
    const common::VectorArm arm_q = common::extractArm(m_full_q);
    const common::VectorArm arm_v = common::extractArm(m_full_v);
    m_arm_tau = m_last_cmd.kp.cwiseProduct(m_last_cmd.q_des - arm_q) +
                m_last_cmd.kd.cwiseProduct(m_last_cmd.v_des - arm_v) +
                m_last_cmd.tau_ff;

    // 组装完整力矩向量 (云台力矩为 0，假设云台由下位机控制)
    common::VectorJ full_tau_motor = common::VectorJ::Zero();
    full_tau_motor.head<common::ARM_JOINT_NUM>() = m_arm_tau;

    // 简化的物理积分 (F = ma)
    common::VectorJ inertia = common::VectorJ::Constant(0.2);
    common::VectorJ damping = common::VectorJ::Constant(0.5);

    const common::VectorJ acc =
        (full_tau_motor - tau_grav - damping.cwiseProduct(m_full_v))
            .cwiseQuotient(inertia);

    // 半隐式欧拉积分
    m_full_v += acc * dt;
    m_full_q += m_full_v * dt;
  }

  void enable() override { m_logger->info("Simulation enabled"); }

  void disable() override { m_logger->info("Simulation disabled"); }

private:
  pinocchio::Model m_sim_model;
  pinocchio::Data m_sim_data;
  common::VectorJ m_full_q;                               // 完整 9D 位置
  common::VectorJ m_full_v;                               // 完整 9D 速度
  common::VectorArm m_arm_tau{common::VectorArm::Zero()}; // 机械臂力矩反馈
  common::ArmCommand m_last_cmd;
  std::shared_ptr<spdlog::logger> m_logger;
};
} // namespace detail

ArmHW::ArmHW(one::can::CanDriver &can) {
  const auto logger = core::create_logger("ArmHW", spdlog::level::info);
  logger->info("Reading joint config from: {}", JOINT_CONFIG_PATH);

  auto tbl = toml::parse_file(JOINT_CONFIG_PATH);
  const auto simulate = tbl["simulate"].value<bool>().value();
  if (simulate) {
    logger->info("Simulation mode detected. Creating FakeArmHW...");
    m_impl = std::make_unique<detail::FakeArmHW>();
  } else {
    logger->info("Real hardware mode detected. Creating RealArmHW...");
    m_impl = std::make_unique<detail::RealArmHW>(can);
  }
}

ArmHW::~ArmHW() = default;

void ArmHW::read(common::ArmState &state) { m_impl->read(state); }

void ArmHW::write(const common::ArmCommand &cmd) { m_impl->write(cmd); }

void ArmHW::enable() { m_impl->enable(); }

void ArmHW::disable() { m_impl->disable(); }

void ArmHW::step(double dt) { m_impl->step(dt); }
} // namespace yandy::modules

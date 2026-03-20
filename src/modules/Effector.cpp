#include <one/PID/PidController.hpp>
#include <one/PID/PidParams.hpp>
#include <one/can/CanDriver.hpp>
#include <one/motor/dji/DjiMotor.hpp>
#include <one/motor/dji/DjiParam.hpp>
#include <one/motor/dm/DmParam.hpp>
#include <toml++/toml.hpp>
#include <yandy/core/Logger.hpp>
#include <yandy/modules/Effector.hpp>

#include <chrono>
#include <thread>

#define EFFECTOR_CONFIG_PATH YANDY_CONFIG_PATH "effector.toml"

namespace yandy::modules {

namespace {

one::pid::PidParams<> parsePidParams(const toml::table &tbl) {
  return one::pid::PidParams<>{
      .Kp = tbl["kp"].value<float>().value(),
      .Ki = tbl["ki"].value<float>().value(),
      .Kd = tbl["kd"].value<float>().value(),
      .MaxOutput = tbl["max_output"].value<float>().value(),
      .Deadband = tbl["deadband"].value<float>().value(),
      .IntegralLimit = tbl["integral_limit"].value<float>().value(),
  };
}

static one::motor::dji::Param g_zeroing_params = {
    .mode = one::motor::dji::AngMode{one::pid::PidParams{}}};

static one::motor::dji::Param g_normal_params = {
    .mode = one::motor::dji::AngMode{one::pid::PidParams{}}};

} // namespace

Effector::Effector(one::can::CanDriver &can) {
  m_logger = core::create_logger("Effector", spdlog::level::info);

  auto tbl = toml::parse_file(EFFECTOR_CONFIG_PATH);

  // 读取电机 ID
  const auto motorId = tbl["id"].value<uint8_t>().value();

  // 读取 PID 参数
  auto posPidParams = parsePidParams(*tbl["position_pid"].as_table());
  auto angPidParams = parsePidParams(*tbl["angular_pid"].as_table());

  // 读取回零参数
  m_zeroingCurrentMA = tbl["zeroing_current_mA"].value<float>().value();
  m_zeroingVelocityThreshold =
      tbl["zeroing_velocity_threshold"].value<float>().value();
  m_zeroingStallTimeMs = tbl["zeroing_stall_time_ms"].value<int>().value();

  // 读取软限位缓冲
  m_softLimitBuffer = tbl["soft_limit_buffer"].value<float>().value();

  // 读取夹持参数
  m_holdingMaxCurrentMA = tbl["holding_max_current_mA"].value<float>().value();

  // 读取碰撞检测参数
  m_collisionPosErrorThreshold =
      tbl["collision_pos_error_threshold"].value<float>().value();
  m_collisionCurrentRateThreshold =
      tbl["collision_current_rate_threshold"].value<float>().value();
  m_collisionVelocityDropThreshold =
      tbl["collision_velocity_drop_threshold"].value<float>().value();

  m_logger->info("配置已加载: motor_id={}, zeroing_current={:.0f}mA, "
                 "holding_max_current={:.0f}mA",
                 motorId, m_zeroingCurrentMA, m_holdingMaxCurrentMA);

  g_zeroing_params = {
      .id = motorId,
      .mode = one::motor::dji::AngMode{one::pid::PidParams<>{
          .Kp = 60, .Ki = 10, .Kd = 0, .MaxOutput = m_zeroingCurrentMA}}};
  g_normal_params = {
      .id = motorId,
      .mode = one::motor::dji::PosAngMode{
          one::motor::dji::PosAngMode{posPidParams, angPidParams}}};

  (void)m_motor.init(can, g_zeroing_params).or_else([this](auto &&e) {
    m_logger->critical("{}", e.message);
    throw std::runtime_error(e.message);
  });

  performZeroing();
}

void Effector::performZeroing() {
  m_logger->info("开始夹爪回零...");

  (void)m_motor.enable();

  int stallTimeMs = 0;
  constexpr int kLoopIntervalMs = 2;

  while (stallTimeMs < m_zeroingStallTimeMs) {
    auto status = m_motor.getStatusPlain();
    float velocity = status.reduced_angular_rad_s;

    // 给定恒定小电流让夹爪向外开
    m_motor.setTorRef(m_zeroingCurrentMA);

    if (std::abs(velocity) < m_zeroingVelocityThreshold) {
      stallTimeMs += kLoopIntervalMs;
    } else {
      stallTimeMs = 0;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(kLoopIntervalMs));
  }

  // 撞到物理限位，记录当前位置为 POS_MAX
  auto status = m_motor.getStatusPlain();
  m_posMax = status.reduced_angle_rad;
  m_posSoftMax = m_posMax - m_softLimitBuffer;

  m_logger->info("回零完成, POS_MAX = {:.4f} rad, 软限位 = {:.4f} rad",
                 m_posMax, m_posSoftMax);

  m_state = State::Idle;
  (void)m_motor.disable()
      .and_then([this] { return m_motor.setParam(g_normal_params); })
      .or_else([this](auto &&e) {
        m_logger->critical("设置参数失败：{}", e.message);
      });
  m_motor.setPosRef(m_posSoftMax);
}

void Effector::openClaw() {
  if (m_state == State::Opening) {
    return;
  }

  m_logger->info("张开夹爪");
  m_state = State::Opening;

  // 目标位置设在软限位，不撞击物理极限
  m_motor.setPosRef(m_posSoftMax);
}

void Effector::closeClaw() {
  if (m_state == State::Closing || m_state == State::Holding) {
    return;
  }

  m_logger->info("闭合夹爪");
  m_state = State::Closing;

  // 目标设为 0（全闭合），在 update() 中检测碰撞
  m_motor.setPosRef(0.0f);

  // 重置碰撞检测状态
  m_prevCurrent = 0.0f;
  m_prevVelocity = 0.0f;
  m_firstUpdateInClosing = true;
}

void Effector::update() {
  auto status = m_motor.getStatusPlain();

  switch (m_state) {
  case State::Opening: {
    float posError = std::abs(m_posSoftMax - status.reduced_angle_rad);
    if (posError < 0.05f) {
      m_state = State::Idle;
      m_logger->info("夹爪已张开");
    }
    break;
  }

  case State::Closing: {
    float currentPos = status.reduced_angle_rad;
    float currentVel = status.reduced_angular_rad_s;
    float currentMA = status.real_current_mA;

    // 首次进入时初始化
    if (m_firstUpdateInClosing) {
      m_prevCurrent = currentMA;
      m_prevVelocity = currentVel;
      m_firstUpdateInClosing = false;
      break;
    }

    float posError = std::abs(0.0f - currentPos);
    float currentRate = currentMA - m_prevCurrent;
    float velocityDrop = m_prevVelocity - currentVel;

    // 碰撞检测条件：位置误差大 && 电流急剧上升 && 速度急剧下降
    bool collision = (posError > m_collisionPosErrorThreshold) &&
                     (currentRate > m_collisionCurrentRateThreshold) &&
                     (velocityDrop > m_collisionVelocityDropThreshold);

    if (collision) {
      m_logger->info(
          "检测到物体，切换到保持模式 (pos={:.3f}, current={:.1f}mA)",
          currentPos, currentMA);
      enterHolding(currentPos);
    }

    m_prevCurrent = currentMA;
    m_prevVelocity = currentVel;
    break;
  }

  case State::Holding:
    // 保持模式下无需特殊处理，电流已被限幅
    break;

  case State::Idle:
  default:
    break;
  }
}

void Effector::enterHolding(float holdPosition) {
  m_state = State::Holding;

  // 将目标设为当前位置，锁定不动
  m_motor.setPosRef(holdPosition);

  m_logger->info("进入保持模式，保持位置 = {:.4f} rad", holdPosition);
}

void Effector::release() {
  m_logger->info("释放夹爪");
  openClaw();
}

bool Effector::isHolding() const { return m_state == State::Holding; }

bool Effector::isOpen() {
  if (m_state != State::Idle && m_state != State::Opening) {
    return false;
  }
  auto status = m_motor.getStatusPlain();
  return std::abs(m_posSoftMax - status.reduced_angle_rad) < 0.1f;
}

bool Effector::isClosed() {
  auto status = m_motor.getStatusPlain();
  return status.reduced_angle_rad < 0.1f;
}

Effector::State Effector::getState() const { return m_state; }

} // namespace yandy::modules

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

namespace detail {

// =============================================================================
// RealEffector - 真实硬件实现
// =============================================================================
class RealEffector : public IEffector {
public:
  explicit RealEffector(one::can::CanDriver &can) {
    m_logger = core::create_logger("RealEffector", spdlog::level::info);

    auto tbl = toml::parse_file(EFFECTOR_CONFIG_PATH);

    m_motorId = tbl["id"].value<uint8_t>().value();
    float kp = tbl["mit_pid"]["kp"].value<float>().value();
    float kd = tbl["mit_pid"]["kd"].value<float>().value();
    m_mitKp = kp;
    m_mitKd = kd;

    m_zeroingCurrentMA = tbl["zeroing_current_mA"].value<float>().value();
    m_zeroingVelocityThreshold =
        tbl["zeroing_velocity_threshold"].value<float>().value();
    m_zeroingStallTimeMs = tbl["zeroing_stall_time_ms"].value<int>().value();
    m_softLimitBuffer = tbl["soft_limit_buffer"].value<float>().value();
    m_holdingMaxCurrentMA =
        tbl["holding_max_current_mA"].value<float>().value();
    m_collisionPosErrorThreshold =
        tbl["collision_pos_error_threshold"].value<float>().value();
    m_collisionCurrentRateThreshold =
        tbl["collision_current_rate_threshold"].value<float>().value();
    m_collisionVelocityDropThreshold =
        tbl["collision_velocity_drop_threshold"].value<float>().value();

    m_logger->info("配置已加载: motor_id={}, MIT参数 kp={:.1f} kd={:.3f}, "
                   "zeroing_current={:.0f}mA",
                   m_motorId, kp, kd, m_zeroingCurrentMA);

    one::motor::dji::Param params = {
        .id = m_motorId,
        .mode = one::motor::dji::MITMode{kp, kd}};

    (void)m_motor.init(can, params).or_else([this](auto &&e) {
      m_logger->critical("{}", e.message);
      throw std::runtime_error(e.message);
    });

    performZeroing();
  }

  void openClaw() override {
    if (m_state == State::Opening) {
      return;
    }
    m_logger->info("张开夹爪");
    m_state = State::Opening;
    m_motor.setPosRef(m_posSoftMax);
  }

  void closeClaw() override {
    if (m_state == State::Closing || m_state == State::Holding) {
      return;
    }
    m_logger->info("闭合夹爪");
    m_state = State::Closing;
    m_motor.setPosRef(0.0f);
    m_prevCurrent = 0.0f;
    m_prevVelocity = 0.0f;
    m_firstUpdateInClosing = true;
  }

  void update() override {
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

      if (m_firstUpdateInClosing) {
        m_prevCurrent = currentMA;
        m_prevVelocity = currentVel;
        m_firstUpdateInClosing = false;
        break;
      }

      float posError = std::abs(0.0f - currentPos);
      float currentRate = currentMA - m_prevCurrent;
      float velocityDrop = m_prevVelocity - currentVel;

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
    case State::Idle:
    default:
      break;
    }
  }

  void release() override {
    m_logger->info("释放夹爪");
    openClaw();
  }

  [[nodiscard]] bool isHolding() const override {
    return m_state == State::Holding;
  }

  [[nodiscard]] bool isOpen() override {
    if (m_state != State::Idle && m_state != State::Opening) {
      return false;
    }
    auto status = m_motor.getStatusPlain();
    return std::abs(m_posSoftMax - status.reduced_angle_rad) < 0.1f;
  }

  [[nodiscard]] bool isClosed() override {
    auto status = m_motor.getStatusPlain();
    return status.reduced_angle_rad < 0.1f;
  }

  [[nodiscard]] State getState() const override { return m_state; }

private:
  void performZeroing() {
    m_logger->info("开始夹爪回零...");

    (void)m_motor.enable();

    int stallTimeMs = 0;
    constexpr int kLoopIntervalMs = 2;

    while (stallTimeMs < m_zeroingStallTimeMs) {
      auto status = m_motor.getStatusPlain();
      float velocity = status.reduced_angular_rad_s;
      m_motor.setTorRef(m_zeroingCurrentMA);

      if (std::abs(velocity) < m_zeroingVelocityThreshold) {
        stallTimeMs += kLoopIntervalMs;
      } else {
        stallTimeMs = 0;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(kLoopIntervalMs));
    }

    auto status = m_motor.getStatusPlain();
    m_posMax = status.reduced_angle_rad;
    m_posSoftMax = m_posMax - m_softLimitBuffer;

    m_logger->info("回零完成, POS_MAX = {:.4f} rad, 软限位 = {:.4f} rad",
                   m_posMax, m_posSoftMax);

    m_state = State::Idle;

    // MIT 模式无需切换参数
    m_motor.setPosRef(m_posSoftMax);
  }

  void enterHolding(float holdPosition) {
    m_state = State::Holding;
    m_motor.setPosRef(holdPosition);
    m_logger->info("进入保持模式，保持位置 = {:.4f} rad", holdPosition);
  }

  one::motor::dji::M2006 m_motor;
  std::shared_ptr<spdlog::logger> m_logger;

  uint8_t m_motorId = 8;
  float m_mitKp = 50.0f;
  float m_mitKd = 0.1f;

  State m_state = State::Idle;
  float m_posMax = 0.0f;
  float m_posSoftMax = 0.0f;

  float m_prevCurrent = 0.0f;
  float m_prevVelocity = 0.0f;
  bool m_firstUpdateInClosing = true;

  float m_zeroingCurrentMA = 1000.0f;
  float m_zeroingVelocityThreshold = 0.05f;
  int m_zeroingStallTimeMs = 100;
  float m_softLimitBuffer = 0.1f;
  float m_holdingMaxCurrentMA = 3000.0f;
  float m_collisionPosErrorThreshold = 0.3f;
  float m_collisionCurrentRateThreshold = 50.0f;
  float m_collisionVelocityDropThreshold = 0.1f;
};

// =============================================================================
// FakeEffector - 仿真实现
// =============================================================================
class FakeEffector : public IEffector {
public:
  FakeEffector() {
    m_logger = core::create_logger("FakeEffector", spdlog::level::info);

    auto tbl = toml::parse_file(EFFECTOR_CONFIG_PATH);

    m_softLimitBuffer = tbl["soft_limit_buffer"].value<float>().value();
    m_posMax = tbl["sim_pos_max"].value<float>().value();
    m_moveSpeed = tbl["sim_move_speed"].value<float>().value();

    m_posSoftMax = m_posMax - m_softLimitBuffer;
    m_currentPos = m_posSoftMax; // 初始位置为张开状态

    m_logger->info("仿真模式初始化: POS_MAX={:.2f}, 软限位={:.2f}, 移动速度={:.2f}",
                   m_posMax, m_posSoftMax, m_moveSpeed);
  }

  void openClaw() override {
    if (m_state == State::Opening) {
      return;
    }
    m_logger->info("张开夹爪 (仿真)");
    m_state = State::Opening;
    m_targetPos = m_posSoftMax;
  }

  void closeClaw() override {
    if (m_state == State::Closing || m_state == State::Holding) {
      return;
    }
    m_logger->info("闭合夹爪 (仿真)");
    m_state = State::Closing;
    m_targetPos = 0.0f;
  }

  void update() override {
    constexpr float dt = 0.004f; // 250Hz

    switch (m_state) {
    case State::Opening: {
      float diff = m_targetPos - m_currentPos;
      if (std::abs(diff) < 0.05f) {
        m_currentPos = m_targetPos;
        m_state = State::Idle;
        m_logger->info("夹爪已张开 (仿真)");
      } else {
        float step = m_moveSpeed * dt;
        m_currentPos += (diff > 0 ? step : -step);
      }
      break;
    }

    case State::Closing: {
      float diff = m_targetPos - m_currentPos;
      if (std::abs(diff) < 0.05f) {
        // 仿真中随机触发碰撞（用于测试）
        if (m_currentPos > 0.3f) {
          m_logger->info("检测到物体 (仿真), 切换到保持模式");
          m_state = State::Holding;
        } else {
          m_currentPos = m_targetPos;
          m_state = State::Idle;
          m_logger->info("夹爪已闭合 (仿真)");
        }
      } else {
        float step = m_moveSpeed * dt;
        m_currentPos += (diff > 0 ? step : -step);
      }
      break;
    }

    case State::Holding:
    case State::Idle:
    default:
      break;
    }
  }

  void release() override {
    m_logger->info("释放夹爪 (仿真)");
    openClaw();
  }

  [[nodiscard]] bool isHolding() const override {
    return m_state == State::Holding;
  }

  [[nodiscard]] bool isOpen() override {
    return (m_state == State::Idle || m_state == State::Opening) &&
           std::abs(m_posSoftMax - m_currentPos) < 0.1f;
  }

  [[nodiscard]] bool isClosed() override { return m_currentPos < 0.1f; }

  [[nodiscard]] State getState() const override { return m_state; }

private:
  std::shared_ptr<spdlog::logger> m_logger;

  State m_state = State::Idle;
  float m_posMax = 2.0f;
  float m_posSoftMax = 1.9f;
  float m_softLimitBuffer = 0.1f;
  float m_moveSpeed = 3.0f;

  float m_currentPos = 0.0f;
  float m_targetPos = 0.0f;
};

} // namespace detail

// =============================================================================
// Effector 外观类实现
// =============================================================================

Effector::Effector(one::can::CanDriver &can) {
  auto logger = core::create_logger("Effector", spdlog::level::info);
  logger->info("读取配置: {}", EFFECTOR_CONFIG_PATH);

  auto tbl = toml::parse_file(EFFECTOR_CONFIG_PATH);
  const auto simulate = tbl["simulate"].value<bool>().value();

  if (simulate) {
    logger->info("检测到仿真模式，创建 FakeEffector...");
    m_impl = std::make_unique<detail::FakeEffector>();
  } else {
    logger->info("检测到真实硬件模式，创建 RealEffector...");
    m_impl = std::make_unique<detail::RealEffector>(can);
  }
}

Effector::~Effector() = default;

void Effector::openClaw() { m_impl->openClaw(); }

void Effector::closeClaw() { m_impl->closeClaw(); }

void Effector::update() { m_impl->update(); }

void Effector::release() { m_impl->release(); }

bool Effector::isHolding() const { return m_impl->isHolding(); }

bool Effector::isOpen() { return m_impl->isOpen(); }

bool Effector::isClosed() { return m_impl->isClosed(); }

Effector::State Effector::getState() const { return m_impl->getState(); }

} // namespace yandy::modules

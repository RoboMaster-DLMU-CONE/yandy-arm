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
    m_dir = tbl["dir"].value<float>().value();
    float kp = tbl["mit_pid"]["kp"].value<float>().value();
    float kd = tbl["mit_pid"]["kd"].value<float>().value();
    m_mitKp = kp;
    m_mitKd = kd;
    m_mitFeedforwardTorque = tbl["mit_feedforward_torque"].value<float>().value();

    m_zeroingTorque = tbl["zeroing_torque"].value<float>().value();
    m_zeroingVelocityThreshold =
        tbl["zeroing_velocity_threshold"].value<float>().value();
    m_zeroingStallTimeMs = tbl["zeroing_stall_time_ms"].value<int>().value();
    m_openPosition = tbl["open_position"].value<float>().value();
    m_collisionCurrentThreshold =
        tbl["collision_current_threshold"].value<float>().value();

    m_logger->info("配置已加载: motor_id={}, MIT参数 kp={:.1f} kd={:.3f}, "
                   "zeroing_torque={:.2f}N·m",
                   m_motorId, kp, kd, m_zeroingTorque);

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
    m_motor.setPosRef(m_openPosition + m_offset);
    m_motor.setTorRef(m_mitFeedforwardTorque);
  }

  void closeClaw() override {
    if (m_state == State::Closing || m_state == State::Holding) {
      return;
    }
    m_logger->info("闭合夹爪");
    m_state = State::Closing;
    m_motor.setPosRef(0.0f + m_offset);
    m_motor.setTorRef(m_mitFeedforwardTorque);
    m_prevCurrent = 0.0f;
    m_firstUpdateInClosing = true;
  }

  void update() override {
    auto status = m_motor.getStatusPlain();

    switch (m_state) {
    case State::Opening: {
      float posError = std::abs((m_openPosition + m_offset) - status.reduced_angle_rad);
      if (posError < 0.05f) {
        m_state = State::Idle;
        m_logger->info("夹爪已张开");
      }
      break;
    }

    case State::Closing: {
      float currentPos = status.reduced_angle_rad;
      float currentMA = status.real_current_mA;

      if (m_firstUpdateInClosing) {
        m_prevCurrent = currentMA;
        m_firstUpdateInClosing = false;
        break;
      }

      // 碰撞检测：电流超过阈值
      bool collision = std::abs(currentMA) > m_collisionCurrentThreshold;

      if (collision) {
        m_logger->info(
            "检测到物体，切换到保持模式 (pos={:.3f}, current={:.1f}mA)",
            currentPos, currentMA);
        enterHolding(currentPos);
      }

      m_prevCurrent = currentMA;
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
    return std::abs((m_openPosition + m_offset) - status.reduced_angle_rad) < 0.1f;
  }

  [[nodiscard]] bool isClosed() override {
    auto status = m_motor.getStatusPlain();
    return std::abs(m_offset - status.reduced_angle_rad) < 0.1f;
  }

  [[nodiscard]] State getState() const override { return m_state; }

private:
  void performZeroing() {
    m_logger->info("开始夹爪回零...");

    (void)m_motor.enable();

    // 记录初始位置
    auto initialStatus = m_motor.getStatusPlain();
    float initialPos = initialStatus.reduced_angle_rad;

    int stallTimeMs = 0;
    constexpr int kLoopIntervalMs = 2;

    // 往正方向（闭合）施加力矩直到堵转
    m_motor.setPosRef(10.0f);
    m_motor.setTorRef(m_zeroingTorque);

    while (stallTimeMs < m_zeroingStallTimeMs) {
      auto status = m_motor.getStatusPlain();
      float velocity = status.reduced_angular_rad_s;

      if (std::abs(velocity) < m_zeroingVelocityThreshold) {
        stallTimeMs += kLoopIntervalMs;
      } else {
        stallTimeMs = 0;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(kLoopIntervalMs));
    }

    auto status = m_motor.getStatusPlain();
    float zeroPos = status.reduced_angle_rad;
    m_offset = zeroPos - initialPos;

    m_logger->info("回零完成, 初始位置={:.4f} rad, 零点={:.4f} rad, offset={:.4f} rad",
                   initialPos, zeroPos, m_offset);

    m_state = State::Idle;

    // 张开到限位位置
    m_motor.setPosRef(m_openPosition + m_offset);
    m_motor.setTorRef(m_mitFeedforwardTorque);
  }

  void enterHolding(float holdPosition) {
    m_state = State::Holding;
    m_motor.setPosRef(holdPosition);
    m_motor.setTorRef(m_mitFeedforwardTorque);
    m_logger->info("进入保持模式，保持位置 = {:.4f} rad", holdPosition);
  }

  one::motor::dji::M2006 m_motor;
  std::shared_ptr<spdlog::logger> m_logger;

  uint8_t m_motorId = 8;
  float m_dir = -1.0f;
  float m_mitKp = 5.0f;
  float m_mitKd = 0.5f;
  float m_mitFeedforwardTorque = 0.2f;

  State m_state = State::Idle;
  float m_offset = 0.0f;        // 校准偏移量
  float m_openPosition = 0.5f;  // 张开位置

  float m_prevCurrent = 0.0f;
  bool m_firstUpdateInClosing = true;

  float m_zeroingTorque = 0.18f;
  float m_zeroingVelocityThreshold = 0.05f;
  int m_zeroingStallTimeMs = 100;
  float m_collisionCurrentThreshold = 3000.0f;
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

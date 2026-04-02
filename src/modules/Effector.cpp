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
    m_mitFeedforwardTorque =
        tbl["mit_feedforward_torque"].value<float>().value();
    m_holdingTorque = tbl["holding_torque"].value<float>().value();

    m_zeroingTorque = tbl["zeroing_torque"].value<float>().value();
    m_zeroingVelocityThreshold =
        tbl["zeroing_velocity_threshold"].value<float>().value();
    m_zeroingStallTimeMs = tbl["zeroing_stall_time_ms"].value<int>().value();
    m_stroke = tbl["stroke"].value<float>().value();
    m_collisionCurrentThreshold =
        tbl["collision_current_threshold"].value<float>().value();

    m_logger->info("配置已加载: motor_id={}, MIT参数 kp={:.1f} kd={:.3f}, "
                   "zeroing_torque={:.2f}N·m",
                   m_motorId, kp, kd, m_zeroingTorque);

    one::motor::dji::Param params = {.id = m_motorId,
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
    m_motor.setPosRef(m_offset - m_dir * m_stroke);
    m_motor.setTorRef(-m_dir * m_mitFeedforwardTorque);
  }

  void closeClaw() override {
    if (m_state == State::Closing || m_state == State::Holding) {
      return;
    }
    m_logger->info("闭合夹爪 (目标: 超过原零点 0.2 rad)");
    m_state = State::Closing;

    // 目标位置: 往闭合方向（m_dir）多走 0.2 rad
    float over_close_pos = m_offset + m_dir * 0.2f;
    m_motor.setPosRef(over_close_pos);
    m_motor.setTorRef(m_dir * m_mitFeedforwardTorque);

    m_prevCurrent = 0.0f;
    m_firstUpdateInClosing = true;
  }

  void update() override {
    auto status = m_motor.getStatusPlain();

    switch (m_state) {
    case State::Opening: {
      float targetPos = m_offset - m_dir * m_stroke;
      float posError = std::abs(targetPos - status.reduced_angle_rad);
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
        m_closingStartTicks = 0;
        break;
      }

      m_closingStartTicks++;

      // 正常闭合到达目标判断 (超过原零点 0.2 rad)
      float over_close_pos = m_offset + m_dir * 0.2f;
      float posError = std::abs(over_close_pos - currentPos);
      if (posError < 0.05f) {
        m_state = State::Idle;
        m_logger->info("夹爪已完全闭合 (未检测到物体)");

        // 可选：如果到达了这个极限位置，也许这里就是新的物理极限，
        // 我们也可以把这里当做新的零点，但目前用户需求是“检测到电流加大”时更新，
        // 如果能走到这里说明没有碰壁。我们可以暂时什么都不做，或者也可以更新零点。
        // 根据“如果在零点之后检测到电流加大，那就把这个新的点作为新的零点更新”，
        // 这里没有电流加大，所以仅仅只是走到了设定最大位置。
        break;
      }

      // 启动防抖：前 25 帧（约 100ms，假设 250Hz）不进行碰撞检测
      if (m_closingStartTicks > 25) {
        // 碰撞检测：电流超过阈值 且 速度极小
        bool collision = std::abs(currentMA) > m_collisionCurrentThreshold &&
                         std::abs(status.reduced_angular_rad_s) <
                             m_zeroingVelocityThreshold * 2.0f;

        if (collision) {
          // 判断碰撞位置是否在原零点之后
          // 因为 m_dir 是闭合方向，所以 m_dir * (currentPos - m_offset)
          // 如果大于 0，说明超过了原零点 举例：m_dir = -1, m_offset =
          // 0。currentPos = -0.1，则 -1 * (-0.1 - 0) = 0.1 > 0
          bool is_past_zero = (m_dir * (currentPos - m_offset) > 0.0f);

          if (is_past_zero) {
            m_logger->info("在原零点之后检测到物体，更新零点并切换到保持模式 "
                           "(pos={:.3f}, 原零点={:.3f}, current={:.1f}mA)",
                           currentPos, m_offset, currentMA);

            // 更新新的零点
            m_offset = currentPos;

            // 直接在当前位置保持，依靠 holdingTorque 提供纯粹的握力，避免 PD 控制器产生反向内耗
            enterHolding(currentPos);
          } else {
            m_logger->info("在原零点之前检测到物体，切换到保持模式 "
                           "(pos={:.3f}, 原零点={:.3f}, current={:.1f}mA)",
                           currentPos, m_offset, currentMA);
            enterHolding(currentPos);
          }
        }
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
    auto status = m_motor.getStatusPlain();
    float targetPos = m_offset - m_dir * m_stroke;
    float posError = std::abs(targetPos - status.reduced_angle_rad);
    // 只要距离张开位置足够近，或者正在张开途中，都可以视为“非闭合”状态
    // 但为了 Toggle 准确，这里判断是否“已经处于”张开位置
    return posError < 0.15f; 
  }

  [[nodiscard]] bool isClosed() override {
    if (m_state == State::Holding) {
      return true;
    }
    auto status = m_motor.getStatusPlain();
    // 判断是否在零点附近，或者已经越过了零点（往闭合方向走得更远）
    float relativePos = m_dir * (status.reduced_angle_rad - m_offset);
    return relativePos > -0.05f; // 只要不是在张开方向 0.05rad 之外，就认为关上了
  }

  [[nodiscard]] State getState() const override { return m_state; }

private:
  void performZeroing() {
    m_logger->info("开始夹爪回零 (目标: 张开极限)...");

    (void)m_motor.enable();

    int stallTimeMs = 0;
    constexpr int kLoopIntervalMs = 2;

    // 确定张开方向。
    // openClaw 的目标是 m_offset - m_dir * m_stroke
    // 所以从 m_offset 出发，张开方向就是 -m_dir * m_stroke 的方向
    float zeroingDir = (m_stroke >= 0) ? -m_dir : m_dir;

    m_logger->info("回零运动方向: {}", zeroingDir > 0 ? "正向" : "反向");

    // 往张开方向平滑施加位置指令和恒定力矩，避免位置误差过大导致瞬间冲击
    float current_target = m_motor.getStatusPlain().reduced_angle_rad;
    float search_speed = 2.0f; // 寻找限位的速度 (rad/s)
    float step = search_speed * (kLoopIntervalMs / 1000.0f);

    while (stallTimeMs < m_zeroingStallTimeMs) {
      current_target += zeroingDir * step;
      m_motor.setPosRef(current_target);
      m_motor.setTorRef(zeroingDir * m_zeroingTorque);

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
    float openLimitPos = status.reduced_angle_rad;

    // 重新计算 m_offset (闭合零点)。
    // 根据公式: openLimitPos = m_offset - m_dir * m_stroke
    // 推导得: m_offset = openLimitPos + m_dir * m_stroke
    m_offset = openLimitPos + m_dir * m_stroke;

    m_logger->info("回零完成, 张开极限位置={:.4f} rad, "
                   "计算出的闭合零点(offset)={:.4f} rad",
                   openLimitPos, m_offset);

    // 回零后停在当前的张开极限位置，避免突发运动
    m_motor.setPosRef(openLimitPos);
    m_motor.setTorRef(0);
  }

  void enterHolding(float holdPosition) {
    m_state = State::Holding;
    m_motor.setPosRef(holdPosition);
    m_motor.setTorRef(m_dir * m_holdingTorque);
    m_logger->info("进入保持模式，保持位置 = {:.4f} rad", holdPosition);
  }

  one::motor::dji::M2006 m_motor;
  std::shared_ptr<spdlog::logger> m_logger;

  uint8_t m_motorId = 8;
  float m_dir = -1.0f;
  float m_mitKp = 5.0f;
  float m_mitKd = 0.5f;
  float m_mitFeedforwardTorque = 0.2f;
  float m_holdingTorque = 1.0f;

  State m_state = State::Idle;
  float m_offset = 0.0f; // 校准偏移量
  float m_stroke = 0.5f; // 行程 (从张开极限到闭合零点的距离)

  float m_prevCurrent = 0.0f;
  bool m_firstUpdateInClosing = true;
  int m_closingStartTicks = 0;

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

    m_posMax = tbl["sim_pos_max"].value<float>().value();
    m_moveSpeed = tbl["sim_move_speed"].value<float>().value();

    m_posSoftMax = m_posMax - m_softLimitBuffer;
    m_currentPos = m_posSoftMax; // 初始位置为张开状态

    m_logger->info(
        "仿真模式初始化: POS_MAX={:.2f}, 软限位={:.2f}, 移动速度={:.2f}",
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

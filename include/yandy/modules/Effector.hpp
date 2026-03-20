#ifndef YANDY_ARM_EFFECTOR_HPP
#define YANDY_ARM_EFFECTOR_HPP

#include <one/motor/dji/DjiMotor.hpp>
#include <spdlog/logger.h>

namespace one::can {
class CanDriver;
}
namespace one::motor {
class IMotor;
}

namespace yandy::modules {

/**
 * @brief 末端执行器（夹爪）控制模块
 *
 * 功能：
 * - 构造时自动回零：给定小电流向外开，撞限位后记录 POS_MAX
 * - openClaw(): 快速张开到软限位 (POS_MAX - 缓冲)
 * - closeClaw(): 快速闭合，检测碰撞后自动进入 Holding 状态
 * - update(): 每控制周期调用，处理碰撞检测和状态转换
 */
class Effector {
public:
  enum class State {
    Idle,    // 空闲
    Opening, // 正在张开
    Closing, // 正在闭合
    Holding  // 夹持保持
  };

  explicit Effector(one::can::CanDriver &can);

  /// 张开夹爪（移动到软限位）
  void openClaw();

  /// 闭合夹爪（快速闭合，碰撞后自动保持）
  void closeClaw();

  /// 每控制周期调用，处理状态机和碰撞检测
  void update();

  /// 释放物体（等同于 openClaw）
  void release();

  /// 查询是否处于保持状态
  [[nodiscard]] bool isHolding() const;

  /// 查询夹爪是否已张开
  [[nodiscard]] bool isOpen();

  /// 查询夹爪是否已闭合
  [[nodiscard]] bool isClosed();

  /// 获取当前状态
  [[nodiscard]] State getState() const;

private:
  void performZeroing();
  void enterHolding(float holdPosition);

  one::motor::dji::M2006 m_motor;
  std::shared_ptr<spdlog::logger> m_logger;

  State m_state = State::Idle;

  float m_posMax = 0.0f;     // 物理极限位置 (回零时测得)
  float m_posSoftMax = 0.0f; // 软限位 (POS_MAX - 缓冲)

  // 碰撞检测用的历史数据
  float m_prevCurrent = 0.0f;
  float m_prevVelocity = 0.0f;
  bool m_firstUpdateInClosing = true;

  // 从配置文件读取的参数
  float m_zeroingCurrentMA = 1000.0f;
  float m_zeroingVelocityThreshold = 0.05f;
  int m_zeroingStallTimeMs = 100;
  float m_softLimitBuffer = 0.1f;
  float m_holdingMaxCurrentMA = 3000.0f;
  float m_collisionPosErrorThreshold = 0.3f;
  float m_collisionCurrentRateThreshold = 50.0f;
  float m_collisionVelocityDropThreshold = 0.1f;
};

} // namespace yandy::modules

#endif // YANDY_ARM_EFFECTOR_HPP

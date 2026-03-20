#ifndef YANDY_ARM_EFFECTOR_HPP
#define YANDY_ARM_EFFECTOR_HPP

#include <memory>
#include "IEffector.hpp"

namespace one::can {
class CanDriver;
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
 *
 * 根据 config/effector.toml 中的 simulate 字段选择实际硬件或仿真实现
 */
class Effector {
public:
  using State = IEffector::State;

  explicit Effector(one::can::CanDriver &can);
  ~Effector();

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
  std::unique_ptr<IEffector> m_impl;
};

} // namespace yandy::modules

#endif // YANDY_ARM_EFFECTOR_HPP

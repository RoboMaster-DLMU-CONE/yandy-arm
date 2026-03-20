#ifndef YANDY_ARM_IEFFECTOR_HPP
#define YANDY_ARM_IEFFECTOR_HPP

namespace yandy::modules {

/**
 * @brief 末端执行器接口
 */
class IEffector {
public:
  enum class State {
    Idle,    // 空闲
    Opening, // 正在张开
    Closing, // 正在闭合
    Holding  // 夹持保持
  };

  virtual ~IEffector() = default;

  /// 张开夹爪
  virtual void openClaw() = 0;

  /// 闭合夹爪
  virtual void closeClaw() = 0;

  /// 每控制周期调用
  virtual void update() = 0;

  /// 释放物体
  virtual void release() = 0;

  /// 查询是否处于保持状态
  [[nodiscard]] virtual bool isHolding() const = 0;

  /// 查询夹爪是否已张开
  [[nodiscard]] virtual bool isOpen() = 0;

  /// 查询夹爪是否已闭合
  [[nodiscard]] virtual bool isClosed() = 0;

  /// 获取当前状态
  [[nodiscard]] virtual State getState() const = 0;
};

} // namespace yandy::modules

#endif // YANDY_ARM_IEFFECTOR_HPP

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
class Effector {
public:
  explicit Effector(one::can::CanDriver &can);
  void closeClaw();
  void openClaw();

private:
  one::motor::dji::M2006 motor;
  std::shared_ptr<spdlog::logger> m_logger;
};
} // namespace yandy::modules

#endif // YANDY_ARM_EFFECTOR_HPP

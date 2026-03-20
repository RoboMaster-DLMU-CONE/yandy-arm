#ifndef YANDY_ARM_ARM_HPP
#define YANDY_ARM_ARM_HPP

#include "IArmHW.hpp"
#include <memory>
#include <yandy/common/Types.hpp>

namespace one::can {
class CanDriver;
}

namespace yandy::modules {
class ArmHW {
public:
  explicit ArmHW(one::can::CanDriver &can);
  ~ArmHW();

  void read(common::ArmState &state);
  void write(const common::ArmCommand &cmd);
  void enable();
  void disable();

  // Extra method for simulation stepping
  void step(double dt);

private:
  std::unique_ptr<IArmHW> m_impl;
};
} // namespace yandy::modules

#endif // YANDY_ARM_ARM_HPP

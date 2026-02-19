#ifndef YANDY_ARM_ARM_HPP
#define YANDY_ARM_ARM_HPP

#include <memory>
#include <one/can/CanDriver.hpp>
#include <yandy/common/Types.hpp>
#include "IArmHW.hpp"

namespace yandy::modules
{
    class ArmHW
    {
    public:
        ArmHW();

        void read(common::JointState& state);
        void write(const common::JointCommand& cmd);
        void enable();
        void disable();

        // Extra method for simulation stepping
        void step(double dt);

    private:
        std::unique_ptr<IArmHW> m_impl;
        std::unique_ptr<one::can::CanDriver> m_can_driver;
    };
}

#endif //YANDY_ARM_ARM_HPP

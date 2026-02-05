#ifndef YANDY_ARM_ARM_HPP
#define YANDY_ARM_ARM_HPP

#include <array>
#include <memory>

#include <OneMotor/Motor/IMotor.hpp>

namespace yandy::modules
{
    class Arm
    {

    public:
        Arm();
    private:
        std::array<std::unique_ptr<OneMotor::Motor::IMotor>, 5> motors;
    };
}

#endif //YANDY_ARM_ARM_HPP
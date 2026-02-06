#include <OneMotor/Can/CanDriver.hpp>
#include <yandy/core/Logger.hpp>
#include <yandy/modules/ArmHW.hpp>

#include "yandy/modules/DynamicsSolver.hpp"

int main()
{
    yandy::core::init_logging();

    spdlog::info("Yandy Logger initialized.");

    OneMotor::Can::CanDriver driver("can0");
    yandy::modules::ArmHW arm_hw(driver);
    yandy::modules::DynamicsSolver solver;

    return 0;
}

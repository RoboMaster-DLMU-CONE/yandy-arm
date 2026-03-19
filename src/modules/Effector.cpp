#include <one/PID/PidParams.hpp>
#include <one/can/CanDriver.hpp>
#include <one/motor/dji/DjiMotor.hpp>
#include <one/motor/dji/DjiParam.hpp>
#include <one/motor/dm/DmParam.hpp>
#include <yandy/core/Logger.hpp>
#include <yandy/modules/Effector.hpp>

static constexpr one::pid::PidParams<> g_pos_pid_param{
    .Kp = 20,
    .Ki = 0,
    .Kd = 0,
};
static constexpr one::pid::PidParams<> g_ang_pid_param{
    .Kp = 20,
    .Ki = 0,
    .Kd = 0,
};

namespace yandy::modules {
Effector::Effector(one::can::CanDriver &can) {

  m_logger = core::create_logger("Effector", spdlog::level::info);
  (void)motor
      .init(can, one::motor::dji::Param{8,
                                        one::motor::dji::PosAngMode{
                                            g_pos_pid_param, g_ang_pid_param}})
      .or_else([this](auto &&e) {
        m_logger->critical("{}", e.message);
        throw std::runtime_error(e.message);
      });
}

void Effector::closeClaw() {}

void Effector::openClaw() {}
} // namespace yandy::modules

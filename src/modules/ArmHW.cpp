#include <yandy/modules/ArmHW.hpp>
#include <yandy/core/Logger.hpp>

#include <one/motor/dm/DmMotor.hpp>

#include <one/motor/dji/DjiMotor.hpp>
#include <one/can/CanDriver.hpp>

using one::pid::PidParams;
using one::pid::PidController;
using one::pid::PidChain;
using namespace one::motor;
using dji::GM6020_Voltage;
using dji::M3508;
using dm::J4310;
using dm::J4340;
using dm::J8009;
using dm::J10010L;
using one::can::CanDriver;
using std::make_unique;

using namespace one::motor::units::literals;

#define JOINT_CONFIG_PATH YANDY_CONFIG_PATH "joint.toml"

namespace yandy::modules
{
    ArmHW::ArmHW(CanDriver& driver) : m_driver(driver)
    {
        m_logger = core::create_logger("ArmHW", spdlog::level::info);
        parse_config();
    }

    void ArmHW::read(common::JointState& state)
    {
        for (int i = 0; i < common::JOINT_NUM; ++i)
        {
            double raw_pos; // rad
            double raw_vel; // rad/s
            double raw_tau{}; // Nm
            auto status = m_motors[i]->getStatusVariant().value();
            std::visit(
                [&]<typename T>(const T& payload)
                {
                    using StatusType = std::decay_t<T>;
                    if constexpr (std::is_same_v<StatusType, one::motor::dm::MotorStatus>)
                    {
                        raw_pos = payload.position.numerical_value_in(rad);
                        raw_vel = payload.velocity.numerical_value_in(rad / s);
                        raw_tau = payload.torque.numerical_value_in(N * m);
                    }
                    else
                    {
                        raw_pos = payload.reduced_angle.numerical_value_in(rad);
                        raw_vel = payload.reduced_angular.numerical_value_in(rad / s);
                    }
                },
                status);

            // q_model = Dir * (q_motor - Offset)
            state.q[i] = m_dirs[i] * (raw_pos - m_offsets[i]);

            // v_model = Dir * v_motor
            state.v[i] = m_dirs[i] * raw_vel;

            // tau_model = Dir * tau_motor
            state.tau[i] = m_dirs[i] * raw_tau;
        }
    }

    void ArmHW::write(const common::JointCommand& cmd)
    {
        for (int i = 0; i < common::JOINT_NUM; ++i)
        {
            double target_pos = (m_dirs[i] * cmd.q_des[i]) + m_offsets[i];
            double target_vel = m_dirs[i] * cmd.v_des[i];
            double ff_torque = m_dirs[i] * cmd.tau_ff[i];

            (void)m_motors[i]->setUnitRefs(target_pos * rad, target_vel * rad / s, ff_torque * N * m);
        }
    }

    void ArmHW::enable()
    {
        m_logger->info("Enabling all Arm motors...");
        for (int i = 0; i < common::JOINT_NUM; ++i)
        {
            if (auto res = m_motors[i]->enable(); !res.has_value())
            {
                m_logger->error("Failed to enable motor {}: {}", i + 1, res.error().message);
            }
        }
    }

    void ArmHW::disable()
    {
        m_logger->info("Disabling all Arm motors...");
        for (int i = 0; i < common::JOINT_NUM; ++i)
        {
            if (auto res = m_motors[i]->disable(); !res.has_value())
            {
                m_logger->error("Failed to enable motor {}: {}", i + 1, res.error().message);
            }
        }
    }

    void ArmHW::parse_config()
    {
        m_logger->info("Initializing Arm Hardware, loading configs from: {}...", JOINT_CONFIG_PATH);

        auto tbl = toml::parse_file(JOINT_CONFIG_PATH);
        parse_dm_motor(tbl["joint_1"], 0);
        parse_dm_motor(tbl["joint_2"], 1);
        parse_dm_motor(tbl["joint_3"], 2);
        parse_dji_motor(tbl["joint_4"], 3);
        parse_dm_motor(tbl["joint_5"], 4);
    }

    void ArmHW::parse_dm_motor(const toml::v3::node_view<toml::v3::node> joint_node, const size_t joint_index)
    {
        uint16_t can_id{}, master_id{};
        float kp{}, kd{};
        can_id = joint_node["can_id"].value<uint16_t>().value();
        master_id = joint_node["master_id"].value<uint16_t>().value();
        m_dirs[joint_index] = joint_node["dir"].value<float>().value();
        m_offsets[joint_index] = joint_node["offset"].value<float>().value();

        kp = joint_node["mit_pid"]["kp"].value<float>().value();
        kd = joint_node["mit_pid"]["kd"].value<float>().value();
        switch (joint_index)
        {
        case 0:
            // J1 : 4340
            m_motors[joint_index] = std::make_unique<
                J4340>(m_driver, dm::Param{can_id, master_id, dm::MITMode{kp, kd}});
        case 1:
            // J2 : 10010L
            m_motors[joint_index] = std::make_unique<J10010L>(m_driver, dm::Param{
                                                                  can_id, master_id, dm::MITMode{kp, kd}
                                                              });
        case 2:
            // J3 : 8009
            m_motors[joint_index] = std::make_unique<
                J8009>(m_driver, dm::Param{can_id, master_id, dm::MITMode{kp, kd}});
        case 4:
            // J5 : 4310
            m_motors[joint_index] = std::make_unique<
                J4310>(m_driver, dm::Param{can_id, master_id, dm::MITMode{kp, kd}});
        default: ;
        }
        m_logger->info(
            "J{} DM motor parsed: can_id: {}, master_id: {}, dir: {}, offset: {}, MIT params: {}, {}",
            joint_index + 1, can_id, master_id,
            m_dirs[joint_index], m_offsets[joint_index], kp, kd);
    }

    void ArmHW::parse_dji_motor(toml::v3::node_view<toml::v3::node> joint_node, size_t joint_index)
    {
        float kp{}, kd{};
        const uint8_t id = joint_node["id"].value<uint8_t>().value();
        m_dirs[joint_index] = joint_node["dir"].value<float>().value();
        m_offsets[joint_index] = joint_node["offset"].value<float>().value();
        kp = joint_node["mit_pid"]["kp"].value<float>().value();
        kd = joint_node["mit_pid"]["kd"].value<float>().value();

        m_motors[joint_index] = std::make_unique<GM6020_Voltage>(m_driver, dji::Param{id, dji::MITMode{kp, kd}});

        m_logger->info(
            "J{} DJI motor parsed: dir: {}, offset: {}",
            joint_index + 1, m_dirs[joint_index], m_offsets[joint_index]);
    }
}

#include <yandy/modules/ArmHW.hpp>
#include <yandy/core/Logger.hpp>

#include <OneMotor/Motor/DJI/DjiMotor.hpp>
#include <OneMotor/Motor/DM/DmMotor.hpp>
#include <one/PID/PidChain.hpp>
#include <one/PID/PidConfig.hpp>

using one::pid::PidParams;
using one::pid::PidConfig;
using one::pid::PidController;
using one::pid::PidChain;
using OneMotor::Motor::DJI::GM6020_Voltage;
using OneMotor::Motor::DJI::M3508;
using OneMotor::Motor::DM::J4310_MIT;
using OneMotor::Motor::DM::J4340_MIT;
using OneMotor::Motor::DM::J8009_MIT;
using OneMotor::Motor::DM::J10010L_MIT;
using OneMotor::Can::CanDriver;
using OneMotor::Motor::DJI::PIDFeatures;
using std::make_unique;

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
                    if constexpr (std::is_same_v<StatusType, OneMotor::Motor::DM::DmStatus>)
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

            (void)m_motors[i]->setRefs(target_pos * rad, target_vel * rad / s, ff_torque * N * m);
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
        float kp{}, ki{}, kd{};
        can_id = joint_node["can_id"].value<uint16_t>().value();
        master_id = joint_node["master_id"].value<uint16_t>().value();
        m_dirs[joint_index] = joint_node["dir"].value<float>().value();
        m_offsets[joint_index] = joint_node["offset"].value<float>().value();
        switch (joint_index)
        {
        case 0:
            // J1 : 4340
            m_motors[joint_index] = std::make_unique<J4340_MIT>(m_driver, can_id, master_id);
        case 1:
            // J2 : 10010L
            m_motors[joint_index] = std::make_unique<J10010L_MIT>(m_driver, can_id, master_id);
        case 2:
            // J3 : 8009
            m_motors[joint_index] = std::make_unique<J8009_MIT>(m_driver, can_id, master_id);
        case 4:
            // J5 : 4310
            m_motors[joint_index] = std::make_unique<J4310_MIT>(m_driver, can_id, master_id);
        default: ;
        }
        kp = joint_node["mit_pid"]["kp"].value<float>().value();
        ki = joint_node["mit_pid"]["ki"].value<float>().value();
        kd = joint_node["mit_pid"]["kd"].value<float>().value();
        (void)m_motors[joint_index]->setPidParams(kp, ki, kd);
        m_logger->info(
            "J{} DM motor parsed: can_id: {}, master_id: {}, dir: {}, offset: {}, MIT params: {}, {}, {}",
            joint_index + 1, can_id, master_id,
            m_dirs[joint_index], m_offsets[joint_index], kp, ki, kd);
    }

    void ArmHW::parse_dji_motor(toml::v3::node_view<toml::v3::node> joint_node, size_t joint_index)
    {
        float kp{}, ki{}, kd{};
        m_dirs[joint_index] = joint_node["dir"].value<float>().value();
        m_offsets[joint_index] = joint_node["offset"].value<float>().value();
        kp = joint_node["position_pid"]["kp"].value<float>().value();
        ki = joint_node["position_pid"]["ki"].value<float>().value();
        kd = joint_node["position_pid"]["kd"].value<float>().value();

        const PidParams<> pos_params{
            .Kp = kp,
            .Ki = ki,
            .Kd = kd,
            .MaxOutput = 20000,
            .Deadband = 0,
            .IntegralLimit = 5000,
        };
        const PidConfig<one::pid::Positional, float, PIDFeatures> pos_config = {pos_params};
        kp = joint_node["velocity_pid"]["kp"].value<float>().value();
        ki = joint_node["velocity_pid"]["ki"].value<float>().value();
        kd = joint_node["velocity_pid"]["kd"].value<float>().value();
        const PidParams<> ang_params{
            .Kp = kp,
            .Ki = ki,
            .Kd = kd,
            .MaxOutput = 20000,
            .Deadband = 0,
            .IntegralLimit = 8000,
        };
        const PidConfig<one::pid::Positional, float, PIDFeatures> ang_config = {ang_params};
        PidChain pid_chain(pos_config, ang_config);
        m_motors[joint_index] = std::make_unique<GM6020_Voltage<4, decltype(pid_chain)>>(m_driver, pid_chain);

        m_logger->info(
            "J{} DJI motor parsed: dir: {}, offset: {}",
            joint_index + 1, m_dirs[joint_index], m_offsets[joint_index]);
    }
}

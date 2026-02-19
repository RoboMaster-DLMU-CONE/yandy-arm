#include <yandy/modules/ArmHW.hpp>
#include <yandy/core/Logger.hpp>

#include <one/motor/dm/DmMotor.hpp>
#include <one/motor/dji/DjiMotor.hpp>
#include <one/can/CanDriver.hpp>
#include <toml++/toml.hpp>

// Simulation Includes
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/algorithm/geometry.hpp>

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
    namespace
    {
        // ---------------------------------------------------------------------
        // RealArmHW Implementation
        // ---------------------------------------------------------------------
        class RealArmHW : public IArmHW
        {
        public:
            explicit RealArmHW(one::can::CanDriver& driver) : m_driver(driver)
            {
                m_logger = core::create_logger("RealArmHW", spdlog::level::info);
                parse_config();
            }

            void read(common::JointState& state) override
            {
                for (int i = 0; i < common::JOINT_NUM; ++i)
                {
                    double raw_pos{}; // rad
                    double raw_vel{}; // rad/s
                    double raw_tau{}; // Nm
                    // Check if motor is initialized
                    if (!m_motors[i]) continue;

                    auto status_opt = m_motors[i]->getStatusVariant();
                    if (!status_opt.has_value()) continue;
                    auto status = status_opt.value();
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

            void write(const common::JointCommand& cmd) override
            {
                for (int i = 0; i < common::JOINT_NUM; ++i)
                {
                    if (!m_motors[i]) continue;

                    double target_pos = (m_dirs[i] * cmd.q_des[i]) + m_offsets[i];
                    double target_vel = m_dirs[i] * cmd.v_des[i];
                    double ff_torque = m_dirs[i] * cmd.tau_ff[i];

                    (void)m_motors[i]->setUnitRefs(target_pos * rad, target_vel * rad / s, ff_torque * N * m);
                }
            }

            void enable() override
            {
                m_logger->info("Enabling all Arm motors...");
                for (int i = 0; i < common::JOINT_NUM; ++i)
                {
                    if (!m_motors[i]) continue;
                    if (auto res = m_motors[i]->enable(); !res.has_value())
                    {
                        m_logger->error("Failed to enable motor {}: {}", i + 1, res.error().message);
                    }
                }
            }

            void disable() override
            {
                m_logger->info("Disabling all Arm motors...");
                for (int i = 0; i < common::JOINT_NUM; ++i)
                {
                    if (!m_motors[i]) continue;
                    if (auto res = m_motors[i]->disable(); !res.has_value())
                    {
                        m_logger->error("Failed to disable motor {}: {}", i + 1, res.error().message);
                    }
                }
            }

        private:
            void parse_config()
            {
                m_logger->info("Initializing Real Arm Hardware, loading configs from: {}...", JOINT_CONFIG_PATH);

                auto tbl = toml::parse_file(JOINT_CONFIG_PATH);
                parse_dm_motor(tbl["joint_1"], 0);
                parse_dm_motor(tbl["joint_2"], 1);
                parse_dm_motor(tbl["joint_3"], 2);
                parse_dji_motor(tbl["joint_4"], 3);
                parse_dm_motor(tbl["joint_5"], 4);
            }

            void parse_dm_motor(const toml::v3::node_view<toml::v3::node> joint_node, const size_t joint_index)
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
                    break;
                case 1:
                    // J2 : 10010L
                    m_motors[joint_index] = std::make_unique<J10010L>(m_driver, dm::Param{
                                                                          can_id, master_id, dm::MITMode{kp, kd}
                                                                      });
                    break;
                case 2:
                    // J3 : 8009
                    m_motors[joint_index] = std::make_unique<
                        J8009>(m_driver, dm::Param{can_id, master_id, dm::MITMode{kp, kd}});
                    break;
                case 4:
                    // J5 : 4310
                    m_motors[joint_index] = std::make_unique<
                        J4310>(m_driver, dm::Param{can_id, master_id, dm::MITMode{kp, kd}});
                    break;
                default: ;
                }
                m_logger->info(
                    "J{} DM motor parsed: can_id: {}, master_id: {}, dir: {}, offset: {}, MIT params: {}, {}",
                    joint_index + 1, can_id, master_id,
                    m_dirs[joint_index], m_offsets[joint_index], kp, kd);
            }

            void parse_dji_motor(toml::v3::node_view<toml::v3::node> joint_node, size_t joint_index)
            {
                float kp{}, kd{};
                const uint8_t id = joint_node["id"].value<uint8_t>().value();
                m_dirs[joint_index] = joint_node["dir"].value<float>().value();
                m_offsets[joint_index] = joint_node["offset"].value<float>().value();
                kp = joint_node["mit_pid"]["kp"].value<float>().value();
                kd = joint_node["mit_pid"]["kd"].value<float>().value();

                m_motors[joint_index] = std::make_unique<
                    GM6020_Voltage>(m_driver, dji::Param{id, dji::MITMode{kp, kd}});

                m_logger->info(
                    "J{} DJI motor parsed: dir: {}, offset: {}",
                    joint_index + 1, m_dirs[joint_index], m_offsets[joint_index]);
            }

            one::can::CanDriver& m_driver;
            std::array<std::unique_ptr<IMotor>, 5> m_motors;
            std::array<float, 5> m_dirs{};
            std::array<float, 5> m_offsets{};
            std::shared_ptr<spdlog::logger> m_logger;
        };

        // ---------------------------------------------------------------------
        // FakeArmHW Implementation
        // ---------------------------------------------------------------------
        class FakeArmHW : public IArmHW
        {
        public:
            FakeArmHW()
            {
                m_logger = core::create_logger("FakeArmHW", spdlog::level::info);
                const std::string urdf_path = YANDY_CONFIG_PATH "urdf/yandy_urdf.urdf";
                m_logger->info("Initializing Fake Arm Hardware, loading URDF from: {}", urdf_path);

                pinocchio::urdf::buildModel(urdf_path, m_sim_model);
                m_sim_data = pinocchio::Data(m_sim_model);

                m_state.q = common::VectorJ::Zero();
                m_state.v = common::VectorJ::Zero();
                m_state.tau = common::VectorJ::Zero();

                // 初始姿态：避开奇异点，同时落在关节限位内
                m_state.q << 0.0, 0.6, 1.5, 0.0, 0.8;

                // 初始化 last_cmd，避免第一帧飞车
                m_last_cmd.q_des = m_state.q;
                m_last_cmd.kp.fill(20.0); // 提高刚度
                m_last_cmd.kd.fill(1.0);
            }

            void read(common::JointState& out_state) override
            {
                out_state = m_state;
            }

            void write(const common::JointCommand& cmd) override
            {
                m_last_cmd = cmd;
            }

            void step(double dt) override
            {
                using common::VectorJ;
                // 计算重力 (G)
                VectorJ tau_grav = pinocchio::rnea(m_sim_model, m_sim_data, m_state.q, VectorJ::Zero(),
                                                   VectorJ::Zero());

                // 计算电机力矩 (MIT Mode)
                VectorJ tau_motor = m_last_cmd.kp.cwiseProduct(m_last_cmd.q_des - m_state.q) +
                    m_last_cmd.kd.cwiseProduct(m_last_cmd.v_des - m_state.v) +
                    m_last_cmd.tau_ff;

                // 简化的物理积分 (F = ma)
                // 减小模拟惯量，让它响应更快
                VectorJ inertia;
                inertia.fill(0.2);

                VectorJ damping;
                damping.fill(0.5); // 稍微加点阻尼防止震荡

                const VectorJ acc = (tau_motor - tau_grav - damping.cwiseProduct(m_state.v)).cwiseQuotient(inertia);

                // 半隐式欧拉积分
                m_state.v += acc * dt;
                m_state.q += m_state.v * dt;
                m_state.tau = tau_motor;
            }

            void enable() override
            {
                m_logger->info("Simulation enabled");
            }

            void disable() override
            {
                m_logger->info("Simulation disabled");
            }

        private:
            pinocchio::Model m_sim_model;
            pinocchio::Data m_sim_data;
            common::JointState m_state;
            common::JointCommand m_last_cmd;
            std::shared_ptr<spdlog::logger> m_logger;
        };
    }

    ArmHW::ArmHW()
    {
        const auto logger = core::create_logger("ArmHW", spdlog::level::info);
        logger->info("Reading joint config from: {}", JOINT_CONFIG_PATH);

        auto tbl = toml::parse_file(JOINT_CONFIG_PATH);
        const auto simulate = tbl["simulate"].value<bool>().value();
        const auto can_port = tbl["can_port"].value<std::string>().value();
        if (simulate)
        {
            logger->info("Simulation mode detected. Creating FakeArmHW...");
            m_impl = std::make_unique<FakeArmHW>();
        }
        else
        {
            logger->info("Real hardware mode detected. Creating RealArmHW...");
            m_can_driver = std::make_unique<CanDriver>(can_port);
            m_impl = std::make_unique<RealArmHW>(*m_can_driver.get());
        }
    }

    void ArmHW::read(common::JointState& state)
    {
        m_impl->read(state);
    }

    void ArmHW::write(const common::JointCommand& cmd)
    {
        m_impl->write(cmd);
    }

    void ArmHW::enable()
    {
        m_impl->enable();
    }

    void ArmHW::disable()
    {
        m_impl->disable();
    }

    void ArmHW::step(double dt)
    {
        m_impl->step(dt);
    }
}


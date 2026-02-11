#ifndef YANDY_ARM_ARM_HPP
#define YANDY_ARM_ARM_HPP

#include <array>
#include <memory>
#include <one/can/CanDriver.hpp>

#include <one/motor/IMotor.hpp>
#include <spdlog/logger.h>
#include <yandy/common/Types.hpp>

#include <toml++/toml.hpp>

#include "IArmHW.hpp"

namespace yandy::modules
{
    class ArmHW : public IArmHW
    {
    public:
        explicit ArmHW(one::can::CanDriver& driver);
        void read(common::JointState& state) override;
        void write(const common::JointCommand& cmd) override;
        void enable() override;
        void disable() override;

    private:
        void parse_config();
        void parse_dm_motor(toml::v3::node_view<toml::v3::node> joint_node, size_t joint_index);
        void parse_dji_motor(toml::v3::node_view<toml::v3::node> joint_node, size_t joint_index);
        std::array<std::unique_ptr<one::motor::IMotor>, common::JOINT_NUM> m_motors;
        std::array<float, common::JOINT_NUM> m_dirs{};
        std::array<float, common::JOINT_NUM> m_offsets{};
        std::shared_ptr<spdlog::logger> m_logger;
        one::can::CanDriver& m_driver;
    };
}

#endif //YANDY_ARM_ARM_HPP

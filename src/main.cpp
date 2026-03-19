#include <yandy/Robot.hpp>
#include <one/can/CanDriver.hpp>
#include <toml++/toml.hpp>

#define YANDY_JOINT_CONFIG YANDY_CONFIG_PATH "joint.toml"

int main()
{
    yandy::core::init_logging();

    // 从配置文件读取 CAN 端口
    auto tbl = toml::parse_file(YANDY_JOINT_CONFIG);
    const auto can_port = tbl["can_port"].value<std::string>().value();
    one::can::CanDriver can(can_port);

    yandy::Robot robot(can);
    robot.start();

    return 0;
}

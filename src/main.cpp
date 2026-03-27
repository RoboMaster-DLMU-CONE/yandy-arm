#include <yandy/Robot.hpp>
#include <one/can/CanDriver.hpp>
#include <toml++/toml.hpp>

#include <atomic>
#include <csignal>
#include <thread>

#define YANDY_JOINT_CONFIG YANDY_CONFIG_PATH "joint.toml"

static std::atomic<bool> g_running{true};
static yandy::Robot* g_robot_ptr{nullptr};

static void signal_handler(int) {
  g_running.store(false, std::memory_order_release);
  if (g_robot_ptr) {
    g_robot_ptr->stop();
  }
}

int main()
{
    yandy::core::init_logging();
    std::signal(SIGINT, signal_handler);

    // 从配置文件读取 CAN 端口
    auto tbl = toml::parse_file(YANDY_JOINT_CONFIG);
    const auto can_port = tbl["can_port"].value<std::string>().value();
    one::can::CanDriver can(can_port);

    yandy::Robot robot(can);
    g_robot_ptr = &robot;

    std::thread robot_thread([&robot] { robot.start(); });

    // 主循环：等待退出信号
    while (g_running.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    robot.stop();
    robot_thread.join();
    g_robot_ptr = nullptr;

    return 0;
}

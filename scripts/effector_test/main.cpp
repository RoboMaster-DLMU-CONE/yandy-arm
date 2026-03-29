#include <atomic>
#include <chrono>
#include <csignal>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <one/can/CanDriver.hpp>
#include <spdlog/spdlog.h>
#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <toml++/toml.hpp>
#include <unistd.h>
#include <yandy/core/Logger.hpp>
#include <yandy/modules/Effector.hpp>

using namespace yandy;
using namespace std::chrono_literals;

std::atomic<bool> g_running{true};

void signal_handler(int signum) { g_running = false; }

// 非阻塞读取用户输入
bool readInput(std::string &input) {
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);

  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 100000; // 100ms 超时

  int retval = select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv);
  if (retval > 0 && FD_ISSET(STDIN_FILENO, &fds)) {
    // 有输入可用，读取一行
    if (std::getline(std::cin, input)) {
      return true;
    }
  }
  return false;
}

const char *stateToString(modules::Effector::State state) {
  switch (state) {
  case modules::Effector::State::Idle:
    return "Idle";
  case modules::Effector::State::Opening:
    return "Opening";
  case modules::Effector::State::Closing:
    return "Closing";
  case modules::Effector::State::Holding:
    return "Holding";
  default:
    return "Unknown";
  }
}

int main() {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  core::init_logging();

  auto logger = core::create_logger("EffectorTest", spdlog::level::info);
  logger->info("Starting Effector Test...");
  logger->info("============================================");
  logger->info("Commands:");
  logger->info("  o - Open claw (张开夹爪)");
  logger->info("  c - Close claw (闭合夹爪)");
  logger->info("  r - Release (释放)");
  logger->info("  s - Print status (打印状态)");
  logger->info("  q - Quit (退出)");
  logger->info("============================================");

  auto joint_tbl = toml::parse_file(YANDY_CONFIG_PATH "joint.toml");
  const auto can_port = joint_tbl["can_port"].value<std::string>().value();
  logger->info("Using CAN port: {}", can_port);

  one::can::CanDriver can(can_port);

  logger->info("Initializing Effector (will perform zeroing)...");
  auto effector = std::make_unique<modules::Effector>(can);
  logger->info("Effector initialized.");

  // 状态更新线程
  auto update_thread = std::jthread([&](const std::stop_token &stop_token) {
    auto next_time = std::chrono::steady_clock::now();
    const auto period = 4ms; // 250Hz

    while (!stop_token.stop_requested() && g_running) {
      effector->update();

      next_time += period;
      std::this_thread::sleep_until(next_time);
    }
  });

  // 状态显示线程
  auto display_thread = std::jthread([&](const std::stop_token &stop_token) {
    while (!stop_token.stop_requested() && g_running) {
      auto state = effector->getState();
      logger->info("State: {} | isOpen: {} | isClosed: {} | isHolding: {}",
                   stateToString(state), effector->isOpen(),
                   effector->isClosed(), effector->isHolding());
      std::this_thread::sleep_for(1s);
    }
  });

  // 主循环处理用户输入
  std::string input;
  logger->info("Enter command:");
  while (g_running) {

    if (readInput(input)) {
      // 有输入
      if (input == "o" || input == "open") {
        effector->openClaw();
      } else if (input == "c" || input == "close") {
        effector->closeClaw();
      } else if (input == "r" || input == "release") {
        effector->release();
      } else if (input == "s" || input == "status") {
        auto state = effector->getState();
        std::cout << "State: " << stateToString(state)
                  << " | isOpen: " << effector->isOpen()
                  << " | isClosed: " << effector->isClosed()
                  << " | isHolding: " << effector->isHolding() << std::endl;
      } else if (input == "q" || input == "quit" || input == "exit") {
        g_running = false;
        break;
      } else if (!input.empty()) {
        std::cout << "Unknown command: " << input << std::endl;
      }
    }
    // readInput 超时后会返回 false，继续循环检查 g_running
  }

  logger->info("Stopping...");
  update_thread.request_stop();
  display_thread.request_stop();
  update_thread.join();
  display_thread.join();
  logger->info("Done.");

  return 0;
}

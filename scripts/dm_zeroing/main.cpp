#include <iostream>
#include <one/can/CanDriver.hpp>
#include <one/motor/dm/DmMotor.hpp>
#include <toml++/toml.hpp>

using namespace one::motor::dm;
#define YANDY_JOINT_CONFIG YANDY_CONFIG_PATH "joint.toml"

int main() {
  auto can = one::can::CanDriver("can0");
  (void)can.open().or_else([](const auto &e) {
    std::cerr << e.message << std::endl;
    std::abort();
  });
  auto tbl = toml::parse_file(YANDY_JOINT_CONFIG);

  J4340 j1(can, {tbl["joint_1"]["can_id"].value<uint16_t>().value(),
                 tbl["joint_1"]["master_id"].value<uint16_t>().value(),
                 MITMode{0, 0}});
  J10010L j2(can, {tbl["joint_2"]["can_id"].value<uint16_t>().value(),
                   tbl["joint_2"]["master_id"].value<uint16_t>().value(),
                   MITMode{0, 0}});
  J8009 j3(can, {tbl["joint_3"]["can_id"].value<uint16_t>().value(),
                 tbl["joint_3"]["master_id"].value<uint16_t>().value(),
                 MITMode{0, 0}});
  J4310 j5(can, {tbl["joint_5"]["can_id"].value<uint16_t>().value(),
                 tbl["joint_5"]["master_id"].value<uint16_t>().value(),
                 MITMode{0, 0}});

  auto t = std::jthread([&](const std::stop_token &stop_token) {
    while (!stop_token.stop_requested()) {
      (void)j1.sendRefreshStatus();
      (void)j2.sendRefreshStatus();
      (void)j3.sendRefreshStatus();
      (void)j5.sendRefreshStatus();
      std::cout << std::format("j1: {}, j2: {}, j3: {}, j5: {}\n",
                               j1.getStatus().position, j2.getStatus().position,
                               j3.getStatus().position,
                               j5.getStatus().position);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  });

  std::string input;

  while (input != "exit") {
    std::cin >> input;
    tl::expected<void, one::motor::Error> res;
    if (input == "1") {
      res = j1.setZeroPosition();
    } else if (input == "2") {
      res = j2.setZeroPosition();
    } else if (input == "3") {
      res = j3.setZeroPosition();
    } else if (input == "5") {
      res = j5.setZeroPosition();
    } else {
      std::cout << "Invalid input; \n";
      continue;
    }
    if (!res) {
      std::cerr << res.error().message << std::endl;
    } else {
      std::cout << std::format("Successfully zeroing j{}\n", input);
    }
  }

  t.request_stop();
  t.join();

  return 0;
}

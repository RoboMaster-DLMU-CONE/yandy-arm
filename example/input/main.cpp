#include <yandy/core/Logger.hpp>
#include <yandy/modules/InputProvider.hpp>
std::atomic<bool> running = true;

void printPack(const YandyControlPack &pack) {}

using namespace std::chrono_literals;

int main() {
  yandy::core::init_logging();
  auto logger = yandy::core::get_default_logger();
  yandy::modules::InputProvider provider;
  const auto printPack = [logger](const YandyControlPack &pack) {
    logger->info(
        "Received: x={}, y={}, z={}, ee_qw={}, ee_qx={}, ee_qy={}, ee_qz={}, cmd={}", pack.x,
        pack.y, pack.z, pack.ee_qw, pack.ee_qx, pack.ee_qy, pack.ee_qz, pack.cmd);
    logger->info("  gimbal: z={}, yaw={}, pitch={}", pack.gimbal_z,
                 pack.gimbal_yaw, pack.gimbal_pitch);
    logger->info("  IMU acc: ax={}, ay={}, az={}", pack.ax, pack.ay, pack.az);
    logger->info("  IMU gyro: gx={}, gy={}, gz={}", pack.gx, pack.gy, pack.gz);
    logger->info("  Chassis quat: qw={}, qx={}, qy={}, qz={}", pack.qw, pack.qx,
                 pack.qy, pack.qz);
  };
  provider.setCommandCb([logger](YandyControlCmd cmd) {
    logger->info("State change to : {}", cmd);
  });
  while (running.load(std::memory_order_acquire)) {
    printPack(provider.getLatestCommand());
    std::this_thread::sleep_for(100ms);
  }

  return 0;
}

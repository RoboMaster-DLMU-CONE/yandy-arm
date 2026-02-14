#include <yandy/core/Logger.hpp>
#include <yandy/modules/InputProvider.hpp>
std::atomic<bool> running = true;

void printPack(const YandyControlPack& pack)
{
}

using namespace std::chrono_literals;

int main()
{
    yandy::core::init_logging();
    auto logger = yandy::core::get_default_logger();
    yandy::modules::InputProvider provider;
    const auto printPack = [logger](const YandyControlPack& pack)
    {
        const auto [x,y,z,roll,pitch,yaw, s] = pack;
        logger->info("Received: {}, {}, {}, {}, {}, {}, {}", x, y, z, roll, pitch, yaw, s);
    };
    while (running.load(std::memory_order_acquire))
    {
        printPack(provider.getLatestCommand());
        std::this_thread::sleep_for(100ms);
    }

    return 0;
}

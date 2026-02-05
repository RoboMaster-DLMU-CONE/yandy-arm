#include <yandy/core/Logger.hpp>

int main()
{
    yandy::core::init_logging();

    spdlog::info("Yandy Logger initialized.");

    return 0;
}

#ifndef YANDY_ARM_LOGGER_HPP
#define YANDY_ARM_LOGGER_HPP

#include <spdlog/spdlog.h>

namespace yandy::core
{
    using sink_ptr = spdlog::sink_ptr;
    void init_logging();
    bool is_logging_initialized() noexcept;
    std::shared_ptr<spdlog::logger> create_logger(const std::string& name,
                                                  spdlog::level::level_enum level = spdlog::level::info);
    std::shared_ptr<spdlog::logger> get_default_logger();
}

#endif //YANDY_ARM_LOGGER_HPP

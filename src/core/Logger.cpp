#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/async.h>
#include <yandy/core/Logger.hpp>
#include <yandy/utils.hpp>

#define LOG_CONFIG_PATH YANDY_CONFIG_PATH "log.toml"

#include <toml++/toml.hpp>

namespace yandy::core
{
    namespace
    {
        struct Storage
        {
            std::vector<sink_ptr> sinks;
            std::shared_ptr<spdlog::logger> default_logger;
            bool async = false;
        };

        Storage& storage()
        {
            static Storage s;
            return s;
        }

        std::once_flag init_flag;
    }

    void init_logging()
    {
        std::call_once(init_flag, [&]()
        {
            // parse log.toml config
            auto tbl = toml::parse_file(LOG_CONFIG_PATH);
            const auto is_async = tbl["async"].value<bool>().value();
            const auto default_name = tbl["default_name"].value<std::string>().value();
            const auto is_console_enabled = tbl["console_sink"]["enable"].value<bool>().value();
            const auto is_file_enabled = tbl["file_sink"]["enable"].value<bool>().value();
            const auto file_sink_path = tbl["file_sink"]["path"].value<std::string>().value();
            auto& [sinks, default_logger, async] = storage();
            async = is_async;
            if (is_console_enabled)
            {
                auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
                console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%l%$] %v");
                sinks.push_back(console_sink);
            }
            if (is_file_enabled)
            {
                auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
                    file_sink_path, 1024 * 1024 * 10, 3);
                file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] %v");
                sinks.push_back(file_sink);
            }

            // async thread-pool
            if (async)
            {
                spdlog::init_thread_pool(8192, 1);
                auto logger = std::make_shared<spdlog::async_logger>(
                    default_name, sinks.begin(), sinks.end(),
                    spdlog::thread_pool(), spdlog::async_overflow_policy::block);
                logger->set_level(spdlog::level::info);
                spdlog::register_logger(logger);
                spdlog::set_default_logger(logger);
                default_logger = logger;
            }
            else
            {
                auto logger = std::make_shared<spdlog::logger>(default_name, sinks.begin(), sinks.end());
                logger->set_level(spdlog::level::info);
                spdlog::register_logger(logger);
                spdlog::set_default_logger(logger);
                default_logger = logger;
            }
        });
    }

    bool is_logging_initialized() noexcept
    {
        // 检查是否为 nullptr
        return static_cast<bool>(storage().default_logger);
    }

    std::shared_ptr<spdlog::logger> create_logger(const std::string& name,
                                                  spdlog::level::level_enum level)
    {
        auto& s = storage();
        if (s.sinks.empty()) return nullptr; // not initialized or no sinks

        if (s.async)
        {
            auto logger = std::make_shared<spdlog::async_logger>(
                name, s.sinks.begin(), s.sinks.end(), spdlog::thread_pool(), spdlog::async_overflow_policy::block);
            logger->set_level(level);
            spdlog::register_logger(logger);
            return logger;
        }
        auto logger = std::make_shared<spdlog::logger>(name, s.sinks.begin(), s.sinks.end());
        logger->set_level(level);
        spdlog::register_logger(logger);
        return logger;
    }

    std::shared_ptr<spdlog::logger> get_default_logger()
    {
        return storage().default_logger;
    }
}

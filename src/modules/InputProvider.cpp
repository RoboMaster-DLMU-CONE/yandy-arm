#include <yandy/modules/InputProvider.hpp>
#include <yandy/core/Logger.hpp>

#include <toml++/toml.hpp>

#define YANDY_INPUT_CONFIG YANDY_CONFIG_PATH "input.toml"

namespace yandy::modules
{
    namespace detail
    {
        IInputProvider::IInputProvider() : m_par(m_des)
        {
        }

        UdpProvider::UdpProvider()
        {
        }

        bool UdpProvider::getLatestCommand(YandyControlPack& packet)
        {
            return true;
        }

        UsbProvider::UsbProvider()
        {
        }

        bool UsbProvider::getLatestCommand(YandyControlPack& packet)
        {
            return true;
        }
    }


    InputProvider::InputProvider()
    {
        const auto temp_logger = core::get_default_logger();
        temp_logger->info("Constructing InputProvider, loading config from {}", YANDY_INPUT_CONFIG);
        auto tbl = toml::parse_file(YANDY_INPUT_CONFIG);
        if (auto type = tbl["type"].value<std::string_view>().value(); type == "usb")
        {
            temp_logger->info("Construction UsbInputProvider...");
            m_provider = std::make_unique<detail::UsbProvider>();
        }
        else if (type == "udp")
        {
            temp_logger->info("Construction UdpInputProvider...");
            m_provider = std::make_unique<detail::UdpProvider>();
        }
        else
        {
            temp_logger->error("Invalid type: {}, please use 'udp' or 'usb'", type);
            throw std::runtime_error("invalid type");
        }
    }

    bool InputProvider::getLatestCommand(YandyControlPack& packet) const
    {
        return m_provider->getLatestCommand(packet);
    }
}

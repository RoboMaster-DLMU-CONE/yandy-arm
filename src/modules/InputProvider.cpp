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

        UdpProvider::UdpProvider() : m_socket(m_io_context), m_running(true)
        {
            m_logger = core::create_logger("UdpInputProvider", spdlog::level::info);
            m_logger->info("Constructing UdpProvider, loading config from {}", YANDY_INPUT_CONFIG);

            auto tbl = toml::parse_file(YANDY_INPUT_CONFIG);
            auto address = tbl["udp"]["address"].value<std::string>().value();
            auto port = tbl["udp"]["port"].value<uint16_t>().value();

            udp::endpoint endpoint(boost::asio::ip::make_address(address), port);
            m_socket.open(endpoint.protocol());
            m_socket.bind(endpoint);

            m_logger->info("UdpProvider bound to {}:{}", address, port);

            startReceiveThread();
        }

        void UdpProvider::startReceiveThread()
        {
            m_receive_thread = std::thread([this]()
            {
                doReceive();
                m_io_context.run();
            });
        }

        void UdpProvider::doReceive()
        {
            m_socket.async_receive_from(
                boost::asio::buffer(m_recv_buffer),
                m_remote_endpoint,
                [this](const boost::system::error_code& error, std::size_t bytes_transferred)
                {
                    receiveHandler(error, bytes_transferred);
                });
        }

        void UdpProvider::receiveHandler(const boost::system::error_code& error, std::size_t bytes_transferred)
        {
            if (!error && m_running)
            {
                const auto data_ptr = reinterpret_cast<const uint8_t*>(m_recv_buffer.data());

                auto result = m_par.push_data(data_ptr, bytes_transferred);
                if (result)
                {
                    m_packet_buffer.write(m_des.get<YandyControlPack>());
                }

                // 重新开始接收下一个数据包
                doReceive();
            }
            else if (error != boost::asio::error::operation_aborted && m_running)
            {
                m_logger->error("UDP receive error: {}", error.message());

                // 出错后继续尝试接收
                doReceive();
            }
        }

        bool UdpProvider::getLatestCommand(YandyControlPack& packet)
        {
            packet = m_packet_buffer.read();
            return true;
        }

        UsbProvider::UsbProvider()
        {
            m_logger = core::create_logger("UdpInputProvider", spdlog::level::info);
            m_logger->info("Constructing UdpProvider, loading config from {}", YANDY_INPUT_CONFIG);

            auto tbl = toml::parse_file(YANDY_INPUT_CONFIG);
            auto device = tbl["usb"]["device"].value<std::string>().value();
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

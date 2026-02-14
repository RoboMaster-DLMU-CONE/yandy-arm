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

        YandyControlPack UdpProvider::getLatestCommand()
        {
            return m_packet_buffer.read();
        }

        UsbProvider::UsbProvider()
        {
            m_logger = core::create_logger("UdpInputProvider", spdlog::level::info);
            m_logger->info("Constructing UdpProvider, loading config from {}", YANDY_INPUT_CONFIG);

            auto tbl = toml::parse_file(YANDY_INPUT_CONFIG);
            auto device = tbl["usb"]["device"].value<std::string>().value();
            auto baud_rate = tbl["usb"]["baud_rate"].value<uint32_t>().value();

            HySerial::Builder builder;
            builder.device(device)
                   .baud_rate(baud_rate)
                   .data_bits(HySerial::DataBits::BITS_8)
                   .parity(HySerial::Parity::NONE)
                   .stop_bits(HySerial::StopBits::ONE);

            builder.on_read([this](const std::span<const std::byte> data) { on_serial_read(data); });
            builder.on_error([this](const ssize_t e) { on_serial_error(e); });

            auto serial_or_err = builder.build();
            if (!serial_or_err)
            {
                m_logger->error("Failed to create Serial: {}", serial_or_err.error().message);
                throw std::runtime_error("Failed to create Usb Serial port");
            }
            m_serial = std::move(serial_or_err.value());
            m_serial->start_read();
        }

        UsbProvider::~UsbProvider()
        {
            if (m_serial)
            {
                m_serial->stop_read();
            }
        }

        YandyControlPack UsbProvider::getLatestCommand()
        {
            return m_buf.read();
        }

        void UsbProvider::on_serial_read(std::span<const std::byte> data)
        {
            const auto size = data.size();
            const auto u8_data = reinterpret_cast<const uint8_t*>(data.data());
            (void)m_par.push_data(u8_data, size);
            m_buf.write(m_des.get<YandyControlPack>());
        }

        void UsbProvider::on_serial_error(const ssize_t e) const
        {
            m_logger->error("{}", strerror(static_cast<int>(e)));
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

    YandyControlPack InputProvider::getLatestCommand() const
    {
        return m_provider->getLatestCommand();
    }
}

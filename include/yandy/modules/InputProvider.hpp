#ifndef YANDY_ARM_INPUTINTERFACE_HPP
#define YANDY_ARM_INPUTINTERFACE_HPP

#include <yandy/common/Types.hpp>
#include <yandy/common/NBuf.hpp>
#include <yandy/core/Logger.hpp>
#include <RPL/Deserializer.hpp>
#include <RPL/Parser.hpp>

#include <boost/asio.hpp>
#include <thread>
#include <mutex>
#include <optional>
#include <array>

namespace yandy::modules
{
    namespace detail
    {
        using udp = boost::asio::ip::udp;

        class IInputProvider
        {
        public:
            IInputProvider();

            virtual ~IInputProvider()
            {
            };

            virtual bool getLatestCommand(YandyControlPack& packet) = 0;

        protected:
            RPL::Deserializer<YandyControlPack> m_des;
            RPL::Parser<YandyControlPack> m_par;
            std::shared_ptr<spdlog::logger> m_logger;
        };

        class UdpProvider : public IInputProvider
        {
        public:
            UdpProvider();
            bool getLatestCommand(YandyControlPack& packet) override;

        private:
            void startReceiveThread();
            void receiveHandler(const boost::system::error_code& error, std::size_t bytes_transferred);
            void doReceive();

        private:
            boost::asio::io_context m_io_context;
            udp::socket m_socket;
            udp::endpoint m_remote_endpoint;
            std::array<char, 1024> m_recv_buffer;
            std::thread m_receive_thread;
            NBuf<YandyControlPack, 10> m_packet_buffer; // 使用N重缓冲区，大小为10
            bool m_running;
        };

        class UsbProvider : public IInputProvider
        {
        public:
            UsbProvider();
            bool getLatestCommand(YandyControlPack& packet) override;
        };
    }

    class InputProvider
    {
    public:
        InputProvider();
        bool getLatestCommand(YandyControlPack& packet) const;

    private:
        std::unique_ptr<detail::IInputProvider> m_provider;
    };
}

#endif //YANDY_ARM_INPUTINTERFACE_HPP

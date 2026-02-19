#ifndef YANDY_ARM_INPUTINTERFACE_HPP
#define YANDY_ARM_INPUTINTERFACE_HPP

#include <yandy/common/Types.hpp>
#include <yandy/common/NBuf.hpp>
#include <yandy/core/Logger.hpp>
#include <RPL/Deserializer.hpp>
#include <RPL/Parser.hpp>
#include <HySerial/HySerial.hpp>

#include <boost/asio.hpp>
#include <thread>
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

            virtual ~IInputProvider() = default;
            void setCommandCb(const std::function<void(YandyControlCmd)>& func);

            YandyControlPack getLatestCommand();

        protected:
            void update_cmd(const YandyControlCmd cmd);

            int m_filter_threshold{};
            int m_stability_count{};
            YandyControlCmd m_candidate_cmd = YandyControlCmd::CMD_NONE; // 正在观测的命令
            YandyControlCmd m_current_stable_cmd = YandyControlCmd::CMD_NONE; // 当前生效的命令

            RPL::Deserializer<YandyControlPack> m_des;
            RPL::Parser<YandyControlPack> m_par;
            std::shared_ptr<spdlog::logger> m_logger;
            NBuf<YandyControlPack, 10> m_buf; // 使用N重缓冲区，大小为10
            std::function<void(YandyControlCmd)> m_func = [](YandyControlCmd)
            {
            };
        };

        class UdpProvider : public IInputProvider
        {
        public:
            UdpProvider();

        private:
            void startReceiveThread();
            void receiveHandler(const boost::system::error_code& error, std::size_t bytes_transferred);
            void doReceive();

            boost::asio::io_context m_io_context;
            udp::socket m_socket;
            udp::endpoint m_remote_endpoint;
            std::array<char, 1024> m_recv_buffer{};
            std::thread m_receive_thread;

            bool m_running;
        };

        class UsbProvider : public IInputProvider
        {
        public:
            UsbProvider();
            ~UsbProvider() override;

        private:
            void on_serial_read(std::span<const std::byte> data);
            void on_serial_error(ssize_t e) const;
            std::unique_ptr<HySerial::Serial> m_serial;
            NBuf<YandyControlPack, 10> m_buf;
        };
    }

    class InputProvider
    {
    public:
        InputProvider();
        void setCommandCb(const std::function<void(YandyControlCmd)>& func) const;
        [[nodiscard]] YandyControlPack getLatestCommand() const;

    private:
        std::unique_ptr<detail::IInputProvider> m_provider;
    };
}

#endif //YANDY_ARM_INPUTINTERFACE_HPP

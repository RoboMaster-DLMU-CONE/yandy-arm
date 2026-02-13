#ifndef YANDY_ARM_INPUTINTERFACE_HPP
#define YANDY_ARM_INPUTINTERFACE_HPP

#include <yandy/common/Types.hpp>
#include <yandy/core/Logger.hpp>
#include <RPL/Deserializer.hpp>
#include <RPL/Parser.hpp>

namespace yandy::modules
{
    namespace detail
    {
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

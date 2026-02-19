#ifndef YANDY_ARM_IARMHW_HPP
#define YANDY_ARM_IARMHW_HPP

#include <yandy/common/Types.hpp>

namespace yandy::modules
{
    class IArmHW
    {
    public:
        virtual ~IArmHW() = default;

        // 读取当前状态
        virtual void read(common::JointState& state) = 0;

        // 写入控制指令
        virtual void write(const common::JointCommand& cmd) = 0;

        // 硬件使能/失能
        virtual void enable() = 0;
        virtual void disable() = 0;

        // Simulation step, default empty
        virtual void step(double /*dt*/)
        {
        }
    };
}

#endif //YANDY_ARM_IARMHW_HPP

#ifndef YANDY_ARM_ROBOT_HPP
#define YANDY_ARM_ROBOT_HPP


#include <yandy/modules/ArmHW.hpp>
#include <yandy/modules/DynamicsSolver.hpp>
#include <yandy/modules/Effector.hpp>
#include <yandy/modules/FSM.hpp>
#include <yandy/common/NBuf.hpp>

namespace yandy
{
    namespace detail
    {
        struct
        {
        };
    }

    class Robot
    {
    public:
        Robot();

    private:
        modules::ArmHW m_arm_hw;
        modules::DynamicsSolver m_solver;
        modules::YandyArmFSM m_fsm;
        modules::Effector m_effector;
    };
}

#endif //YANDY_ARM_ROBOT_HPP

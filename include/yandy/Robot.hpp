#ifndef YANDY_ARM_ROBOT_HPP
#define YANDY_ARM_ROBOT_HPP


#include <yandy/modules/ArmHW.hpp>
#include <yandy/modules/DynamicsSolver.hpp>
#include <yandy/modules/Effector.hpp>
#include <yandy/modules/FSM.hpp>
#include <yandy/modules/InputProvider.hpp>
#include <yandy/modules/VisionSystem.hpp>
#include <yandy/common/NBuf.hpp>

namespace yandy
{
    namespace detail
    {
        struct ControlTarget
        {
            Eigen::Vector3d target_pos_w;
            Eigen::Matrix3d target_rot_w;
        };

        struct VisionData
        {
            Eigen::Isometry3d unit_pose; // 只有一个，取置信度最高
        };
    }

    class Robot
    {
    public:
        Robot();
        ~Robot();
        void start();
        void stop();

    private:
        modules::ArmHW m_arm_hw;
        modules::DynamicsSolver m_solver;
        modules::YandyArmFSM m_fsm;
        modules::InputProvider m_input;
        modules::HikDriver m_hik_driver;
        modules::EnergyDetector m_detector;
        modules::EnergyPoseSolver m_pose_solver;
        modules::Effector m_effector;
        std::shared_ptr<spdlog::logger> m_logger;
        std::atomic<bool> m_running{true};
    };
}

#endif //YANDY_ARM_ROBOT_HPP

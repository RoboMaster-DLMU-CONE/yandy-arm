#ifndef YANDY_ARM_ROBOT_HPP
#define YANDY_ARM_ROBOT_HPP

#include <thread>

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
            bool valid{false};
            Eigen::Isometry3d unit_pose{Eigen::Isometry3d::Identity()}; // 相机坐标系下，取置信度最高
        };

        struct RobotVizData
        {
            common::VectorJ q{common::VectorJ::Zero()};
            Eigen::Isometry3d ee_pose{Eigen::Isometry3d::Identity()};
            Eigen::Isometry3d target_pose{Eigen::Isometry3d::Identity()};
            YandyState state{YandyState::Disabled};
            bool vision_valid{false};
            Eigen::Isometry3d vision_unit_pose{Eigen::Isometry3d::Identity()};
            Eigen::Isometry3d vision_unit_pose_base{Eigen::Isometry3d::Identity()};
        };
    }

    class Robot
    {
    public:
        Robot();
        ~Robot();
        void start();
        void stop();

        const NBuf<detail::RobotVizData, 3>& vizBuf() const { return m_viz_buf; }

    private:
        // ---- 控制循环常量 ----
        static constexpr double DT = 0.004; // 250Hz

        // 存储模式预设位姿 (基座坐标系)
        static const Eigen::Isometry3d STORE_POSE;

        // ---- Modules ----
        modules::ArmHW m_arm_hw;
        modules::DynamicsSolver m_solver;
        modules::YandyArmFSM m_fsm;
        modules::InputProvider m_input;
        modules::HikDriver m_hik_driver;
        modules::EnergyDetector m_detector;
        modules::EnergyPoseSolver m_pose_solver;
        modules::Effector m_effector;

        // ---- 线程间通信 ----
        NBuf<detail::VisionData, 3> m_vision_buf;
        NBuf<detail::RobotVizData, 3> m_viz_buf;
        std::thread m_vision_thread;

        // ---- 运行时状态 ----
        std::shared_ptr<spdlog::logger> m_logger;
        std::atomic<bool> m_running{true};
        YandyState m_prev_state{YandyState::Disabled};
        common::JointState m_state{};
        common::JointCommand m_cmd{};
        Eigen::Isometry3d m_target_pose{Eigen::Isometry3d::Identity()};
        bool m_is_simulate = false;
        Eigen::Isometry3d m_sim_cam_pose{Eigen::Isometry3d::Identity()}; // 仿真模式下手持相机的固定位姿 (base_link 系)

        // ---- 私有方法 ----
        void visionLoop();
        void onStateTransition(YandyState from, YandyState to);
        void handleManual();
        void handleFetching();
        void handleStore();
    };
}

#endif //YANDY_ARM_ROBOT_HPP

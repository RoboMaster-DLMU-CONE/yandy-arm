#ifndef YANDY_ARM_TRAJECTORY_PLANNER_HPP
#define YANDY_ARM_TRAJECTORY_PLANNER_HPP

#include <array>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

#include <Eigen/Core>
#include <ruckig/ruckig.hpp>
#include <spdlog/spdlog.h>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <yandy/common/Types.hpp>
#include <yandy/common/NBuf.hpp>

namespace yandy::modules
{
    // OMPL 规划结果 — 固定大小以满足 NBuf 的 standard_layout 要求
    struct TrajectoryPlan
    {
        static constexpr int MAX_WAYPOINTS = 64;
        std::array<common::VectorArm, MAX_WAYPOINTS> waypoints{};
        int count{0};
        bool valid{false};
    };

    // OMPL 规划请求
    struct PlanRequest
    {
        common::VectorArm q_start{};
        common::VectorArm q_goal{};
        bool pending{false};
    };

    class TrajectoryPlanner
    {
    public:
        TrajectoryPlanner(double dt,
                          const pinocchio::Model& model,
                          const pinocchio::GeometryModel& geom_model);
        ~TrajectoryPlanner();

        TrajectoryPlanner(const TrajectoryPlanner&) = delete;
        TrajectoryPlanner& operator=(const TrajectoryPlanner&) = delete;

        // ---- 主循环调用 (250Hz) ----

        // 设置直达目标 (无碰撞时) - 只针对机械臂 6 DoF
        void setTarget(const common::VectorArm& q_goal);

        // 设置当前真实状态 (每帧调用，用于 Ruckig 的 current state) - 只针对机械臂
        void syncState(const common::VectorArm& q, const common::VectorArm& v);

        // 更新云台状态 (用于 OMPL 碰撞检测)
        void updateGimbalState(const common::VectorGimbal& gimbal_q);

        // Ruckig 步进，返回本帧的期望位置和速度 (机械臂 6 DoF)
        // 返回 false 表示轨迹已完成 (Finished)
        bool update(common::VectorArm& q_des, common::VectorArm& v_des);

        // 紧急停车: 将目标设为当前位置
        void brake();

        // 轨迹是否已完成
        bool isFinished() const { return m_finished.load(std::memory_order_relaxed); }

        // ---- OMPL 线程相关 ----

        // 请求 OMPL 规划 (唤醒后台线程) - 只针对机械臂
        void requestPlan(const common::VectorArm& q_start, const common::VectorArm& q_goal);

        // 检查 OMPL 是否有新结果
        // 返回值: 0 = 无结果, 1 = 成功加载, -1 = 规划失败
        int consumePlanResult();

        void stopPlanThread();

    private:
        static constexpr int DOF = common::ARM_JOINT_NUM;  // 只规划机械臂 6 DoF

        double m_dt;

        // ---- Ruckig (只针对机械臂 6 DoF) ----
        ruckig::Ruckig<DOF, ruckig::EigenVector> m_ruckig;
        ruckig::InputParameter<DOF, ruckig::EigenVector> m_ruckig_input;
        ruckig::OutputParameter<DOF, ruckig::EigenVector> m_ruckig_output;
        std::atomic<bool> m_finished{true};
        common::VectorArm m_current_target{common::VectorArm::Zero()}; // 当前 Ruckig 目标，用于去重

        // ---- Waypoint 队列 (来自 OMPL) ----
        std::vector<common::VectorArm> m_waypoints;
        int m_waypoint_idx{0};

        // ---- OMPL 后台线程 ----
        std::thread m_plan_thread;
        std::mutex m_plan_mutex;
        std::condition_variable m_plan_cv;
        PlanRequest m_plan_request;
        std::atomic<bool> m_plan_running{true};
        NBuf<TrajectoryPlan, 3> m_plan_result_buf;
        
        // 云台状态 (用于 OMPL 碰撞检测)
        common::VectorGimbal m_current_gimbal_q{common::VectorGimbal::Zero()};
        std::mutex m_gimbal_mutex;

        void planLoop();
        TrajectoryPlan runOMPL(const common::VectorArm& q_start, const common::VectorArm& q_goal);

        void advanceToNextWaypoint();

        // ---- OMPL 碰撞检测 (线程私有数据) ----
        pinocchio::Model m_model;               // 拷贝，OMPL 线程独占
        pinocchio::Data m_data;
        pinocchio::GeometryModel m_geom_model;
        pinocchio::GeometryData m_geom_data;

        std::shared_ptr<spdlog::logger> m_logger;
    };
}

#endif // YANDY_ARM_TRAJECTORY_PLANNER_HPP

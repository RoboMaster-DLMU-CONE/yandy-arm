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
        std::array<common::VectorJ, MAX_WAYPOINTS> waypoints{};
        int count{0};
        bool valid{false};
    };

    // OMPL 规划请求
    struct PlanRequest
    {
        common::VectorJ q_start{};
        common::VectorJ q_goal{};
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

        // 设置直达目标 (无碰撞时)
        void setTarget(const common::VectorJ& q_goal);

        // 设置当前真实状态 (每帧调用，用于 Ruckig 的 current state)
        void syncState(const common::VectorJ& q, const common::VectorJ& v);

        // Ruckig 步进，返回本帧的期望位置和速度
        // 返回 false 表示轨迹已完成 (Finished)
        bool update(common::VectorJ& q_des, common::VectorJ& v_des);

        // 紧急停车: 将目标设为当前位置
        void brake();

        // 轨迹是否已完成
        bool isFinished() const { return m_finished.load(std::memory_order_relaxed); }

        // ---- OMPL 线程相关 ----

        // 请求 OMPL 规划 (唤醒后台线程)
        void requestPlan(const common::VectorJ& q_start, const common::VectorJ& q_goal);

        // 检查 OMPL 是否有新结果，有则加载到 Ruckig waypoint 队列
        bool consumePlanResult();

        void stopPlanThread();

    private:
        static constexpr int DOF = common::JOINT_NUM;

        double m_dt;

        // ---- Ruckig ----
        ruckig::Ruckig<DOF, ruckig::EigenVector> m_ruckig;
        ruckig::InputParameter<DOF, ruckig::EigenVector> m_ruckig_input;
        ruckig::OutputParameter<DOF, ruckig::EigenVector> m_ruckig_output;
        std::atomic<bool> m_finished{true};
        common::VectorJ m_current_target{common::VectorJ::Zero()}; // 当前 Ruckig 目标，用于去重

        // ---- Waypoint 队列 (来自 OMPL) ----
        std::vector<common::VectorJ> m_waypoints;
        int m_waypoint_idx{0};

        // ---- OMPL 后台线程 ----
        std::thread m_plan_thread;
        std::mutex m_plan_mutex;
        std::condition_variable m_plan_cv;
        PlanRequest m_plan_request;
        std::atomic<bool> m_plan_running{true};
        NBuf<TrajectoryPlan, 3> m_plan_result_buf;

        void planLoop();
        TrajectoryPlan runOMPL(const common::VectorJ& q_start, const common::VectorJ& q_goal);

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

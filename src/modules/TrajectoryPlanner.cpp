#include <yandy/modules/TrajectoryPlanner.hpp>
#include <yandy/core/Logger.hpp>

#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/collision/collision.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace yandy::modules
{
    // ================================================================
    // 构造 / 析构
    // ================================================================

    TrajectoryPlanner::TrajectoryPlanner(double dt,
                                         const pinocchio::Model& model,
                                         const pinocchio::GeometryModel& geom_model)
        : m_dt(dt)
        , m_ruckig(dt)
        , m_model(model)
        , m_data(m_model)
        , m_geom_model(geom_model)
        , m_geom_data(m_geom_model)
    {
        m_logger = core::create_logger("TrajPlanner", spdlog::level::info);

        // Ruckig 运动学约束 (只针对机械臂 6 DoF)
        m_ruckig_input.max_velocity.setConstant(6.0);       // rad/s (URDF limit: 8)
        m_ruckig_input.max_acceleration.setConstant(20.0);   // rad/s^2
        m_ruckig_input.max_jerk.setConstant(100.0);          // rad/s^3

        m_ruckig_input.current_position.setZero();
        m_ruckig_input.current_velocity.setZero();
        m_ruckig_input.current_acceleration.setZero();
        m_ruckig_input.target_position.setZero();
        m_ruckig_input.target_velocity.setZero();
        m_ruckig_input.target_acceleration.setZero();

        // 启动 OMPL 后台线程
        m_plan_thread = std::thread([this] { planLoop(); });
        m_logger->info("TrajectoryPlanner initialized, dt={:.4f}s, DOF={}", dt, DOF);
    }

    TrajectoryPlanner::~TrajectoryPlanner()
    {
        stopPlanThread();
    }

    void TrajectoryPlanner::stopPlanThread()
    {
        m_plan_running.store(false, std::memory_order_release);
        m_plan_cv.notify_one();
        if (m_plan_thread.joinable())
            m_plan_thread.join();
    }

    // ================================================================
    // 主循环接口
    // ================================================================

    void TrajectoryPlanner::syncState(const common::VectorArm& q, const common::VectorArm& v)
    {
        m_ruckig_input.current_position = q;
        m_ruckig_input.current_velocity = v;
        // acceleration 由 Ruckig 内部跟踪，不从外部覆盖
    }

    void TrajectoryPlanner::updateGimbalState(const common::VectorGimbal& gimbal_q)
    {
        std::lock_guard lock(m_gimbal_mutex);
        m_current_gimbal_q = gimbal_q;
    }

    void TrajectoryPlanner::setTarget(const common::VectorArm& q_goal)
    {
        // 目标没有显著变化时跳过，避免每帧重置 Ruckig
        constexpr double TARGET_CHANGE_THRESHOLD = 1e-3; // rad
        if ((q_goal - m_current_target).norm() < TARGET_CHANGE_THRESHOLD && !m_finished.load(std::memory_order_relaxed))
        {
            return;
        }

        m_ruckig_input.target_position = q_goal;
        m_ruckig_input.target_velocity.setZero();
        m_ruckig_input.target_acceleration.setZero();
        m_waypoints.clear();
        m_waypoint_idx = 0;
        m_current_target = q_goal;
        m_finished.store(false, std::memory_order_relaxed);
    }

    void TrajectoryPlanner::brake()
    {
        m_ruckig_input.target_position = m_ruckig_input.current_position;
        m_ruckig_input.target_velocity.setZero();
        m_ruckig_input.target_acceleration.setZero();
        m_waypoints.clear();
        m_waypoint_idx = 0;
        m_finished.store(false, std::memory_order_relaxed);
    }

    bool TrajectoryPlanner::update(common::VectorArm& q_des, common::VectorArm& v_des)
    {
        auto result = m_ruckig.update(m_ruckig_input, m_ruckig_output);

        q_des = m_ruckig_output.new_position;
        v_des = m_ruckig_output.new_velocity;

        if (result == ruckig::Result::Finished)
        {
            // 如果还有后续 waypoint，切换到下一个
            if (!m_waypoints.empty() && m_waypoint_idx < static_cast<int>(m_waypoints.size()))
            {
                advanceToNextWaypoint();
            }
            else
            {
                m_finished.store(true, std::memory_order_relaxed);
            }
        }

        m_ruckig_output.pass_to_input(m_ruckig_input);
        return result == ruckig::Result::Working;
    }

    void TrajectoryPlanner::advanceToNextWaypoint()
    {
        m_ruckig_input.target_position = m_waypoints[m_waypoint_idx];
        m_ruckig_input.target_velocity.setZero();
        m_ruckig_input.target_acceleration.setZero();
        ++m_waypoint_idx;
        m_finished.store(false, std::memory_order_relaxed);
    }

    // ================================================================
    // OMPL 接口
    // ================================================================

    void TrajectoryPlanner::requestPlan(const common::VectorArm& q_start, const common::VectorArm& q_goal)
    {
        {
            std::lock_guard lock(m_plan_mutex);
            m_plan_request.q_start = q_start;
            m_plan_request.q_goal = q_goal;
            m_plan_request.pending = true;
        }
        m_plan_cv.notify_one();
        m_logger->info("OMPL plan requested");
    }

    bool TrajectoryPlanner::consumePlanResult()
    {
        auto plan = m_plan_result_buf.try_read();
        if (!plan.has_value() || !plan->valid)
            return false;

        m_waypoints.clear();
        m_waypoints.reserve(plan->count);
        for (int i = 0; i < plan->count; ++i)
        {
            m_waypoints.push_back(plan->waypoints[i]);
        }
        m_waypoint_idx = 0;

        // 立即开始追踪第一个 waypoint
        if (!m_waypoints.empty())
        {
            advanceToNextWaypoint();
        }

        m_logger->info("Loaded OMPL plan with {} waypoints", plan->count);
        return true;
    }

    // ================================================================
    // OMPL 后台线程
    // ================================================================

    void TrajectoryPlanner::planLoop()
    {
        m_logger->info("OMPL plan thread started");

        while (m_plan_running.load(std::memory_order_acquire))
        {
            PlanRequest req;
            {
                std::unique_lock lock(m_plan_mutex);
                m_plan_cv.wait(lock, [this]
                {
                    return m_plan_request.pending || !m_plan_running.load(std::memory_order_relaxed);
                });

                if (!m_plan_running.load(std::memory_order_relaxed))
                    break;

                req = m_plan_request;
                m_plan_request.pending = false;
            }

            auto result = runOMPL(req.q_start, req.q_goal);
            m_plan_result_buf.write(std::move(result));
        }

        m_logger->info("OMPL plan thread exited");
    }

    TrajectoryPlan TrajectoryPlanner::runOMPL(const common::VectorArm& q_start, const common::VectorArm& q_goal)
    {
        TrajectoryPlan plan;
        
        // 获取当前云台状态 (线程安全)
        common::VectorGimbal gimbal_q;
        {
            std::lock_guard lock(m_gimbal_mutex);
            gimbal_q = m_current_gimbal_q;
        }

        // 构建关节空间 (只针对机械臂 6 DoF)
        auto space = std::make_shared<ob::RealVectorStateSpace>(DOF);
        ob::RealVectorBounds bounds(DOF);

        // 从 Pinocchio model 读取机械臂关节限位 (前 6 个关节)
        for (int i = 0; i < DOF; ++i)
        {
            bounds.setLow(i, m_model.lowerPositionLimit[i]);
            bounds.setHigh(i, m_model.upperPositionLimit[i]);
        }
        space->setBounds(bounds);

        og::SimpleSetup ss(space);

        // 碰撞检测回调 — 使用线程私有的 Pinocchio data
        // 将 6D 机械臂配置 + 3D 云台状态组装成完整 9D 进行碰撞检测
        ss.setStateValidityChecker([this, &gimbal_q](const ob::State* state) -> bool
        {
            const auto* s = state->as<ob::RealVectorStateSpace::StateType>();
            common::VectorArm arm_q;
            for (int i = 0; i < DOF; ++i)
                arm_q[i] = s->values[i];

            // 组装完整 9D 配置
            const common::VectorJ full_q = common::combineJoints(arm_q, gimbal_q);

            return !pinocchio::computeCollisions(m_model, m_data, m_geom_model, m_geom_data, full_q, true);
        });

        // 设置起点和终点
        ob::ScopedState<ob::RealVectorStateSpace> start(space);
        ob::ScopedState<ob::RealVectorStateSpace> goal(space);
        for (int i = 0; i < DOF; ++i)
        {
            start[i] = q_start[i];
            goal[i] = q_goal[i];
        }
        ss.setStartAndGoalStates(start, goal);

        // 使用 RRTConnect
        ss.setPlanner(std::make_shared<og::RRTConnect>(ss.getSpaceInformation()));

        auto solved = ss.solve(1.0); // 最多 1 秒

        if (solved)
        {
            ss.simplifySolution();
            auto& path = ss.getSolutionPath();
            auto& states = path.getStates();

            plan.count = std::min(static_cast<int>(states.size()), TrajectoryPlan::MAX_WAYPOINTS);
            for (int i = 0; i < plan.count; ++i)
            {
                const auto* s = states[i]->as<ob::RealVectorStateSpace::StateType>();
                for (int j = 0; j < DOF; ++j)
                {
                    plan.waypoints[i][j] = s->values[j];
                }
            }
            plan.valid = true;
            m_logger->info("OMPL solved: {} waypoints, length={:.3f}", plan.count, path.length());
        }
        else
        {
            m_logger->warn("OMPL failed to find a solution");
            plan.valid = false;
        }

        return plan;
    }
}

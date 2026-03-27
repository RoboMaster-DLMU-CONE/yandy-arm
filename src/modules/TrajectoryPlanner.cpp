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
                                         const pinocchio::GeometryModel& geom_model,
                                         const TrajectoryConfig& config)
        : m_dt(dt)
        , m_ruckig(dt)
        , m_model(model)
        , m_data(m_model)
        , m_geom_model(geom_model)
        , m_geom_data(m_geom_model)
        , m_config(config)
    {
        m_logger = core::create_logger("TrajPlanner", spdlog::level::info);

        // Ruckig 运动学约束 (只针对机械臂 6 DoF)
        m_ruckig_input.max_velocity.setConstant(config.max_velocity);
        m_ruckig_input.max_acceleration.setConstant(config.max_acceleration);
        m_ruckig_input.max_jerk.setConstant(config.max_jerk);

        m_ruckig_input.current_position.setZero();
        m_ruckig_input.current_velocity.setZero();
        m_ruckig_input.current_acceleration.setZero();
        m_ruckig_input.target_position.setZero();
        m_ruckig_input.target_velocity.setZero();
        m_ruckig_input.target_acceleration.setZero();

        // 启动 OMPL 后台线程
        m_plan_thread = std::thread([this] { planLoop(); });
        m_logger->info("TrajectoryPlanner initialized, dt={:.4f}s, DOF={}, max_vel={:.2f}, max_acc={:.2f}, max_jerk={:.2f}",
                       dt, DOF, config.max_velocity, config.max_acceleration, config.max_jerk);
    }

    TrajectoryPlanner::~TrajectoryPlanner()
    {
        stopPlanThread();
    }

    void TrajectoryPlanner::stopPlanThread()
    {
        m_plan_running.store(false, std::memory_order_release);
        m_plan_cv.notify_one();
        if (m_plan_thread.joinable()) {
            // 使用限时 join 避免长时间阻塞
            // 注意：C++ 标准库没有 timed_join，这里只能普通 join
            // 但 planLoop 会在下次 check 时退出，OMPL 规划最多 2 秒
            m_plan_thread.join();
        }
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
            // 修复：检查是否还有未执行的 waypoint（索引从 0 开始）
            if (m_waypoint_idx < static_cast<int>(m_waypoints.size()))
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

    int TrajectoryPlanner::consumePlanResult()
    {
        auto plan = m_plan_result_buf.try_read();
        if (!plan.has_value())
            return 0;  // 无结果

        if (!plan->valid || plan->count <= 0)
            return -1;  // 规划失败或无效数据

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
        else
        {
            m_logger->error("OMPL plan loaded but waypoints empty!");
            return -1;
        }

        m_logger->info("Loaded OMPL plan with {} waypoints", plan->count);
        return 1;  // 成功
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

            // 执行 OMPL 规划，但需要定期检查停止信号
            // 由于 runOMPL 内部无法中断，我们只在调用前后检查
            auto result = runOMPL(req.q_start, req.q_goal);
            
            // 只在仍然运行时写入结果 (避免停止后写入无效数据)
            if (m_plan_running.load(std::memory_order_relaxed)) {
                m_plan_result_buf.write(std::move(result));
            }
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

        // 从 Pinocchio model 读取机械臂关节限位 (使用正确的 Pinocchio 索引)
        bool invalid_bounds = false;
        for (int i = 0; i < DOF; ++i)
        {
            const int idx = common::ARM_Q_INDICES[i];
            const double low = m_model.lowerPositionLimit[idx];
            const double high = m_model.upperPositionLimit[idx];
            // 记录到 bounds（即便无效也先记录，随后检测）
            bounds.setLow(i, low);
            bounds.setHigh(i, high);
            if (!(low < high)) {
                m_logger->error("OMPL invalid joint limits for arm dim {} (model idx {}): lower={} >= upper={}", i, idx, low, high);
                invalid_bounds = true;
            }
        }

        if (invalid_bounds) {
            // 返回无效的规划结果，避免抛出 OMPL 异常导致进程终止
            m_logger->warn("OMPL plan aborted due to invalid joint limits (see errors)");
            plan.valid = false;
            return plan;
        }

        // 打印模型信息与关节限位（便于诊断）
        m_logger->info("OMPL model.nq={}, DOF={} ", m_model.nq, DOF);
        for (int i = 0; i < DOF; ++i) {
            const int idx = common::ARM_Q_INDICES[i];
            m_logger->info("OMPL bounds dim {} -> model idx {} : lower={}, upper={}", i, idx,
                           bounds.low[i], bounds.high[i]);
        }

        space->setBounds(bounds);

        try {
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
            auto planner = std::make_shared<og::RRTConnect>(ss.getSpaceInformation());
            planner->setRange(m_config.rrt_range);
            ss.setPlanner(planner);

            auto solved = ss.solve(m_config.planning_timeout);

            if (solved)
            {
                ss.simplifySolution();
                auto& path = ss.getSolutionPath();
                
                // 插值路径以获得更平滑的轨迹
                path.interpolate(m_config.path_interpolation_points);  // 至少20个waypoints
                
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
        } catch (const ompl::Exception& e) {
            m_logger->error("OMPL threw exception: {}", e.what());
            plan.valid = false;
            return plan;
        } catch (const std::exception& e) {
            m_logger->error("Unexpected exception during OMPL: {}", e.what());
            plan.valid = false;
            return plan;
        } catch (...) {
            m_logger->error("Unknown error during OMPL planning");
            plan.valid = false;
            return plan;
        }

        // 返回结果（正常或捕获异常后都会到此）
        return plan;
    }
}
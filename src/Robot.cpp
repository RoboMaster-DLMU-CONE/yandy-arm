#include <yandy/Robot.hpp>
#include <chrono>
#include <algorithm>
#include <toml++/toml.hpp>

#define YANDY_ROBOT_CONFIG YANDY_CONFIG_PATH "robot.toml"


// ============================================================
// 构造 / 析构
// ============================================================

yandy::Robot::Robot(one::can::CanDriver& can)
    : m_arm_hw(can),
      m_effector(can)
{
    m_logger = core::create_logger("YandyRobot", spdlog::level::info);
    m_logger->info("try loading config from {}", YANDY_ROBOT_CONFIG);

    // 构造 TrajectoryPlanner (需要 m_solver 的 model)
    m_planner = std::make_unique<modules::TrajectoryPlanner>(
        DT, m_solver.getModel(), m_solver.getGeometryModel());

    // 注册 FSM 回调: 一次性硬件动作由 action 直接驱动
    modules::detail::FSMCallbacks cb;
    cb.on_enable = [this] { m_arm_hw.enable(); };
    cb.on_disable = [this] { m_arm_hw.disable(); };
    cb.on_open_claw = [this] { m_effector.openClaw(); };
    cb.on_close_claw = [this] { m_effector.closeClaw(); };
    cb.on_brake = [this] { m_planner->brake(); };
    cb.on_enter_store = [this](int idx) { m_store_pose_index = idx; };
    m_fsm.setCallbacks(cb);

    m_input.setCommandCb([this](const YandyControlCmd cmd)
    {
        m_fsm.processCmd(cmd);
    });

    auto tbl = toml::parse_file(YANDY_ROBOT_CONFIG);
    m_is_simulate = tbl["simulate"].value<bool>().value();
    if (m_is_simulate)
    {
        m_logger->info("Simulate mode is on.");

        // 加载手持相机在 base_link 系下的固定位姿
        auto trans = tbl["simulate_camera"]["translation"].as_array();
        auto rpy = tbl["simulate_camera"]["rpy"].as_array();

        Eigen::Vector3d t(
            trans->get(0)->value<double>().value(),
            trans->get(1)->value<double>().value(),
            trans->get(2)->value<double>().value());
        double roll = rpy->get(0)->value<double>().value();
        double pitch = rpy->get(1)->value<double>().value();
        double yaw = rpy->get(2)->value<double>().value();

        m_sim_cam_pose = Eigen::Isometry3d::Identity();
        m_sim_cam_pose.translate(t);
        m_sim_cam_pose.rotate(
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));

        m_logger->info("Simulate camera pose: t=[{:.3f},{:.3f},{:.3f}] rpy=[{:.3f},{:.3f},{:.3f}]",
                       t.x(), t.y(), t.z(), roll, pitch, yaw);
    }
}

yandy::Robot::~Robot()
{
    stop();
}

// ============================================================
// 生命周期
// ============================================================

void yandy::Robot::start()
{
    m_logger->info("Starting Robot...");

    // 初始化视觉子系统
    if (m_hik_driver.init() && m_detector.init())
    {
        m_vision_thread = std::thread([this] { visionLoop(); });
        m_logger->info("Vision thread launched.");
    }
    else
    {
        m_logger->warn("Vision subsystem init failed, running without vision.");
    }

    // 读取初始关节状态，预填充 cmd 防止首帧飞车
    m_arm_hw.read(m_arm_state);
    m_solver.updateKinematics(m_arm_state.q, m_arm_state.v, m_gimbal_state.q);
    m_arm_cmd.q_des = m_arm_state.q;
    m_arm_cmd.v_des.setZero();
    m_arm_cmd.tau_ff = m_solver.computeGravity();
    m_arm_cmd.kp.fill(20.0);
    m_arm_cmd.kd.fill(1.0);

    // 初始化 Ruckig 的当前状态
    m_planner->syncState(m_arm_state.q, m_arm_state.v);
    m_planner->updateGimbalState(m_gimbal_state.q);

    // 读取当前 store 关节位置
    for (size_t i = 0; i < 2; ++i)
    {
        m_store_pose[i] = m_solver.getStoreFrame(i);
    }

    m_logger->info("Entering main control loop at {}Hz.", static_cast<int>(1.0 / DT));

    // ---- 主控制循环 ----
    while (m_running.load(std::memory_order_relaxed))
    {
        const auto loop_start = std::chrono::steady_clock::now();

        // 1. 读取硬件状态 (机械臂 6 DoF)
        m_arm_hw.read(m_arm_state);
        
        // 2. 从输入获取云台状态
        const auto pack = m_input.getLatestCommand();
        updateGimbalFromPack(pack);
        
        // 3. 更新运动学 (传入分离的 arm/gimbal 状态)
        m_solver.updateKinematics(m_arm_state.q, m_arm_state.v, m_gimbal_state.q);
        m_planner->updateGimbalState(m_gimbal_state.q);

        // 4. 检查 OMPL 是否有新规划结果
        if (m_ompl_pending && m_planner->consumePlanResult())
        {
            m_ompl_pending = false;
            m_logger->info("OMPL plan loaded into trajectory planner");
        }

        // 5. 获取当前状态 (用于连续行为派发)
        const auto cur_state = m_fsm.getState();

        // 6. 按状态派发 (设置目标给 planner)
        switch (cur_state)
        {
        case YandyState::Manual:
            handleManual();
            break;
        case YandyState::Fetching:
            handleFetching();
            break;
        case YandyState::Store:
            handleStore();
            break;
        case YandyState::Disabled:
        case YandyState::Error:
        default:
            // 保持当前位置 + 重力补偿
            m_arm_cmd.q_des = m_arm_state.q;
            m_arm_cmd.v_des.setZero();
            m_arm_cmd.tau_ff = m_solver.computeGravity();
            m_arm_hw.write(m_arm_cmd);
            break;
        }

        // 7. 仿真步进 (真实硬件时 step() 为空)
        m_arm_hw.step(DT);

        // 8. 写入可视化数据
        {
            detail::RobotVizData vd;
            // 组装完整 9D 关节状态用于可视化
            vd.q = common::combineJoints(m_arm_state.q, m_gimbal_state.q);
            vd.ee_pose = m_solver.getEndEffectorPose();
            vd.target_pose = m_target_pose;
            vd.state = cur_state;

            // 每帧尝试读取视觉数据并转换到基座系
            auto vis = m_vision_buf.try_read();
            if (vis.has_value() && vis->valid)
            {
                vd.vision_valid = true;
                vd.vision_unit_pose = vis->unit_pose;
                vd.vision_unit_pose_base = m_is_simulate
                                               ? m_sim_cam_pose * vis->unit_pose
                                               : m_solver.transformObjectToBase(vis->unit_pose);
            }

            m_viz_buf.write(vd);
        }

        // 9. 定频
        std::this_thread::sleep_until(loop_start + std::chrono::duration<double>(DT));
    }

    m_logger->info("Main loop exited.");
}

void yandy::Robot::stop()
{
    if (!m_running.load(std::memory_order_acquire))
        return; // 已经停止
    m_running.store(false, std::memory_order_release);

    m_planner->stopPlanThread();

    if (m_vision_thread.joinable())
        m_vision_thread.join();

    m_arm_hw.disable();
    m_logger->info("Robot stopped.");
}

// ============================================================
// 云台状态更新
// ============================================================

void yandy::Robot::updateGimbalFromPack(const YandyControlPack& pack)
{
    // 从 YandyControlPack 提取云台状态
    m_gimbal_state.q[0] = static_cast<double>(pack.gimbal_z);
    m_gimbal_state.q[1] = static_cast<double>(pack.gimbal_yaw);
    m_gimbal_state.q[2] = static_cast<double>(pack.gimbal_pitch);
    // 云台速度暂时设为 0 (下位机不提供速度信息)
    m_gimbal_state.v.setZero();
}

// ============================================================
// 视觉线程
// ============================================================

void yandy::Robot::visionLoop()
{
    m_logger->info("Vision loop started.");
    cv::Mat frame;

    while (m_running.load(std::memory_order_relaxed))
    {
        if (!m_hik_driver.getLatestFrame(frame))
            continue;

        auto detections = m_detector.detect(frame);
        if (detections.empty())
            continue;

        // 取置信度最高的目标
        const auto& best = *std::ranges::max_element(detections,
                                                     [](const modules::EnergyUnit& a, const modules::EnergyUnit& b)
                                                     {
                                                         return a.confidence < b.confidence;
                                                     });

        Eigen::Isometry3d T_cam_obj;
        if (m_pose_solver.solve(best, T_cam_obj))
        {
            detail::VisionData vd;
            vd.valid = true;
            vd.unit_pose = T_cam_obj;
            m_vision_buf.write(vd);
        }
    }

    m_logger->info("Vision loop exited.");
}

// ============================================================
// 持续性状态处理
// ============================================================

bool yandy::Robot::solveAndPlan(const Eigen::Isometry3d& target_pose)
{
    m_target_pose = target_pose;
    m_arm_cmd.tau_ff = m_solver.computeGravity();

    // IK 求解 (只针对机械臂 6 DoF)
    auto q_sol = m_solver.solveIK(target_pose, m_arm_state.q, 0.01);
    if (!q_sol)
    {
        // IK 失败，Ruckig 继续执行当前轨迹 (或已停车)
        m_planner->update(m_arm_cmd.q_des, m_arm_cmd.v_des);
        m_arm_hw.write(m_arm_cmd);
        return false;
    }

    const auto& q_goal = q_sol.value();

    // 路径碰撞检测 (使用当前云台状态)
    if (m_solver.checkPathCollision(m_arm_state.q, q_goal))
    {
        // 路径有碰撞 → 停车 + 唤醒 OMPL
        if (!m_ompl_pending)
        {
            m_logger->warn("Path collision detected, braking and requesting OMPL plan");
            m_planner->brake();
            m_planner->requestPlan(m_arm_state.q, q_goal);
            m_ompl_pending = true;
        }
        // OMPL 正在规划中，Ruckig 继续执行停车/已有轨迹
        m_planner->update(m_arm_cmd.q_des, m_arm_cmd.v_des);
        m_arm_hw.write(m_arm_cmd);
        return false;
    }

    // 无碰撞 → 直接设目标
    m_planner->setTarget(q_goal);
    m_planner->update(m_arm_cmd.q_des, m_arm_cmd.v_des);
    m_arm_hw.write(m_arm_cmd);
    return true;
}

void yandy::Robot::handleManual()
{
    const auto pack = m_input.getLatestCommand();

    // 构造目标位姿 (基座坐标系)
    Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
    target.pretranslate(Eigen::Vector3d(pack.x, pack.y, pack.z));
    target.rotate(
        Eigen::AngleAxisd(pack.yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pack.pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(pack.roll, Eigen::Vector3d::UnitX()));

    solveAndPlan(target);
}

void yandy::Robot::handleFetching()
{
    auto vd = m_vision_buf.try_read();
    if (!vd.has_value() || !vd->valid)
    {
        // 无有效视觉数据，Ruckig 继续执行当前轨迹
        m_arm_cmd.tau_ff = m_solver.computeGravity();
        m_planner->update(m_arm_cmd.q_des, m_arm_cmd.v_des);
        m_arm_hw.write(m_arm_cmd);
        return;
    }

    // 将相机坐标系下的位姿转换到基座坐标系
    Eigen::Isometry3d target;
    if (m_is_simulate)
    {
        target = m_sim_cam_pose * vd->unit_pose;
    }
    else
    {
        target = m_solver.transformObjectToBase(vd->unit_pose);
    }

    solveAndPlan(target);
}

void yandy::Robot::handleStore()
{
    solveAndPlan(m_store_pose[m_store_pose_index]);
}

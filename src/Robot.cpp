#include <yandy/Robot.hpp>
#include <chrono>
#include <algorithm>

// 存储模式预设位姿 (基座坐标系) — 后续可改为从 config 加载
// TODO: 根据实际机械臂工作空间调整此位姿
const Eigen::Isometry3d yandy::Robot::STORE_POSE = []
{
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.pretranslate(Eigen::Vector3d(0.15, 0.0, 0.35));
    return pose;
}();

// ============================================================
// 构造 / 析构
// ============================================================

yandy::Robot::Robot()
{
    m_input.setCommandCb([this](const YandyControlCmd cmd)
    {
        m_fsm.processCmd(cmd);
    });
    m_logger = core::create_logger("YandyRobot", spdlog::level::info);
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
    m_arm_hw.read(m_state);
    m_solver.updateKinematics(m_state.q, m_state.v);
    m_cmd.q_des = m_state.q;
    m_cmd.v_des.setZero();
    m_cmd.tau_ff = m_solver.computeGravity();

    m_logger->info("Entering main control loop at {}Hz.", static_cast<int>(1.0 / DT));

    // ---- 主控制循环 ----
    while (m_running.load(std::memory_order_relaxed))
    {
        const auto loop_start = std::chrono::steady_clock::now();

        // 1. 读取硬件 & 更新运动学
        m_arm_hw.read(m_state);
        m_solver.updateKinematics(m_state.q, m_state.v);

        // 2. 检测状态转换
        const auto cur_state = m_fsm.getState();
        if (cur_state != m_prev_state)
        {
            onStateTransition(m_prev_state, cur_state);
            m_prev_state = cur_state;
        }

        // 3. 按状态派发 (设置 m_cmd)
        switch (cur_state)
        {
        case YandyState::Manual:
            handleManual();
            m_arm_hw.write(m_cmd);
            break;
        case YandyState::Fetching:
            handleFetching();
            m_arm_hw.write(m_cmd);
            break;
        case YandyState::Store:
            handleStore();
            m_arm_hw.write(m_cmd);
            break;
        case YandyState::Disabled:
        case YandyState::Error:
        default:
            // idle — 不发送指令
            break;
        }

        // 4. 仿真步进 (真实硬件时 step() 为空)
        m_arm_hw.step(DT);

        // 5. 定频
        std::this_thread::sleep_until(loop_start + std::chrono::duration<double>(DT));
    }

    m_logger->info("Main loop exited.");
}

void yandy::Robot::stop()
{
    if (!m_running.load(std::memory_order_acquire))
        return; // 已经停止
    m_running.store(false, std::memory_order_release);

    if (m_vision_thread.joinable())
        m_vision_thread.join();

    m_arm_hw.disable();
    m_logger->info("Robot stopped.");
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
        const auto& best = *std::max_element(detections.begin(), detections.end(),
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
// 状态转换一次性动作
// ============================================================

void yandy::Robot::onStateTransition(const YandyState from, const YandyState to)
{
    m_logger->info("State transition: {} -> {}", format_as(from), format_as(to));

    // ---- 进入新状态 ----
    switch (to)
    {
    case YandyState::Manual:
        if (from == YandyState::Disabled)
        {
            m_arm_hw.enable();
        }
        else if (from == YandyState::Fetching)
        {
            m_effector.closeClaw();
        }
        else if (from == YandyState::Store)
        {
            // 存矿完成: 松开夹爪释放; 取矿完成: 闭合夹爪抓住
            // store_finish 已经更新了 mineral_attached:
            //   存矿后 mineral_attached = false, 取矿后 mineral_attached = true
            if (m_fsm.hasMineralAttached())
                m_effector.closeClaw(); // 取矿完成，抓住
            else
                m_effector.openClaw(); // 存矿完成，释放
        }
        break;

    case YandyState::Disabled:
        m_arm_hw.disable();
        break;

    case YandyState::Fetching:
        m_effector.openClaw(); // 张开准备抓取
        break;

    case YandyState::Store:
        // 进入时 mineral_attached 尚未被 store_finish 修改
        if (!m_fsm.hasMineralAttached())
            m_effector.openClaw(); // 准备取矿
        break;

    case YandyState::Error:
        m_arm_hw.disable();
        break;

    default:
        break;
    }
}

// ============================================================
// 持续性状态处理
// ============================================================

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

    // 重力补偿
    m_cmd.tau_ff = m_solver.computeGravity();

    // IK 求解
    auto q_sol = m_solver.solveIK(target, m_state.q);
    if (q_sol.has_value())
    {
        m_cmd.q_des = q_sol.value();
        m_cmd.v_des = (m_cmd.q_des - m_state.q) / DT;
        m_cmd.v_des = m_cmd.v_des.cwiseMin(5.0).cwiseMax(-5.0);
    }
    else
    {
        m_cmd.q_des = m_state.q;
        m_cmd.v_des.setZero();
    }
}

void yandy::Robot::handleFetching()
{
    auto vd = m_vision_buf.try_read();
    if (!vd.has_value() || !vd->valid)
    {
        // 无有效视觉数据，保持当前位置 + 重力补偿
        m_cmd.q_des = m_state.q;
        m_cmd.v_des.setZero();
        m_cmd.tau_ff = m_solver.computeGravity();
        return;
    }

    // 将相机坐标系下的位姿转换到基座坐标系
    Eigen::Isometry3d target = m_solver.transformObjectToBase(vd->unit_pose);

    m_cmd.tau_ff = m_solver.computeGravity();

    auto q_sol = m_solver.solveIK(target, m_state.q);
    if (q_sol.has_value())
    {
        m_cmd.q_des = q_sol.value();
        m_cmd.v_des = (m_cmd.q_des - m_state.q) / DT;
        m_cmd.v_des = m_cmd.v_des.cwiseMin(5.0).cwiseMax(-5.0);
    }
    else
    {
        m_cmd.q_des = m_state.q;
        m_cmd.v_des.setZero();
    }
}

void yandy::Robot::handleStore()
{
    m_cmd.tau_ff = m_solver.computeGravity();

    auto q_sol = m_solver.solveIK(STORE_POSE, m_state.q);
    if (q_sol.has_value())
    {
        m_cmd.q_des = q_sol.value();
        m_cmd.v_des = (m_cmd.q_des - m_state.q) / DT;
        m_cmd.v_des = m_cmd.v_des.cwiseMin(5.0).cwiseMax(-5.0);
    }
    else
    {
        m_cmd.q_des = m_state.q;
        m_cmd.v_des.setZero();
    }
}

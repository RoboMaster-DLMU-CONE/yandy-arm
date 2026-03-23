#include <algorithm>
#include <chrono>
#include <random>
#include <toml++/toml.hpp>
#include <yandy/Robot.hpp>

// ============================================================
// 构造 / 析构
// ============================================================

yandy::Robot::Robot(one::can::CanDriver &can) : m_arm_hw(can), m_effector(can) {
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
  cb.on_enter_fetch = [this] { resetFetchState(); };
  m_fsm.setCallbacks(cb);

  m_input.setCommandCb(
      [this](const YandyControlCmd cmd) { m_fsm.processCmd(cmd); });

  auto tbl = toml::parse_file(YANDY_ROBOT_CONFIG);
  m_is_simulate = tbl["simulate"].value<bool>().value();
  if (m_is_simulate) {
    m_logger->info("Simulate mode is on.");

    // 加载手持相机在 base_link 系下的固定位姿
    auto trans = tbl["simulate_camera"]["translation"].as_array();
    auto rpy = tbl["simulate_camera"]["rpy"].as_array();

    Eigen::Vector3d t(trans->get(0)->value<double>().value(),
                      trans->get(1)->value<double>().value(),
                      trans->get(2)->value<double>().value());
    double roll = rpy->get(0)->value<double>().value();
    double pitch = rpy->get(1)->value<double>().value();
    double yaw = rpy->get(2)->value<double>().value();

    m_sim_cam_pose = Eigen::Isometry3d::Identity();
    m_sim_cam_pose.translate(t);
    m_sim_cam_pose.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));

    m_logger->info("Simulate camera pose: t=[{:.3f},{:.3f},{:.3f}] "
                   "rpy=[{:.3f},{:.3f},{:.3f}]",
                   t.x(), t.y(), t.z(), roll, pitch, yaw);

    // 加载仿真视觉配置
    if (auto sim_vis = tbl["simulate_vision"]) {
      auto mode = sim_vis["mode"].value<std::string>().value_or("fixed");
      m_sim_vision_random = (mode == "random");

      if (!m_sim_vision_random) {
        // 固定模式：读取固定位姿
        auto pos = sim_vis["position"].as_array();
        auto rpy_arr = sim_vis["rpy"].as_array();
        if (pos && rpy_arr) {
          Eigen::Vector3d unit_t(pos->get(0)->value<double>().value(),
                                 pos->get(1)->value<double>().value(),
                                 pos->get(2)->value<double>().value());
          double unit_roll = rpy_arr->get(0)->value<double>().value();
          double unit_pitch = rpy_arr->get(1)->value<double>().value();
          double unit_yaw = rpy_arr->get(2)->value<double>().value();

          m_sim_unit_pose = Eigen::Isometry3d::Identity();
          m_sim_unit_pose.translate(unit_t);
          m_sim_unit_pose.rotate(
              Eigen::AngleAxisd(unit_yaw, Eigen::Vector3d::UnitZ()) *
              Eigen::AngleAxisd(unit_pitch, Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(unit_roll, Eigen::Vector3d::UnitX()));

          m_logger->info("Simulate vision (fixed, base_link): "
                         "t=[{:.3f},{:.3f},{:.3f}] rpy=[{:.3f},{:.3f},{:.3f}]",
                         unit_t.x(), unit_t.y(), unit_t.z(), unit_roll,
                         unit_pitch, unit_yaw);
        }
      } else {
        // 随机模式：读取范围参数
        auto x_range = sim_vis["random_x_range"].as_array();
        auto y_range = sim_vis["random_y_range"].as_array();
        auto z_range = sim_vis["random_z_range"].as_array();
        auto roll_range = sim_vis["random_roll_range"].as_array();
        auto pitch_range = sim_vis["random_pitch_range"].as_array();

        if (x_range && y_range && z_range && roll_range && pitch_range) {
          m_sim_x_range = {x_range->get(0)->value<double>().value(),
                           x_range->get(1)->value<double>().value()};
          m_sim_y_range = {y_range->get(0)->value<double>().value(),
                           y_range->get(1)->value<double>().value()};
          m_sim_z_range = {z_range->get(0)->value<double>().value(),
                           z_range->get(1)->value<double>().value()};
          m_sim_roll_range = {roll_range->get(0)->value<double>().value(),
                              roll_range->get(1)->value<double>().value()};
          m_sim_pitch_range = {pitch_range->get(0)->value<double>().value(),
                               pitch_range->get(1)->value<double>().value()};

          m_logger->info(
              "Simulate vision (random, base_link): x=[{:.3f},{:.3f}] "
              "y=[{:.3f},{:.3f}] "
              "z=[{:.3f},{:.3f}] roll=[{:.3f},{:.3f}] pitch=[{:.3f},{:.3f}]",
              m_sim_x_range[0], m_sim_x_range[1], m_sim_y_range[0],
              m_sim_y_range[1], m_sim_z_range[0], m_sim_z_range[1],
              m_sim_roll_range[0], m_sim_roll_range[1], m_sim_pitch_range[0],
              m_sim_pitch_range[1]);
        }
        // 初始化时生成第一个随机位姿
        generateRandomUnitPose();
      }
    }
  }
}

yandy::Robot::~Robot() { stop(); }

// ============================================================
// 生命周期
// ============================================================

void yandy::Robot::start() {
  m_logger->info("Starting Robot...");

  // 初始化视觉子系统
  if (m_hik_driver.init() && m_detector.init()) {
    m_vision_thread = std::thread([this] { visionLoop(); });
    m_logger->info("Vision thread launched.");
  } else if (m_is_simulate) {
    // 仿真模式下视觉初始化失败，启用仿真视觉
    m_sim_vision_enabled = true;
    m_vision_thread = std::thread([this] { simVisionLoop(); });
    m_logger->info("Real vision init failed, using simulated vision.");
  } else {
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
  for (size_t i = 0; i < 2; ++i) {
    m_store_pose[i] = m_solver.getStoreFrame(i);
  }

  m_logger->info("Entering main control loop at {}Hz.",
                 static_cast<int>(1.0 / DT));

  // ---- 主控制循环 ----
  while (m_running.load(std::memory_order_relaxed)) {
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
    if (m_ompl_pending) {
      int result = m_planner->consumePlanResult();
      if (result == 1) {
        // 规划成功
        m_ompl_pending = false;
        m_ompl_executing = true;
        m_ompl_fail_cooldown = 0;
        m_logger->info("OMPL plan loaded into trajectory planner");
      } else if (result == -1) {
        // 规划失败，设置冷却期 (约0.5秒 @250Hz = 125帧)
        m_ompl_pending = false;
        m_ompl_fail_cooldown = 125;
        m_logger->warn("OMPL plan failed, cooldown before retry");
      }
    }

    // 5. 减少 OMPL 失败冷却计数
    if (m_ompl_fail_cooldown > 0) {
      --m_ompl_fail_cooldown;
    }

    // 6. 检查 OMPL 规划是否执行完毕
    if (m_ompl_executing && m_planner->isFinished()) {
      m_ompl_executing = false;
      // 执行完成后也设置短冷却，避免立即重新检测碰撞
      m_ompl_fail_cooldown = 50; // 约0.2秒
      m_logger->info("OMPL plan execution finished");
    }

    // 7. 获取当前状态 (用于连续行为派发)
    const auto cur_state = m_fsm.getState();

    // 8. 按状态派发 (设置目标给 planner)
    switch (cur_state) {
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
      if (vis.has_value() && vis->valid) {
        vd.vision_valid = true;
        vd.vision_unit_pose = vis->unit_pose;
        vd.vision_unit_pose_base =
            m_is_simulate ? m_sim_cam_pose * vis->unit_pose
                          : m_solver.transformObjectToBase(vis->unit_pose);
      }

      m_viz_buf.write(vd);
    }

    // 9. 定频
    std::this_thread::sleep_until(loop_start +
                                  std::chrono::duration<double>(DT));
  }

  m_logger->info("Main loop exited.");
}

void yandy::Robot::stop() {
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

void yandy::Robot::updateGimbalFromPack(const YandyControlPack &pack) {
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

void yandy::Robot::visionLoop() {
  m_logger->info("Vision loop started.");
  cv::Mat frame;

  while (m_running.load(std::memory_order_relaxed)) {
    if (!m_hik_driver.getLatestFrame(frame))
      continue;

    auto detections = m_detector.detect(frame);
    if (detections.empty())
      continue;

    // 取置信度最高的目标
    const auto &best =
        *std::ranges::max_element(detections, [](const modules::EnergyUnit &a,
                                                 const modules::EnergyUnit &b) {
          return a.confidence < b.confidence;
        });

    Eigen::Isometry3d T_cam_obj;
    if (m_pose_solver.solve(best, T_cam_obj)) {
      detail::VisionData vd;
      vd.valid = true;
      vd.unit_pose = T_cam_obj;
      m_vision_buf.write(vd);
    }
  }

  m_logger->info("Vision loop exited.");
}

// ============================================================
// 仿真视觉
// ============================================================

void yandy::Robot::generateRandomUnitPose() {
  static std::random_device rd;
  static std::mt19937 gen(rd());

  std::uniform_real_distribution<> x_dist(m_sim_x_range[0], m_sim_x_range[1]);
  std::uniform_real_distribution<> y_dist(m_sim_y_range[0], m_sim_y_range[1]);
  std::uniform_real_distribution<> z_dist(m_sim_z_range[0], m_sim_z_range[1]);
  std::uniform_real_distribution<> roll_dist(m_sim_roll_range[0],
                                             m_sim_roll_range[1]);
  std::uniform_real_distribution<> pitch_dist(m_sim_pitch_range[0],
                                              m_sim_pitch_range[1]);

  double x = x_dist(gen);
  double y = y_dist(gen);
  double z = z_dist(gen);
  double roll = roll_dist(gen);
  double pitch = pitch_dist(gen);
  double yaw = 0.0; // yaw 不确定，设为 0

  m_sim_unit_pose = Eigen::Isometry3d::Identity();
  m_sim_unit_pose.translate(Eigen::Vector3d(x, y, z));
  m_sim_unit_pose.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));

  m_logger->info("Generated random unit pose (base_link): "
                 "t=[{:.3f},{:.3f},{:.3f}] rp=[{:.3f},{:.3f}]",
                 x, y, z, roll, pitch);
}

void yandy::Robot::simVisionLoop() {
  m_logger->info("Simulated vision loop started.");

  while (m_running.load(std::memory_order_relaxed)) {
    // 以固定频率发布仿真视觉数据 (模拟30Hz相机)
    // m_sim_unit_pose 是 baselink 系，需要转换到相机系
    // T_cam_obj = T_cam_base * T_base_obj
    Eigen::Isometry3d T_cam_obj = m_sim_cam_pose.inverse() * m_sim_unit_pose;

    detail::VisionData vd;
    vd.valid = true;
    vd.unit_pose = T_cam_obj;
    m_vision_buf.write(vd);

    std::this_thread::sleep_for(std::chrono::milliseconds(33));
  }

  m_logger->info("Simulated vision loop exited.");
}

// ============================================================
// 持续性状态处理
// ============================================================

bool yandy::Robot::solveAndPlan(const Eigen::Isometry3d &target_pose) {
  m_target_pose = target_pose;
  m_arm_cmd.tau_ff = m_solver.computeGravity();

  // 如果 OMPL 正在等待或执行中，让 Ruckig 继续跟踪 waypoints，不重新规划
  if (m_ompl_pending || m_ompl_executing) {
    m_planner->update(m_arm_cmd.q_des, m_arm_cmd.v_des);
    m_arm_hw.write(m_arm_cmd);
    return false;
  }

  // IK 求解 (只针对机械臂 6 DoF)
  auto q_sol = m_solver.solveIK(target_pose, m_arm_state.q, 0.01);
  if (!q_sol) {
    // IK 失败，Ruckig 继续执行当前轨迹 (或已停车)
    m_planner->update(m_arm_cmd.q_des, m_arm_cmd.v_des);
    m_arm_hw.write(m_arm_cmd);
    return false;
  }

  const auto &q_goal = q_sol.value();

  // 路径碰撞检测 (使用当前云台状态)
  // 如果在 OMPL 失败冷却期内，跳过碰撞检测直接尝试移动
  bool collision = m_solver.checkPathCollision(m_arm_state.q, q_goal);
  if (m_ompl_fail_cooldown == 0 && collision) {
    // 路径有碰撞 → 停车 + 唤醒 OMPL
    m_logger->warn("Path collision detected, braking and requesting OMPL plan");
    m_planner->brake();
    m_planner->requestPlan(m_arm_state.q, q_goal);
    m_ompl_pending = true;
    m_planner->update(m_arm_cmd.q_des, m_arm_cmd.v_des);
    m_arm_hw.write(m_arm_cmd);
    return false;
  }

  // 无碰撞（或冷却期内跳过检测）→ 直接设目标
  m_planner->setTarget(q_goal);
  m_planner->update(m_arm_cmd.q_des, m_arm_cmd.v_des);
  m_arm_hw.write(m_arm_cmd);
  return true;
}

void yandy::Robot::handleManual() {
  const auto pack = m_input.getLatestCommand();

  // 构造目标位姿 (基座坐标系)
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.pretranslate(Eigen::Vector3d(pack.x, pack.y, pack.z));
  target.rotate(Eigen::AngleAxisd(pack.yaw, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(pack.pitch, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(pack.roll, Eigen::Vector3d::UnitX()));

  solveAndPlan(target);
}

void yandy::Robot::handleFetching() {
  // 使用内部 m_fetch_phase 来跟踪抓取子阶段
  switch (m_fetch_phase) {
  case FetchPhase::Seeking:
    handleSeeking();
    break;
  case FetchPhase::PreGrasp:
    handlePreGrasp();
    break;
  case FetchPhase::Approaching:
    handleApproaching();
    break;
  case FetchPhase::Withdrawing:
    handleWithdrawing();
    break;
  }
}

void yandy::Robot::handleSeeking() {
  auto vd = m_vision_buf.try_read();
  if (!vd.has_value() || !vd->valid) {
    // 无有效视觉数据，保持当前位置
    m_arm_cmd.tau_ff = m_solver.computeGravity();
    m_planner->update(m_arm_cmd.q_des, m_arm_cmd.v_des);
    m_arm_hw.write(m_arm_cmd);
    return;
  }

  // 将相机坐标系下的位姿转换到基座坐标系
  Eigen::Isometry3d target_pose;
  if (m_is_simulate) {
    target_pose = m_sim_cam_pose * vd->unit_pose;
  } else {
    target_pose = m_solver.transformObjectToBase(vd->unit_pose);
  }

  // 更新位姿历史
  m_pose_history.push_back(target_pose);
  if (m_pose_history.size() > STABILITY_WINDOW) {
    m_pose_history.pop_front();
  }

  // 检查稳定性
  if (isPoseStable()) {
    // 锁定目标
    m_locked_target_pos = target_pose.translation();

    // 从旋转矩阵提取逼近方向 (末端 Z 轴)
    m_locked_approach_dir = target_pose.rotation().col(2);

    m_current_standoff = PREGRASP_DISTANCE;
    m_pose_history.clear();

    m_logger->info("Pose stable, locking target at [{:.3f}, {:.3f}, {:.3f}]",
                   m_locked_target_pos.x(), m_locked_target_pos.y(),
                   m_locked_target_pos.z());
    m_logger->info("Approach direction: [{:.3f}, {:.3f}, {:.3f}]",
                   m_locked_approach_dir.x(), m_locked_approach_dir.y(),
                   m_locked_approach_dir.z());

    const Eigen::Vector3d pregrasp =
        m_locked_target_pos - m_locked_approach_dir * PREGRASP_DISTANCE;
    m_logger->info("Pre-grasp position: [{:.3f}, {:.3f}, {:.3f}]", pregrasp.x(),
                   pregrasp.y(), pregrasp.z());

    // 切换到 PreGrasp 阶段
    m_fetch_phase = FetchPhase::PreGrasp;
    return;
  }

  // 尚未稳定，持续追踪（使用 5DoF IK）
  const Eigen::Vector3d approach_dir = target_pose.rotation().col(2);
  const Eigen::Vector3d pregrasp_pos =
      target_pose.translation() - approach_dir * PREGRASP_DISTANCE;

  solveAndPlan5DoF(pregrasp_pos, approach_dir);
}

void yandy::Robot::handlePreGrasp() {
  // 移动到预抓取点
  const Eigen::Vector3d pregrasp_pos =
      m_locked_target_pos - m_locked_approach_dir * PREGRASP_DISTANCE;

  if (!solveAndPlan5DoF(pregrasp_pos, m_locked_approach_dir)) {
    return;
  }

  // 检查是否到达预抓取点
  const Eigen::Vector3d ee_pos = m_solver.getEndEffectorPose().translation();
  const double dist = (ee_pos - pregrasp_pos).norm();

  if (dist < 0.01 && m_planner->isFinished()) // 10mm 阈值 + 轨迹完成
  {
    m_logger->info("Reached pre-grasp point, distance: {:.3f}m", dist);
    m_fetch_phase = FetchPhase::Approaching;
  }
}

void yandy::Robot::handleApproaching() {
  // 直线逼近：逐步减小 standoff 距离
  m_current_standoff -= APPROACH_SPEED * DT;

  if (m_current_standoff <= 0.0) {
    m_current_standoff = 0.0;
  }

  const Eigen::Vector3d current_target =
      m_locked_target_pos - m_locked_approach_dir * m_current_standoff;

  if (!solveAndPlan5DoF(current_target, m_locked_approach_dir)) {
    return;
  }

  // 检查是否到达目标
  if (m_current_standoff <= 0.0 && m_planner->isFinished()) {
    m_logger->info("Reached grasp target, closing claw");
    m_effector.closeClaw();

    // 切换到 Withdrawing 阶段
    m_fetch_phase = FetchPhase::Withdrawing;
    m_logger->info("Grasp complete, starting withdrawal...");
  }
}

void yandy::Robot::handleWithdrawing() {
  // 直线撤回：逐步增加 standoff 距离
  m_current_standoff += APPROACH_SPEED * DT;

  if (m_current_standoff >= PREGRASP_DISTANCE) {
    m_current_standoff = PREGRASP_DISTANCE;
  }

  const Eigen::Vector3d current_target =
      m_locked_target_pos - m_locked_approach_dir * m_current_standoff;

  if (!solveAndPlan5DoF(current_target, m_locked_approach_dir)) {
    return;
  }

  // 检查是否完成撤回
  if (m_current_standoff >= PREGRASP_DISTANCE && m_planner->isFinished()) {
    m_logger->info("Withdrawal complete, exiting fetch mode");

    // 仿真模式下随机生成下一个能量单元位姿
    if (m_sim_vision_enabled && m_sim_vision_random) {
      generateRandomUnitPose();
    }

    // 抓取完成后自动退出 FetchingMode (发送 CMD_SWITCH_FETCH)
    m_fsm.processCmd(YandyControlCmd::CMD_SWITCH_FETCH);
  }
}

void yandy::Robot::resetFetchState() {
  m_fetch_phase = FetchPhase::Seeking;
  m_pose_history.clear();
  m_current_standoff = PREGRASP_DISTANCE;
  m_locked_target_pos.setZero();
  m_locked_approach_dir = Eigen::Vector3d::UnitZ();
}

bool yandy::Robot::isPoseStable() const {
  if (m_pose_history.size() < STABILITY_WINDOW) {
    return false;
  }

  // 计算位置均值
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto &p : m_pose_history) {
    mean += p.translation();
  }
  mean /= static_cast<double>(m_pose_history.size());

  // 计算位置方差
  double variance = 0.0;
  for (const auto &p : m_pose_history) {
    variance += (p.translation() - mean).squaredNorm();
  }
  variance /= static_cast<double>(m_pose_history.size());

  return std::sqrt(variance) < STABILITY_THRESHOLD;
}

Eigen::Vector3d yandy::Robot::computeApproachDirection(double roll,
                                                       double pitch) const {
  // 从欧拉角计算逼近方向 (末端 Z 轴在世界系下的方向)
  // 假设 yaw = 0，只使用 roll 和 pitch
  const Eigen::Matrix3d R =
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();

  return R.col(2); // Z 轴
}

bool yandy::Robot::solveAndPlan5DoF(const Eigen::Vector3d &target_pos,
                                    const Eigen::Vector3d &approach_dir) {
  // 从 approach_dir 构造完整 6DoF 目标位姿：
  // 将世界 Z 轴 [0,0,1] 旋转到 approach_dir 的最短旋转，
  // 对 approach=[1,0,0] 恰好给出 Ry(π/2)，即 rpy=[0,π/2,0]
  const Eigen::Vector3d z_des = approach_dir.normalized();
  const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
  Eigen::Matrix3d R_des;
  const Eigen::Vector3d rot_axis = up.cross(z_des);
  const double sin_a = rot_axis.norm();
  const double cos_a = up.dot(z_des);
  if (sin_a < 1e-6) {
    // 已对齐或反向：对齐用 Identity，反向绕 X 旋转 π
    R_des = (cos_a > 0) ? Eigen::Matrix3d::Identity()
                        : Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
                              .toRotationMatrix();
  } else {
    R_des = Eigen::AngleAxisd(std::acos(cos_a), rot_axis.normalized())
                .toRotationMatrix();
  }

  m_target_pose = Eigen::Isometry3d::Identity();
  m_target_pose.translate(target_pos);
  m_target_pose.rotate(R_des);

  m_arm_cmd.tau_ff = m_solver.computeGravity();

  // 如果 OMPL 正在等待或执行中，让 Ruckig 继续跟踪 waypoints，不重新规划
  if (m_ompl_pending || m_ompl_executing) {
    m_planner->update(m_arm_cmd.q_des, m_arm_cmd.v_des);
    m_arm_hw.write(m_arm_cmd);
    return false;
  }

  // 用 6DoF IK 求解（5DoF 叉积误差在 approach=[1,0,0] 时退化，改用 6DoF）
  auto q_sol = m_solver.solveIK(m_target_pose, m_arm_state.q, 0.01);
  if (!q_sol) {
    static int ik_fail_count = 0;
    if (++ik_fail_count % 100 == 1) {
      m_logger->warn("5DoF IK failed for target [{:.3f}, {:.3f}, {:.3f}], "
                     "approach [{:.3f}, {:.3f}, {:.3f}]",
                     target_pos.x(), target_pos.y(), target_pos.z(),
                     approach_dir.x(), approach_dir.y(), approach_dir.z());
    }
    m_planner->update(m_arm_cmd.q_des, m_arm_cmd.v_des);
    m_arm_hw.write(m_arm_cmd);
    return false;
  }

  const auto &q_goal = q_sol.value();

  // 路径碰撞检测
  // 如果在 OMPL 失败冷却期内，跳过碰撞检测直接尝试移动
  if (m_ompl_fail_cooldown == 0 &&
      m_solver.checkPathCollision(m_arm_state.q, q_goal)) {
    m_logger->warn("Path collision detected, requesting OMPL plan");
    m_planner->brake();
    m_planner->requestPlan(m_arm_state.q, q_goal);
    m_ompl_pending = true;
    m_planner->update(m_arm_cmd.q_des, m_arm_cmd.v_des);
    m_arm_hw.write(m_arm_cmd);
    return false;
  }

  // 无碰撞（或冷却期内跳过检测），设置目标
  m_planner->setTarget(q_goal);
  m_planner->update(m_arm_cmd.q_des, m_arm_cmd.v_des);
  m_arm_hw.write(m_arm_cmd);
  return true;
}

void yandy::Robot::handleStore() {
  solveAndPlan(m_store_pose[m_store_pose_index]);
}

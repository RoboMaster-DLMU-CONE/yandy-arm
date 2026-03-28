#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <toml++/toml.hpp>
#include <yandy/Robot.hpp>

// ============================================================
// 构造 / 析构
// ============================================================

yandy::Robot::Robot(one::can::CanDriver &can) : m_arm_hw(can), m_effector(can) {
  m_logger = core::create_logger("YandyRobot", spdlog::level::info);
  m_logger->info("try loading config from {}", YANDY_ROBOT_CONFIG);

  // ── 从 solver.toml 决定实例化固定基座还是浮动基座的 DynamicsSolver ──────
  {
    bool use_floating = false;
    try {
      auto stbl = toml::parse_file(YANDY_CONFIG_PATH "solver.toml");
      use_floating = stbl["floating_base"].value<bool>().value_or(false);
    } catch (...) {
      m_logger->warn("Cannot read solver.toml for floating_base, defaulting to "
                     "fixed-base.");
    }

    // optional<variant> 的正确 emplace 写法:
    // optional::emplace(args...) 将 args... 转发给 variant 的构造函数，
    // 而 variant(in_place_type<T>) 构造 T 候选值 (default-construct T).
    if (use_floating) {
      m_logger->info("Instantiating DynamicsSolver<true> (floating-base).");
      m_solver_var.emplace(std::in_place_type<modules::DynamicsSolverFloating>);
    } else {
      m_logger->info("Instantiating DynamicsSolver<false> (fixed-base).");
      m_solver_var.emplace(std::in_place_type<modules::DynamicsSolverFixed>);
    }
  }

  // 构造 TrajectoryPlanner (需要 m_solver_var 中 model 的引用)
  // 先从 trajectory.toml 读取配置参数
  modules::TrajectoryConfig traj_config;
  try {
    auto traj_tbl = toml::parse_file(YANDY_CONFIG_PATH "trajectory.toml");
    traj_config.max_velocity =
        traj_tbl["max_velocity"].value<double>().value_or(6.0);
    traj_config.max_acceleration =
        traj_tbl["max_acceleration"].value<double>().value_or(20.0);
    traj_config.max_jerk = traj_tbl["max_jerk"].value<double>().value_or(100.0);
    traj_config.planning_timeout =
        traj_tbl["planning_timeout"].value<double>().value_or(2.0);
    traj_config.rrt_range = traj_tbl["rrt_range"].value<double>().value_or(0.5);
    traj_config.path_interpolation_points =
        traj_tbl["path_interpolation_points"].value<int>().value_or(20);
    m_logger->info("Loaded trajectory config: max_vel={:.2f}, max_acc={:.2f}, "
                   "max_jerk={:.2f}",
                   traj_config.max_velocity, traj_config.max_acceleration,
                   traj_config.max_jerk);
  } catch (const toml::parse_error &e) {
    m_logger->warn("Failed to parse trajectory.toml, using default config: {}",
                   e.what());
  }

  m_planner = std::make_unique<modules::TrajectoryPlanner>(
      DT,
      withSolver([](auto &s) -> pinocchio::Model & { return s.getModel(); }),
      withSolver([](auto &s) -> const pinocchio::GeometryModel & {
        return s.getGeometryModel();
      }),
      traj_config);

  // 注册 FSM 回调: 一次性硬件动作由 action 直接驱动
  modules::detail::FSMCallbacks cb;
  cb.on_enable = [this] { m_arm_hw.enable(); };
  cb.on_disable = [this] { m_arm_hw.disable(); };
  cb.on_open_claw = [this] { m_effector.openClaw(); };
  cb.on_close_claw = [this] { m_effector.closeClaw(); };
  cb.on_brake = [this] { m_planner->brake(); };
  cb.on_enter_store = [this](int idx) {
    m_store_pose_index = idx;
    // 存矿(有矿): 先到上方再下降; 取矿(无矿): 直接到 store 位置
    m_store_phase = m_fsm.hasMineralAttached() ? StorePhase::Approaching
                                               : StorePhase::PreFetch;
  };
  cb.on_enter_fetch = [this] { resetFetchState(); };
  m_fsm.setCallbacks(cb);

  m_input.setCommandCb(
      [this](const YandyControlCmd cmd) { m_fsm.processCmd(cmd); });

  auto tbl = toml::parse_file(YANDY_ROBOT_CONFIG);
  m_is_simulate = tbl["simulate"].value<bool>().value();

  // 读取强制仿真视觉选项（允许在真实模式下使用仿真视觉）
  m_force_simulate_vision =
      tbl["force_simulate_vision"].value<bool>().value_or(false);

  // 如果启用仿真模式或强制仿真视觉，加载仿真相机和视觉配置
  if (m_is_simulate || m_force_simulate_vision) {
    if (m_is_simulate) {
      m_logger->info("Simulate mode is on.");
    } else {
      m_logger->info("force_simulate_vision is enabled (real hardware with "
                     "simulated vision).");
    }

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

  // 读取抓取参数
  if (auto fetch = tbl["fetch"]) {
    m_stability_window =
        fetch["stability_window"].value<int>().value_or(m_stability_window);
    m_stability_threshold =
        fetch["stability_threshold"].value<double>().value_or(
            m_stability_threshold);
    m_pregrasp_distance = fetch["pregrasp_distance"].value<double>().value_or(
        m_pregrasp_distance);
    m_approach_speed =
        fetch["approach_speed"].value<double>().value_or(m_approach_speed);
    m_extract_distance =
        fetch["extract_distance"].value<double>().value_or(m_extract_distance);
    m_current_standoff = m_pregrasp_distance;
  }
  m_logger->info(
      "Fetch params: stability_window={}, stability_threshold={:.4f}, "
      "pregrasp_distance={:.3f}, approach_speed={:.3f}, "
      "extract_distance={:.3f}",
      m_stability_window, m_stability_threshold, m_pregrasp_distance,
      m_approach_speed, m_extract_distance);

  if (auto store = tbl["store"]; store.is_table()) {
    m_store_approach_offset = store["approach_offset"].value<double>().value_or(
        m_store_approach_offset);
    m_store_pause_after_above_s =
        store["pause_after_above_s"].value<double>().value_or(
            m_store_pause_after_above_s);
    m_store_wait_before_open_s =
        store["wait_before_open_s"].value<double>().value_or(m_store_wait_before_open_s);
    m_store_wait_after_open_s =
        store["wait_after_open_s"].value<double>().value_or(
            m_store_wait_after_open_s);
    m_store_wait_after_close_s =
        store["wait_after_close_s"].value<double>().value_or(m_store_wait_after_close_s);
    m_store_extra_z_above_store_m =
        store["extra_z_above_store_m"].value<double>().value_or(
            m_store_extra_z_above_store_m);
    m_store_retreat_distance_m =
        store["retreat_distance_m"].value<double>().value_or(
            m_store_retreat_distance_m);
    m_store_retreat_offset_x_m =
        store["retreat_offset_x_m"].value<double>().value_or(m_store_retreat_offset_x_m);
    m_store_retreat_offset_y_m =
        store["retreat_offset_y_m"].value<double>().value_or(m_store_retreat_offset_y_m);
    m_retrieve_z_offset_m = store["retrieve_z_offset_m"].value<double>().value_or(m_retrieve_z_offset_m);
    m_retrieve_yaw_offset_rad = store["retrieve_yaw_offset_rad"].value<double>().value_or(m_retrieve_yaw_offset_rad);
  }
  m_logger->info("Store params: approach_offset={:.3f}, "
                 "pause_after_above_s={:.3f}, wait_before_open_s={:.3f}, wait_after_open_s={:.3f}, "
                 "wait_after_close_s={:.3f}, extra_z_above_store_m={:.3f}, retreat_distance_m={:.3f}, "
                 "retreat_offset_x_m={:.3f}, retreat_offset_y_m={:.3f}, "
                 "retrieve_z_offset_m={:.3f}, retrieve_yaw_offset_rad={:.3f}",
                 m_store_approach_offset, m_store_pause_after_above_s, m_store_wait_before_open_s, m_store_wait_after_open_s, m_store_wait_after_close_s,
                 m_store_extra_z_above_store_m, m_store_retreat_distance_m, m_store_retreat_offset_x_m, m_store_retreat_offset_y_m,
                 m_retrieve_z_offset_m, m_retrieve_yaw_offset_rad);
}

yandy::Robot::~Robot() { stop(); }

// ============================================================
// 生命周期
// ============================================================

void yandy::Robot::start() {
  m_logger->info("Starting Robot...");

  // 初始化视觉子系统
  // 如果 force_simulate_vision 启用，直接使用仿真视觉（即使真实相机初始化成功）
  if (m_force_simulate_vision) {
    m_sim_vision_enabled = true;
    m_vision_thread = std::thread([this] { simVisionLoop(); });
    m_logger->info("force_simulate_vision enabled, using simulated vision.");
  } else if (m_hik_driver.init() && m_detector.init()) {
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
  withSolver([&](auto &s) {
    s.updateKinematics(m_arm_state.q, m_arm_state.v, m_gimbal_state.q);
  });
  m_arm_cmd.q_des = m_arm_state.q;
  m_arm_cmd.v_des.setZero();
  // 初始时底盘静止 (ax=0,ay=0,az=+9.81 表示水平), 等价于 computeGravity()
  m_arm_cmd.tau_ff = withSolver([](auto &s) {
    return s.computeDynamics(Eigen::Vector3d(0.0, 0.0, 9.81),
                             Eigen::Vector3d::Zero(),
                             Eigen::Quaterniond::Identity());
  });
  m_arm_cmd.kp.fill(20.0);
  m_arm_cmd.kd.fill(1.0);

  // 初始化 Ruckig 的当前状态
  m_planner->syncState(m_arm_state.q, m_arm_state.v);
  m_planner->updateGimbalState(m_gimbal_state.q);

  // 读取当前 store 关节位置
  for (size_t i = 0; i < 2; ++i) {
    m_store_pose[i] = withSolver([i](auto &s) { return s.getStoreFrame(i); });
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
    withSolver([&](auto &s) {
      s.updateKinematics(m_arm_state.q, m_arm_state.v, m_gimbal_state.q);
    });
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
      m_logger->info("OMPL plan execution finished");
    }

    // 7. 获取当前状态 (用于连续行为派发)
    const auto cur_state = m_fsm.getState();

    // 7.5. 更新末端负载质量 (根据夹爪状态)
    updatePayloadMass();

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
      // 保持当前位置 + 动力学补偿 (含底盘惯性, 固定基座时退化为重力补偿)
      m_arm_cmd.q_des = m_arm_state.q;
      m_arm_cmd.v_des.setZero();
      {
        const Eigen::Vector3d chassis_acc(pack.ax, pack.ay, pack.az);
        const Eigen::Vector3d chassis_omega(pack.gx, pack.gy, pack.gz);
        const Eigen::Quaterniond chassis_quat(pack.qw, pack.qx, pack.qy,
                                              pack.qz);
        m_arm_cmd.tau_ff = withSolver([&](auto &s) {
          return s.computeDynamics(chassis_acc, chassis_omega, chassis_quat);
        });
      }
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
      vd.ee_pose = withSolver([](auto &s) { return s.getEndEffectorPose(); });
      vd.target_pose = m_target_pose;
      vd.state = cur_state;

      // 每帧尝试读取视觉数据并转换到基座系
      auto vis = m_vision_buf.try_read();
      if (vis.has_value() && vis->valid) {
        vd.vision_valid = true;
        vd.vision_unit_pose = vis->unit_pose;
        vd.vision_unit_pose_base =
            (m_is_simulate || m_force_simulate_vision)
                ? m_sim_cam_pose * vis->unit_pose
                : withSolver([&](auto &s) {
                    return s.transformObjectToBase(vis->unit_pose);
                  });
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

  m_logger->info("Stopping Robot...");

  // 1. 先停止规划线程 (避免继续规划)
  m_planner->stopPlanThread();

  // 2. 停止视觉线程
  if (m_vision_thread.joinable())
    m_vision_thread.join();

  // 3. 最后禁用硬件
  try {
    m_arm_hw.disable();
  } catch (const std::exception &e) {
    m_logger->warn("Exception during motor disable: {}", e.what());
  }

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

  // 每帧取最新底盘数据用于动力学补偿
  const auto &dyn_pack = m_input.getLatestCommand();
  const Eigen::Vector3d chassis_acc(dyn_pack.ax, dyn_pack.ay, dyn_pack.az);
  const Eigen::Vector3d chassis_omega(dyn_pack.gx, dyn_pack.gy, dyn_pack.gz);
  const Eigen::Quaterniond chassis_quat(dyn_pack.qw, dyn_pack.qx, dyn_pack.qy,
                                        dyn_pack.qz);
  m_arm_cmd.tau_ff = withSolver([&](auto &s) {
    return s.computeDynamics(chassis_acc, chassis_omega, chassis_quat);
  });

  // 如果 OMPL 正在等待或执行中，让 Ruckig 继续跟踪 waypoints，不重新规划
  if (m_ompl_pending || m_ompl_executing) {
    m_planner->update(m_arm_cmd.q_des, m_arm_cmd.v_des);
    m_arm_hw.write(m_arm_cmd);
    return false;
  }

  // IK 求解 (只针对机械臂 6 DoF)
  auto q_sol = withSolver(
      [&](auto &s) { return s.solveIK(target_pose, m_arm_state.q, 0.01); });
  if (!q_sol) {
    // IK 失败，Ruckig 继续执行当前轨迹 (或已停车)
    m_planner->update(m_arm_cmd.q_des, m_arm_cmd.v_des);
    m_arm_hw.write(m_arm_cmd);
    return false;
  }

  // 将解复制为可修改的向量，并将角度调整到与当前关节角最近的等价角，避免 ±2π
  // 跳变
  auto q_goal = q_sol.value();
  for (int i = 0; i < static_cast<int>(common::ARM_JOINT_NUM); ++i) {
    const double delta = m_arm_state.q[i] - q_goal[i];
    const double n = std::round(delta / (2.0 * M_PI));
    q_goal[i] += 2.0 * M_PI * n;
  }

  // 路径碰撞检测 (使用当前云台状态)
  // 如果在 OMPL 失败冷却期内，跳过碰撞检测直接尝试移动
  bool collision = withSolver(
      [&](auto &s) { return s.checkPathCollision(m_arm_state.q, q_goal); });
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
  // 保险：离开 Store 模式时（不论是正常完成还是手动取消）恢复 base_link
  // 碰撞检测
  withSolver([](auto &s) { s.setBaseLinkCollisionEnabled(true); });

  // 如果刚刚退出 Fetch
  // 模式，忽略一次来自手柄/遥控的自动位姿下发，避免回到视觉起始点
  if (m_ignore_next_manual) {
    m_ignore_next_manual = false;
    // 继续执行当前轨迹/动力学补偿，不改变目标
    m_planner->update(m_arm_cmd.q_des, m_arm_cmd.v_des);
    m_arm_hw.write(m_arm_cmd);
    return;
  }

  auto pack = m_input.getLatestCommand();

  // 如果需要限制返回过程中的 roll，使用当前末端的 roll 替换用户输入的 roll
  if (m_limit_return_roll) {
    Eigen::Matrix3d R_curr =
        withSolver([](auto &s) { return s.getEndEffectorPose().rotation(); });
    Eigen::Vector3d rpy_curr =
        R_curr.eulerAngles(2, 1, 0); // [yaw, pitch, roll]
    const double roll_keep = rpy_curr[2];
    pack.roll = static_cast<float>(roll_keep);
    // 仅在首次手动目标中应用限制
    m_limit_return_roll = false;
  }

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
  case FetchPhase::Extracting:
    handleExtracting();
    break;
  case FetchPhase::Withdrawing:
    handleWithdrawing();
    break;
  }
}

void yandy::Robot::handleSeeking() {
  // 如果已经锁定了目标位姿，直接使用锁定位姿，不再读取视觉数据
  if (m_locked_target_valid) {
    const Eigen::Vector3d pregrasp_pos =
        m_locked_target_pos - m_locked_approach_dir * m_pregrasp_distance;
    solveAndPlan5DoF(pregrasp_pos, m_locked_approach_dir);
    return;
  }

  auto vd = m_vision_buf.try_read();
  if (!vd.has_value() || !vd->valid) {
    // 无有效视觉数据，保持当前位置 + 动力学补偿
    const auto &seek_pack = m_input.getLatestCommand();
    m_arm_cmd.tau_ff = withSolver([&](auto &s) {
      return s.computeDynamics(
          Eigen::Vector3d(seek_pack.ax, seek_pack.ay, seek_pack.az),
          Eigen::Vector3d(seek_pack.gx, seek_pack.gy, seek_pack.gz),
          Eigen::Quaterniond(seek_pack.qw, seek_pack.qx, seek_pack.qy,
                             seek_pack.qz));
    });
    m_planner->update(m_arm_cmd.q_des, m_arm_cmd.v_des);
    m_arm_hw.write(m_arm_cmd);
    return;
  }

  // 将相机坐标系下的位姿转换到基座坐标系
  Eigen::Isometry3d target_pose;
  if (m_is_simulate || m_force_simulate_vision) {
    target_pose = m_sim_cam_pose * vd->unit_pose;
  } else {
    target_pose = withSolver(
        [&](auto &s) { return s.transformObjectToBase(vd->unit_pose); });
  }

  // 更新位姿历史
  m_pose_history.push_back(target_pose);
  if (m_pose_history.size() > m_stability_window) {
    m_pose_history.pop_front();
  }

  // 检查稳定性
  if (isPoseStable()) {
    // 锁定目标（一旦锁定，后续不再改变）
    m_locked_target_pos = target_pose.translation();

    // 视觉位姿的坐标系定义（PnP 解算结果）：
    //   - Z 轴：垂直于能量单元顶面（瓶口方向）
    //   - X/Y 轴：在顶面平面内
    //
    // 夹爪抓取策略：
    //   - 逼近方向 (approach_dir): 固定为 base_link 的 +X
    //   轴方向（从机器人往目标逼近）
    //   - 提取方向 (extract_dir): 瓶口反方向（-Z 轴），瓶口朝上就向下提
    m_locked_approach_dir =
        Eigen::Vector3d::UnitX(); // +X 轴 = 从机器人往目标逼近
    m_locked_extract_dir = -target_pose.rotation().col(2); // -Z 轴 = 瓶口反方向
    m_locked_target_valid = true;                          // 标记已锁定

    m_current_standoff = m_pregrasp_distance;
    m_pose_history.clear();

    m_logger->info("★★★ TARGET LOCKED ★★★");
    m_logger->info("Target position: [{:.3f}, {:.3f}, {:.3f}]",
                   m_locked_target_pos.x(), m_locked_target_pos.y(),
                   m_locked_target_pos.z());
    m_logger->info("Approach direction (+X): [{:.3f}, {:.3f}, {:.3f}]",
                   m_locked_approach_dir.x(), m_locked_approach_dir.y(),
                   m_locked_approach_dir.z());
    m_logger->info(
        "Extract direction (-Z, opposite to bottle): [{:.3f}, {:.3f}, {:.3f}]",
        m_locked_extract_dir.x(), m_locked_extract_dir.y(),
        m_locked_extract_dir.z());

    // 切换到 PreGrasp 阶段
    m_fetch_phase = FetchPhase::PreGrasp;
  } else {
    // 尚未稳定：移动到历史平均位置上方，使用竖直向上方向
    Eigen::Vector3d mean_pos = Eigen::Vector3d::Zero();
    for (const auto &p : m_pose_history) {
      mean_pos += p.translation();
    }
    mean_pos /= static_cast<double>(m_pose_history.size());

    const Eigen::Vector3d approach_dir = Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d pregrasp_pos =
        mean_pos - approach_dir * m_pregrasp_distance;

    solveAndPlan5DoF(pregrasp_pos, approach_dir);
  }
}

void yandy::Robot::handlePreGrasp() {
  // 移动到预抓取点（位姿已锁定，不再改变）
  const Eigen::Vector3d pregrasp_pos =
      m_locked_target_pos - m_locked_approach_dir * m_pregrasp_distance;

  if (!solveAndPlan5DoF(pregrasp_pos, m_locked_approach_dir)) {
    // IK 或规划失败，保持当前位置继续尝试
    return;
  }

  // 检查是否到达预抓取点
  const Eigen::Vector3d ee_pos =
      withSolver([](auto &s) { return s.getEndEffectorPose().translation(); });
  const double dist = (ee_pos - pregrasp_pos).norm();

  if (dist < 0.01 && m_planner->isFinished()) { // 10mm 阈值 + 轨迹完成
    m_logger->info("Reached pre-grasp point, distance: {:.3f}m", dist);
    m_fetch_phase = FetchPhase::Approaching;
  }
}

void yandy::Robot::handleApproaching() {
  // 直线逼近：逐步减小 standoff 距离
  m_current_standoff -= m_approach_speed * DT;

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

    // 设置 FSM 手持状态标志 (用于存矿流程判断)
    m_fsm.setMineralAttached(true);

    m_current_extract_offset = 0.0;
    m_fetch_phase = FetchPhase::Extracting;
    m_logger->info(
        "Grasp complete, starting extraction along [{:.3f},{:.3f},{:.3f}]...",
        m_locked_extract_dir.x(), m_locked_extract_dir.y(),
        m_locked_extract_dir.z());
  }
}

void yandy::Robot::handleExtracting() {
  // 沿提取方向逐步移动，将能量单元从容器中取出
  m_current_extract_offset += m_approach_speed * DT;
  if (m_current_extract_offset > m_extract_distance) {
    m_current_extract_offset = m_extract_distance;
  }

  const Eigen::Vector3d current_target =
      m_locked_target_pos + m_locked_extract_dir * m_current_extract_offset;

  if (!solveAndPlan5DoF(current_target, m_locked_approach_dir)) {
    return;
  }

  if (m_current_extract_offset >= m_extract_distance &&
      m_planner->isFinished()) {
    m_logger->info(
        "Extraction complete ({:.3f}m along [{:.3f},{:.3f},{:.3f}]), "
        "starting withdrawal...",
        m_extract_distance, m_locked_extract_dir.x(), m_locked_extract_dir.y(),
        m_locked_extract_dir.z());
    m_fetch_phase = FetchPhase::Withdrawing;
  }
}

void yandy::Robot::handleWithdrawing() {
  // 直接结束撤回阶段：不要回到预抓取点，立即退出 FetchingMode 并回到手动控制
  m_logger->info("Withdrawing: skipping return-to-pregrasp and exiting fetch "
                 "mode immediately");

  // 停止 planner 并清空内部状态
  m_planner->brake();
  resetFetchState();
  m_ompl_pending = false;
  m_ompl_executing = false;

  // 保持当前关节位置，避免自动下发移动
  m_arm_cmd.q_des = m_arm_state.q;
  m_arm_cmd.v_des.setZero();
  m_arm_hw.write(m_arm_cmd);

  // 忽略下一次 manual 目标下发，避免回到视觉起始点
  m_ignore_next_manual = true;
  // 限制返回 Manual 时末端 roll 变化（首次 Manual 有效）
  m_limit_return_roll = true;

  // 生成仿真下一个目标（如果启用）
  if (m_sim_vision_enabled && m_sim_vision_random) {
    generateRandomUnitPose();
  }

  // 退出 FetchingMode
  m_fsm.processCmd(YandyControlCmd::CMD_SWITCH_FETCH);
}

void yandy::Robot::resetFetchState() {
  m_fetch_phase = FetchPhase::Seeking;
  m_pose_history.clear();
  m_current_standoff = m_pregrasp_distance;
  m_current_extract_offset = 0.0;
  m_locked_target_pos.setZero();
  m_locked_approach_dir =
      Eigen::Vector3d::UnitX(); // +X 轴 = 从机器人往目标逼近
  m_locked_extract_dir = -Eigen::Vector3d::UnitZ(); // -Z 轴 = 默认向下
  m_locked_target_valid = false;                    // 清除锁定标志
}

void yandy::Robot::updatePayloadMass() {
  // 根据夹爪实际状态 (isHolding) 来决定是否附加负载
  const bool is_holding = m_effector.isHolding();
  
  // 只在状态变化时更新，避免重复调用
  if (is_holding && !m_payload_attached) {
    // 夹爪夹持物体，附加 600g 负载
    withSolver([&](auto &s) {
      s.setEndEffectorMass(PAYLOAD_MASS);
    });
    m_payload_attached = true;
    m_logger->info("Payload attached: +{:.1f}g (claw holding)", PAYLOAD_MASS * 1000);
  } else if (!is_holding && m_payload_attached) {
    // 夹爪张开，移除负载
    withSolver([&](auto &s) {
      s.setEndEffectorMass(0.0);
    });
    m_payload_attached = false;
    m_logger->info("Payload removed: 0g (claw open)");
  }
}

bool yandy::Robot::isPoseStable() const {
  if (m_pose_history.size() < m_stability_window) {
    return false;
  }

  // 计算位置均值
  Eigen::Vector3d mean_pos = Eigen::Vector3d::Zero();
  for (const auto &p : m_pose_history) {
    mean_pos += p.translation();
  }
  mean_pos /= static_cast<double>(m_pose_history.size());

  // 计算位置方差
  double pos_variance = 0.0;
  for (const auto &p : m_pose_history) {
    pos_variance += (p.translation() - mean_pos).squaredNorm();
  }
  pos_variance /= static_cast<double>(m_pose_history.size());

  // 计算方向稳定性：检查 Z 轴（逼近方向）的方差
  // 使用单位四元数的角度差来衡量方向变化
  double orient_variance = 0.0;
  Eigen::Quaterniond mean_q = Eigen::Quaterniond::Identity();

  // 简单方法：检查各帧 Z 轴方向的一致性
  Eigen::Vector3d mean_z = Eigen::Vector3d::Zero();
  for (const auto &p : m_pose_history) {
    mean_z += p.rotation().col(2); // Z 轴
  }
  mean_z.normalize();

  for (const auto &p : m_pose_history) {
    const Eigen::Vector3d z_axis = p.rotation().col(2).normalized();
    // 计算与平均方向的夹角余弦值
    const double cos_theta = mean_z.dot(z_axis);
    // 角度差（弧度），cos=1 表示同向，cos=0 表示垂直
    orient_variance += std::acos(std::max(-1.0, std::min(1.0, cos_theta)));
  }
  orient_variance /= static_cast<double>(m_pose_history.size());

  // 位置方差阈值 + 方向角度阈值（10 度 ≈ 0.174 rad）
  constexpr double orient_threshold = 0.174;

  return (std::sqrt(pos_variance) < m_stability_threshold) &&
         (orient_variance < orient_threshold);
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
                                    const Eigen::Vector3d &approach_dir,
                                    bool use5DoF) {
  // 从 approach_dir 构造完整 6DoF 目标位姿：
  // 将世界 Z 轴 [0,0,1] 旋转到 approach_dir 的最短旋转，
  // 但对工具绕 Z 轴的 roll 采用当前末端的 roll 值以避免翻转
  const Eigen::Vector3d z_des = approach_dir.normalized();

  // 获取当前末端的 roll（使用 Z-Y-X Euler: [yaw, pitch, roll]）
  Eigen::Matrix3d R_curr =
      withSolver([](auto &s) { return s.getEndEffectorPose().rotation(); });
  Eigen::Vector3d rpy_curr = R_curr.eulerAngles(2, 1, 0); // [yaw, pitch, roll]
  const double roll_keep = rpy_curr[2];

  // 构建一个以 z_des 为 Z 轴、并以 roll_keep 绕 Z 轴旋转的坐标系
  Eigen::Vector3d ref = Eigen::Vector3d::UnitX();
  if (std::abs(ref.dot(z_des)) > 0.99)
    ref = Eigen::Vector3d::UnitY();
  Eigen::Vector3d base_x = (ref - z_des * (ref.dot(z_des))).normalized();
  Eigen::Vector3d base_y = z_des.cross(base_x).normalized();

  const double c = std::cos(roll_keep);
  const double s = std::sin(roll_keep);
  Eigen::Vector3d x_rot = c * base_x + s * base_y; // X 轴经 roll 旋转后方向
  Eigen::Vector3d y_rot = z_des.cross(x_rot).normalized();

  Eigen::Matrix3d R_des;
  R_des.col(0) = x_rot;
  R_des.col(1) = y_rot;
  R_des.col(2) = z_des;

  m_target_pose = Eigen::Isometry3d::Identity();
  m_target_pose.translate(target_pos);
  m_target_pose.rotate(R_des);

  // 动力学补偿 (含底盘惯性)
  {
    const auto &p5_pack = m_input.getLatestCommand();
    m_arm_cmd.tau_ff = withSolver([&](auto &s) {
      return s.computeDynamics(
          Eigen::Vector3d(p5_pack.ax, p5_pack.ay, p5_pack.az),
          Eigen::Vector3d(p5_pack.gx, p5_pack.gy, p5_pack.gz),
          Eigen::Quaterniond(p5_pack.qw, p5_pack.qx, p5_pack.qy, p5_pack.qz));
    });
  }

  // 如果 OMPL 正在等待或执行中，让 Ruckig 继续跟踪 waypoints，不重新规划
  if (m_ompl_pending || m_ompl_executing) {
    m_planner->update(m_arm_cmd.q_des, m_arm_cmd.v_des);
    m_arm_hw.write(m_arm_cmd);
    return false;
  }

  // 优先尝试 6DoF IK；若失败或显式要求 5DoF，则尝试 5DoF IK（约束位置 +
  // 方向，绕轴自由旋转）
  auto q_sol = withSolver(
      [&](auto &s) { return s.solveIK(m_target_pose, m_arm_state.q, 0.01); });
  if (!q_sol || use5DoF) {
    // 尝试使用 5DoF IK（位置 + 方向约束，绕方向轴自由旋转）
    auto q5_sol = withSolver([&](auto &s) {
      return s.solveIK5DoF(target_pos, approach_dir, m_arm_state.q, 0.01);
    });
    if (q5_sol) {
      q_sol = q5_sol;
    }
  }

  if (!q_sol) {
    static int ik_fail_count = 0;
    if (++ik_fail_count % 100 == 1) {
      m_logger->warn(
          "IK (6DoF/5DoF) failed for target [{:.3f}, {:.3f}, {:.3f}], "
          "approach [{:.3f}, {:.3f}, {:.3f}]",
          target_pos.x(), target_pos.y(), target_pos.z(), approach_dir.x(),
          approach_dir.y(), approach_dir.z());
    }
    m_planner->update(m_arm_cmd.q_des, m_arm_cmd.v_des);
    m_arm_hw.write(m_arm_cmd);
    return false;
  }

  // 将解复制为可修改的向量，并将角度调整到与当前关节角最近的等价角，避免 ±2π
  // 跳变
  auto q_goal = q_sol.value();
  for (int i = 0; i < static_cast<int>(common::ARM_JOINT_NUM); ++i) {
    const double delta = m_arm_state.q[i] - q_goal[i];
    const double n = std::round(delta / (2.0 * M_PI));
    q_goal[i] += 2.0 * M_PI * n;
  }

  // 路径碰撞检测
  // 如果在 OMPL 失败冷却期内，跳过碰撞检测直接尝试移动
  if (m_ompl_fail_cooldown == 0 && withSolver([&](auto &s) {
        return s.checkPathCollision(m_arm_state.q, q_goal);
      })) {
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
  // 存取矿期间忽略与 base_link（底座柱子）的碰撞
  withSolver([](auto &s) { s.setBaseLinkCollisionEnabled(false); });

  const Eigen::Isometry3d &sp = m_store_pose[m_store_pose_index];

  // store_frame 正上方（世界 Z 轴）的预接近位姿
  Eigen::Isometry3d above_pose = sp;
  above_pose.translation() +=
      Eigen::Vector3d::UnitZ() * m_store_approach_offset;

  const bool is_deposit = m_fsm.hasMineralAttached();

  // 诊断日志：打印 store_frame 位姿和实际末端位置
  m_logger->info("Store frame[{}]: pos=[{:.3f}, {:.3f}, {:.3f}]",
                 m_store_pose_index, sp.translation().x(), sp.translation().y(),
                 sp.translation().z());

  if (is_deposit) {
    // 存矿: Approaching -> PauseAbove -> AtTarget -> OpenWait -> PostOpenLift
    // -> Retreating -> Exit
    switch (m_store_phase) {
    case StorePhase::Approaching: {
      m_logger->info(
          "Store (deposit): moving to above_pose [{:.3f}, {:.3f}, {:.3f}]",
          above_pose.translation().x(), above_pose.translation().y(),
          above_pose.translation().z());
      if (!solveAndPlan(above_pose))
        return;
      const Eigen::Vector3d ee = withSolver(
          [](auto &s) { return s.getEndEffectorPose().translation(); });
      m_logger->info("Store (deposit): actual EE pos [{:.3f}, {:.3f}, {:.3f}], "
                     "error [{:.3f}, {:.3f}, {:.3f}]",
                     ee.x(), ee.y(), ee.z(),
                     ee.x() - above_pose.translation().x(),
                     ee.y() - above_pose.translation().y(),
                     ee.z() - above_pose.translation().z());
      if ((ee - above_pose.translation()).norm() < 0.015 &&
          m_planner->isFinished()) {
        m_logger->info(
            "Store: reached above-store, starting dwell before lowering");
        m_store_phase = StorePhase::PauseAbove;
        m_store_phase_start = std::chrono::steady_clock::now();
      }
      break;
    }
    case StorePhase::PauseAbove: {
      // 保持 above_pose，等待一段时间
      if (!solveAndPlan(above_pose))
        return;
      const auto elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        m_store_phase_start)
              .count();
      if (elapsed >= m_store_pause_after_above_s) {
        m_logger->info(
            "Store: pause above complete ({:.3f}s), lowering to store frame",
            elapsed);
        m_store_phase = StorePhase::AtTarget;
      }
      break;
    }
    case StorePhase::AtTarget: {
      m_logger->info(
          "Store (deposit): moving to store_frame [{:.3f}, {:.3f}, {:.3f}]",
          sp.translation().x(), sp.translation().y(), sp.translation().z());
      if (!solveAndPlan(sp))
        return;
      const Eigen::Vector3d ee = withSolver(
          [](auto &s) { return s.getEndEffectorPose().translation(); });
      m_logger->info("Store (deposit): actual EE pos [{:.3f}, {:.3f}, {:.3f}], "
                     "error [{:.3f}, {:.3f}, {:.3f}]",
                     ee.x(), ee.y(), ee.z(), ee.x() - sp.translation().x(),
                     ee.y() - sp.translation().y(),
                     ee.z() - sp.translation().z());
      if ((ee - sp.translation()).norm() < 0.015 && m_planner->isFinished()) {
        m_logger->info("Store: reached store position, waiting before open");
        m_store_phase = StorePhase::OpenWait;
        m_store_phase_start = std::chrono::steady_clock::now();
      }
      break;
    }
    case StorePhase::OpenWait: {
      // 等待一段时间（非阻塞）再打开夹爪
      if (!solveAndPlan(sp)) return;
      const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - m_store_phase_start).count();
      if (elapsed >= m_store_wait_before_open_s) {
        m_logger->info("Store: pre-open wait complete ({:.3f}s), opening claw", elapsed);
        m_effector.openClaw();
        // 进入开爪后的等待阶段
        m_store_phase = StorePhase::PostOpenLift; // repurposed as PostOpenWait
        m_store_phase_start = std::chrono::steady_clock::now();
      }
      break;
    }
    case StorePhase::PostOpenLift: {
      // 开爪后等待一段时间（非阻塞），然后退回
      if (!solveAndPlan(sp)) return;
      const auto elapsed_post = std::chrono::duration<double>(std::chrono::steady_clock::now() - m_store_phase_start).count();
      if (elapsed_post >= m_store_wait_after_open_s) {
        m_logger->info("Store: post-open wait complete ({:.3f}s), starting retreat", elapsed_post);
        m_store_phase = StorePhase::Retreating;
        m_store_phase_start = std::chrono::steady_clock::now();
      }
      break;
    }
    case StorePhase::Retreating: {
      // 计算相对于 store_frame 的 XY 偏移目标（使用 store_frame.z + 可选 retrieve_z_offset）
      Eigen::Isometry3d base_target = sp;
      base_target.translation().z() += m_retrieve_z_offset_m; // 用户指定相对 base_link 的 z 偏移

      // Interpret retreat offsets in base_link frame: add along base_link X/Y
      Eigen::Vector3d target_pos = base_target.translation()
          + Eigen::Vector3d::UnitX() * m_store_retreat_offset_x_m
          + Eigen::Vector3d::UnitY() * m_store_retreat_offset_y_m;

      // 自动根据 store_frame 的 y 值决定镜像：
      // sign = +1 if y>=0, -1 if y<0. 这样左右对称的 frame 会得到相反的 Y 偏移与 yaw
      const double sign_y = (sp.translation().y() >= 0.0) ? 1.0 : -1.0;
      const Eigen::Vector3d applied_offsets = Eigen::Vector3d::UnitX() * m_store_retreat_offset_x_m
          + Eigen::Vector3d::UnitY() * (sign_y * m_store_retreat_offset_y_m);
      const double applied_yaw = sign_y * m_retrieve_yaw_offset_rad;
      target_pos = base_target.translation() + applied_offsets;

      m_logger->info("Store: retreating/repositioning to target [x={:.3f}, y={:.3f}, z={:.3f}] (base_link offsets x={:.3f}, y={:.3f}, z_lift={:.3f}, applied_yaw={:.3f})",
                     target_pos.x(), target_pos.y(), target_pos.z(), m_store_retreat_offset_x_m, sign_y * m_store_retreat_offset_y_m, m_store_extra_z_above_store_m, applied_yaw);

      // 构造朝向：在目标上应用额外的绕末端局部 X 轴的偏航（用户语义：相对于 base_link 为 yaw）
      // (apply rotation around the end-effector's local X axis)
      Eigen::Isometry3d ee_pose = withSolver([](auto &s) { return s.getEndEffectorPose(); });
      const double yaw = applied_yaw;
      // Rotation about local X (end-effector frame). Compose as R_des = R_ee * R_x(yaw_local)
      Eigen::AngleAxisd x_local(yaw, Eigen::Vector3d::UnitX());
      Eigen::Matrix3d R_des = ee_pose.rotation() * x_local.toRotationMatrix();

      m_target_pose = Eigen::Isometry3d::Identity();
      m_target_pose.translate(target_pos);
      m_target_pose.rotate(R_des);

      // 使用 6DoF IK 优先尝试，若失败再尝试 5DoF
      if (!solveAndPlan(m_target_pose)) {
        // 作为回退，尝试 5DoF 约束朝向为 -store_frame.Z
        const Eigen::Vector3d approach_dir = -sp.rotation().col(2).normalized();
        if (!solveAndPlan5DoF(target_pos, approach_dir, true))
          return;
      }

      const Eigen::Vector3d cur = withSolver([](auto &s) { return s.getEndEffectorPose().translation(); });
      if ((cur - target_pos).norm() < 0.03 && m_planner->isFinished()) {
        m_logger->info("Store: reposition complete, exiting store mode");
        withSolver([](auto &s) { s.setBaseLinkCollisionEnabled(true); });
        m_fsm.processCmd(YandyControlCmd::CMD_SWITCH_STORE);
      }
      break;
    }
    default:
      break;
    }
  } else {
    // 取矿: AtTarget（store位置）→ 闭爪 → Retracting（上方）→ 退出
    switch (m_store_phase) {
    case StorePhase::PreFetch: {
      // 计算相对于 store_frame 的 XY 偏移目标（使用 store_frame.z + 可选 retrieve_z_offset）
      Eigen::Isometry3d base_target = sp;
      base_target.translation().z() += m_retrieve_z_offset_m; // 用户指定相对 base_link 的 z 偏移

      // Interpret retreat offsets in base_link frame: add along base_link X/Y
      Eigen::Vector3d target_pos = base_target.translation()
          + Eigen::Vector3d::UnitX() * m_store_retreat_offset_x_m
          + Eigen::Vector3d::UnitY() * m_store_retreat_offset_y_m;

      // 自动根据 store_frame 的 y 值决定镜像：
      // sign = +1 if y>=0, -1 if y<0. 这样左右对称的 frame 会得到相反的 Y 偏移与 yaw
      const double sign_y = (sp.translation().y() >= 0.0) ? 1.0 : -1.0;
      const Eigen::Vector3d applied_offsets = Eigen::Vector3d::UnitX() * m_store_retreat_offset_x_m
          + Eigen::Vector3d::UnitY() * (sign_y * m_store_retreat_offset_y_m);
      const double applied_yaw = sign_y * m_retrieve_yaw_offset_rad;
      target_pos = base_target.translation() + applied_offsets;

      m_logger->info("Store (retrieve PreFetch): moving to pre-fetch target [x={:.3f}, y={:.3f}, z={:.3f}] (base_link offsets x={:.3f}, y={:.3f}, z_lift={:.3f}, applied_yaw={:.3f})",
                     target_pos.x(), target_pos.y(), target_pos.z(), m_store_retreat_offset_x_m, sign_y * m_store_retreat_offset_y_m, m_retrieve_z_offset_m, applied_yaw);

      // 构造朝向：使用 store_frame 的旋转作为参考，再应用额外的绕局部 X 轴的偏航
      // (apply rotation around the store_frame's local X axis)
      const double yaw = applied_yaw;
      Eigen::AngleAxisd x_local(yaw, Eigen::Vector3d::UnitX());
      Eigen::Matrix3d R_des = sp.rotation() * x_local.toRotationMatrix();

      m_target_pose = Eigen::Isometry3d::Identity();
      m_target_pose.translate(target_pos);
      m_target_pose.rotate(R_des);

      // 使用 6DoF IK 优先尝试，若失败再尝试 5DoF
      if (!solveAndPlan(m_target_pose)) {
        // 作为回退，尝试 5DoF 约束朝向为 -store_frame.Z
        const Eigen::Vector3d approach_dir = -sp.rotation().col(2).normalized();
        if (!solveAndPlan5DoF(target_pos, approach_dir, true))
          return;
      }

      const Eigen::Vector3d cur = withSolver([](auto &s) { return s.getEndEffectorPose().translation(); });
      if ((cur - target_pos).norm() < 0.03 && m_planner->isFinished()) {
        m_logger->info("Store (retrieve PreFetch): pre-fetch position reached, starting dwell before lowering");
        m_store_phase = StorePhase::PausePreFetch;
        m_store_phase_start = std::chrono::steady_clock::now();
      }
      break;
    }
    case StorePhase::PausePreFetch: {
      // 保持 PreFetch 位姿，等待一段时间
      if (!solveAndPlan(m_target_pose))
        return;
      const auto elapsed =
          std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                        m_store_phase_start)
              .count();
      if (elapsed >= m_store_pause_pre_fetch_s) {
        m_logger->info(
            "Store (retrieve PreFetch): dwell complete ({:.3f}s), moving to store frame",
            elapsed);
        m_store_phase = StorePhase::AtTarget;
      }
      break;
    }
    case StorePhase::AtTarget: {
      m_logger->info(
          "Store (retrieve): moving to store_frame [{:.3f}, {:.3f}, {:.3f}]",
          sp.translation().x(), sp.translation().y(), sp.translation().z());
      if (!solveAndPlan(sp))
        return;
      const Eigen::Vector3d ee = withSolver(
          [](auto &s) { return s.getEndEffectorPose().translation(); });
      m_logger->info("Store (retrieve): actual EE pos [{:.3f}, {:.3f}, "
                     "{:.3f}], error [{:.3f}, {:.3f}, {:.3f}]",
                     ee.x(), ee.y(), ee.z(), ee.x() - sp.translation().x(),
                     ee.y() - sp.translation().y(),
                     ee.z() - sp.translation().z());
      if ((ee - sp.translation()).norm() < 0.015 && m_planner->isFinished()) {
        m_logger->info("Store: reached store position, closing claw");
        m_effector.closeClaw();
        m_store_phase = StorePhase::Retracting;
        m_store_phase_start = std::chrono::steady_clock::now();
      }
      break;
    }
    case StorePhase::Retracting: {
      // 非阻塞等待闭爪完成（以 m_store_wait_after_close_s 为准）
      const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - m_store_phase_start).count();
      if (elapsed < m_store_wait_after_close_s) {
        // 保持当前以上位姿
        if (!solveAndPlan(above_pose)) return;
        return;
      }

      m_logger->info(
          "Store (retrieve): moving to above_pose [{:.3f}, {:.3f}, {:.3f}]",
          above_pose.translation().x(), above_pose.translation().y(),
          above_pose.translation().z());
      if (!solveAndPlan(above_pose))
        return;
      const Eigen::Vector3d ee = withSolver(
          [](auto &s) { return s.getEndEffectorPose().translation(); });
      m_logger->info("Store (retrieve): actual EE pos [{:.3f}, {:.3f}, {:.3f}], error [{:.3f}, {:.3f}, {:.3f}]",
                     ee.x(), ee.y(), ee.z(),
                     ee.x() - above_pose.translation().x(),
                     ee.y() - above_pose.translation().y(),
                     ee.z() - above_pose.translation().z());
      if ((ee - above_pose.translation()).norm() < 0.015 &&
          m_planner->isFinished()) {
        m_logger->info("Store: lifted store item, exiting store mode");
        withSolver([](auto &s) { s.setBaseLinkCollisionEnabled(true); });
        m_fsm.processCmd(YandyControlCmd::CMD_SWITCH_STORE);
      }
      break;
    }
    default:
      break;
    }
  }
}

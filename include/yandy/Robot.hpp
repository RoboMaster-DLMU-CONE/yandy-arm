#ifndef YANDY_ARM_ROBOT_HPP
#define YANDY_ARM_ROBOT_HPP

#define YANDY_ROBOT_CONFIG YANDY_CONFIG_PATH "robot.toml"

#include <deque>
#include <memory>
#include <random>
#include <thread>

#include <yandy/common/NBuf.hpp>
#include <yandy/modules/ArmHW.hpp>
#include <yandy/modules/DynamicsSolver.hpp>
#include <yandy/modules/Effector.hpp>
#include <yandy/modules/FSM.hpp>
#include <yandy/modules/InputProvider.hpp>
#include <yandy/modules/TrajectoryPlanner.hpp>
#include <yandy/modules/VisionSystem.hpp>

namespace one::can {
class CanDriver;
}

namespace yandy {
namespace detail {
struct VisionData {
  bool valid{false};
  Eigen::Isometry3d unit_pose{
      Eigen::Isometry3d::Identity()}; // 相机坐标系下，取置信度最高
};

struct RobotVizData {
  common::VectorJ q{common::VectorJ::Zero()}; // 完整 9D 关节状态 (用于可视化)
  Eigen::Isometry3d ee_pose{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d target_pose{Eigen::Isometry3d::Identity()};
  YandyState state{YandyState::Disabled};
  bool vision_valid{false};
  Eigen::Isometry3d vision_unit_pose{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d vision_unit_pose_base{Eigen::Isometry3d::Identity()};
};
} // namespace detail

class Robot {
public:
  explicit Robot(one::can::CanDriver &can);
  ~Robot();
  void start();
  void stop();

  const NBuf<detail::RobotVizData, 3> &vizBuf() const { return m_viz_buf; }

private:
  // ---- 控制循环常量 ----
  static constexpr double DT = 0.004; // 250Hz

  // 存储模式预设位姿 (基座坐标系)
  Eigen::Isometry3d m_store_pose[2];

  // ---- Modules ----
  modules::ArmHW m_arm_hw;
  modules::DynamicsSolver m_solver;
  std::unique_ptr<modules::TrajectoryPlanner>
      m_planner; // 延迟构造，依赖 m_solver
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
  common::ArmState m_arm_state{};       // 机械臂状态 (6 DoF)
  common::ArmCommand m_arm_cmd{};       // 机械臂指令 (6 DoF)
  common::GimbalState m_gimbal_state{}; // 云台状态 (3 DoF, 来自下位机)
  Eigen::Isometry3d m_target_pose{Eigen::Isometry3d::Identity()};
  bool m_is_simulate = false;
  Eigen::Isometry3d m_sim_cam_pose{
      Eigen::Isometry3d::Identity()}; // 仿真模式下手持相机的固定位姿 (base_link
                                      // 系)

  // ---- 仿真视觉配置 ----
  bool m_sim_vision_enabled{false};     // 仿真视觉是否启用
  bool m_sim_vision_random{false};      // 是否使用随机模式
  bool m_force_simulate_vision{false};  // 强制使用仿真视觉（即使 simulate=false）
  Eigen::Isometry3d m_sim_unit_pose{
      Eigen::Isometry3d::Identity()}; // 当前虚拟能量单元位姿
  std::array<double, 2> m_sim_x_range{};
  std::array<double, 2> m_sim_y_range{};
  std::array<double, 2> m_sim_z_range{};
  std::array<double, 2> m_sim_roll_range{};
  std::array<double, 2> m_sim_pitch_range{};
  void generateRandomUnitPose(); // 生成随机能量单元位姿
  void simVisionLoop();          // 仿真视觉循环

  bool m_ompl_pending{false};   // OMPL 规划是否正在进行中
  bool m_ompl_executing{false}; // OMPL 规划正在被 Ruckig 执行中
  int m_ompl_fail_cooldown{0};  // OMPL 失败后的冷却帧数
  int m_store_pose_index{0};    // 当前存取矿位姿索引 (由 FSM 回调设置)

  // ---- 存取矿流程状态 ----
  enum class StorePhase { Approaching, AtTarget, Retracting };
  StorePhase m_store_phase{StorePhase::Approaching};
  double m_store_approach_offset{0.3}; // store_frame Z 轴上偏移 (m)

  // ---- 抓取流程状态 ----
  enum class FetchPhase { Seeking, PreGrasp, Approaching, Extracting, Withdrawing };
  FetchPhase m_fetch_phase{FetchPhase::Seeking};

  int m_stability_window{10};          // 稳定性检测窗口 (帧数)
  double m_stability_threshold{0.005}; // 位置方差阈值 (m)
  double m_pregrasp_distance{0.05};    // 预抓取距离 (m)
  double m_approach_speed{0.05};       // 逼近速度 (m/s)
  double m_extract_distance{0.05};     // 提取距离 (m)

  std::deque<Eigen::Isometry3d> m_pose_history; // 视觉测量历史 (用于稳定性判断)
  Eigen::Vector3d m_locked_target_pos{
      Eigen::Vector3d::Zero()}; // 锁定的目标位置
  Eigen::Vector3d m_locked_approach_dir{
      Eigen::Vector3d::UnitZ()};                // 锁定的逼近方向
  Eigen::Vector3d m_locked_extract_dir{
      -Eigen::Vector3d::UnitX()};               // 锁定的提取方向 (= 锁定位姿 -R.col(0) = 瓶口反方向)
  double m_current_standoff{0.05}; // 当前距目标距离 (初始值同 pregrasp_distance)
  double m_current_extract_offset{0.0}; // 当前提取偏移量 (m)

  // ---- 私有方法 ----
  void visionLoop();
  void handleManual();
  void handleFetching();
  void handleStore();

  // 抓取子状态处理
  void handleSeeking();
  void handlePreGrasp();
  void handleApproaching();
  void handleExtracting();
  void handleWithdrawing();
  void resetFetchState(); // 重置抓取状态 (进入/退出 FetchingMode 时调用)

  // 辅助函数
  bool isPoseStable() const;
  Eigen::Vector3d computeApproachDirection(double roll, double pitch) const;
  bool solveAndPlan5DoF(const Eigen::Vector3d &target_pos,
                        const Eigen::Vector3d &approach_dir);

  // 从 YandyControlPack 提取云台状态
  void updateGimbalFromPack(const YandyControlPack &pack);

  // IK 求解 + 碰撞检测 + 轨迹规划调度
  // 返回 true 表示目标已成功设置给 planner
  bool solveAndPlan(const Eigen::Isometry3d &target_pose);
};
} // namespace yandy

#endif // YANDY_ARM_ROBOT_HPP

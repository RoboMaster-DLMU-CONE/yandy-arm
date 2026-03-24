#include <algorithm>
#include <cmath>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <random>
#include <set>
#include <yandy/modules/DynamicsSolver.hpp>

#include "yandy/core/Logger.hpp"

#define YANDY_URDF_PATH YANDY_CONFIG_PATH "urdf/yandy_urdf.urdf"

namespace yandy::modules {
DynamicsSolver::DynamicsSolver()
    : m_geom_data(m_geom_model) // Initialize with empty model, update later
{
  m_logger = core::create_logger("Solver", spdlog::level::info);
  m_logger->info("Initializing solver, loading urdf from {}", YANDY_URDF_PATH);
  try {
    pinocchio::urdf::buildModel(YANDY_URDF_PATH, m_model);

    // Build geometry model for collision
    std::vector<std::string> package_dirs;
    // Assuming YANDY_CONFIG_PATH is a string literal ending with /
    std::string config_path = YANDY_CONFIG_PATH;
    package_dirs.push_back(config_path + "urdf");

    pinocchio::urdf::buildGeom(m_model, YANDY_URDF_PATH, pinocchio::COLLISION,
                               m_geom_model, package_dirs);

    m_geom_model.addAllCollisionPairs();

    // 获取关节间的距离（跳数）
    auto jointDistance = [this](pinocchio::JointIndex j1,
                                pinocchio::JointIndex j2) -> int {
      if (j1 == j2)
        return 0;
      // 找到两个关节的公共祖先
      std::set<pinocchio::JointIndex> ancestors1;
      pinocchio::JointIndex curr = j1;
      while (curr != 0) {
        ancestors1.insert(curr);
        curr = m_model.parents[curr];
      }
      ancestors1.insert(0);

      int dist2 = 0;
      curr = j2;
      while (ancestors1.find(curr) == ancestors1.end()) {
        ++dist2;
        curr = m_model.parents[curr];
      }

      int dist1 = 0;
      pinocchio::JointIndex temp = j1;
      while (temp != curr) {
        ++dist1;
        temp = m_model.parents[temp];
      }

      return dist1 + dist2;
    };

    // Remove collision pairs between close links (distance <= 2)
    for (size_t i = 0; i < m_geom_model.collisionPairs.size();) {
      const auto &cp = m_geom_model.collisionPairs[i];
      const auto &obj1 = m_geom_model.geometryObjects[cp.first];
      const auto &obj2 = m_geom_model.geometryObjects[cp.second];

      const auto joint1 = obj1.parentJoint;
      const auto joint2 = obj2.parentJoint;

      // 移除距离 <= 2 的关节对（自身、父子、祖孙）
      if (jointDistance(joint1, joint2) <= 2) {
        m_geom_model.removeCollisionPair(cp);
        // Do not increment i, as removeCollisionPair shifts the vector
      } else {
        i++;
      }
    }

    // Re-initialize geometry data with the populated model
    m_geom_data = pinocchio::GeometryData(m_geom_model);

    // 缓存涉及 base_link 的碰撞对索引
    for (size_t i = 0; i < m_geom_model.collisionPairs.size(); ++i) {
      const auto &cp = m_geom_model.collisionPairs[i];
      const auto &name1 = m_geom_model.geometryObjects[cp.first].name;
      const auto &name2 = m_geom_model.geometryObjects[cp.second].name;
      if (name1.find("base_link") != std::string::npos ||
          name2.find("base_link") != std::string::npos) {
        m_base_link_pair_indices.push_back(i);
      }
    }

    m_logger->info("Geometry model loaded. Collision pairs: {}",
                   m_geom_model.collisionPairs.size());
  } catch (const std::exception &e) {
    m_logger->error("Error loading URDF: {}", e.what());
    throw;
  }
  m_data = pinocchio::Data(m_model);
  if (m_model.existJointName("joint_6")) {
    m_ee_joint_id = m_model.getJointId("joint_6");
  } else if (m_model.existJointName("joint_5")) {
    m_logger->warn("joint_6 not found, falling back to joint_5");
    m_ee_joint_id = m_model.getJointId("joint_5");
  } else {
    m_logger->error("No valid end-effector joint found, aborting...");
    throw std::runtime_error("No valid end-effector joint found");
  }
  if (m_model.existFrame("gripper_tcp")) {
    m_tcp_frame_id = m_model.getFrameId("gripper_tcp");
  } else {
    m_logger->error("No valid end-effector frame found.");
    throw std::runtime_error("No valid end-effector frame found");
  }
  if (m_model.existFrame("camera_optical_frame")) {
    m_camera_frame_id = m_model.getFrameId("camera_optical_frame");
  } else {
    m_logger->error("No valid camera-optical frame found.");
    throw std::runtime_error("No valid camera-optical frame found");
  }
  // loading store frames
  if (m_model.existFrame("store_frame_1") &&
      m_model.existFrame("store_frame_2")) {
    m_store_frames[0] = m_model.getFrameId("store_frame_1");
    m_store_frames[1] = m_model.getFrameId("store_frame_2");
  } else {
    m_logger->error("No valid store frames found.");
    throw std::runtime_error("No valid store frames found");
  }

  f_ext_.resize(m_model.njoints, pinocchio::Force::Zero());

  m_logger->info("Model loaded. Joints: {}, DoF: {} (Arm: {}, Gimbal: {})",
                 m_model.njoints, m_model.nv, common::ARM_JOINT_NUM,
                 common::GIMBAL_JOINT_NUM);
}

void DynamicsSolver::updateKinematics(const common::VectorArm &arm_q,
                                      const common::VectorArm &arm_v,
                                      const common::VectorGimbal &gimbal_q) {
  // 存储分离的状态
  m_current_arm_q = arm_q;
  m_current_arm_v = arm_v;
  m_current_gimbal_q = gimbal_q;

  // 组装完整的 9D 向量给 Pinocchio
  const common::VectorJ full_q = common::combineJoints(arm_q, gimbal_q);
  const common::VectorJ full_v =
      common::combineJoints(arm_v, common::VectorGimbal::Zero());

  pinocchio::forwardKinematics(m_model, m_data, full_q, full_v);
  pinocchio::updateFramePlacements(m_model, m_data);
}

common::VectorArm
DynamicsSolver::computeRNEA(const common::VectorArm &acc_des,
                            const common::Vector6 &ext_wrench_world) {
  std::ranges::fill(f_ext_, pinocchio::Force::Zero());
  if (!ext_wrench_world.isZero()) {
    // Pinocchio 的 f_ext 要求定义在"关节局部坐标系"下
    const auto &iso_world_to_local = m_data.oMi[m_ee_joint_id].inverse();
    const pinocchio::Force f_world(ext_wrench_world.head<3>(),
                                   ext_wrench_world.tail<3>());
    const pinocchio::Force f_local = iso_world_to_local.act(f_world);
    f_ext_[m_ee_joint_id] = f_local;
  }

  // 组装完整的 9D 向量
  const common::VectorJ full_q =
      common::combineJoints(m_current_arm_q, m_current_gimbal_q);
  const common::VectorJ full_v =
      common::combineJoints(m_current_arm_v, common::VectorGimbal::Zero());
  const common::VectorJ full_acc =
      common::combineJoints(acc_des, common::VectorGimbal::Zero());

  // 运行 RNEA，返回完整 9D 力矩
  const common::VectorJ full_tau =
      pinocchio::rnea(m_model, m_data, full_q, full_v, full_acc, f_ext_);

  // 只返回机械臂部分 (前 6 个元素)
  return common::extractArm(full_tau);
}

common::VectorArm DynamicsSolver::computeGravity() {
  const common::VectorJ full_q =
      common::combineJoints(m_current_arm_q, m_current_gimbal_q);
  const common::VectorJ zero_vel = common::VectorJ::Zero();
  const common::VectorJ full_tau =
      pinocchio::rnea(m_model, m_data, full_q, zero_vel, zero_vel);
  return common::extractArm(full_tau);
}

Eigen::Isometry3d DynamicsSolver::getEndEffectorPose() const {
  pinocchio::SE3 se3 = m_data.oMf[m_tcp_frame_id];

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = se3.translation();
  pose.linear() = se3.rotation();

  return pose;
}

Eigen::Isometry3d DynamicsSolver::getCameraPose() const {
  return Eigen::Isometry3d(m_data.oMf[m_camera_frame_id].toHomogeneousMatrix());
}

Eigen::Isometry3d DynamicsSolver::getStoreFrame(const size_t index) const {
  return Eigen::Isometry3d(
      m_data.oMf[m_store_frames[index]].toHomogeneousMatrix());
}

Eigen::Isometry3d DynamicsSolver::transformObjectToBase(
    const Eigen::Isometry3d &T_cam_obj) const {
  const Eigen::Isometry3d T_base_cam = getCameraPose();
  return T_base_cam * T_cam_obj;
}

tl::expected<common::VectorArm, common::VectorArm>
DynamicsSolver::solveIK(const Eigen::Isometry3d &target_pose,
                        const common::VectorArm &arm_q_guess, double tol,
                        int max_iter) {
  constexpr int MAX_RETRIES = 5;
  common::VectorArm best_arm_q = arm_q_guess;
  double min_err = 1e9;

  // 转换目标位姿为 Pinocchio 的 SE3 格式
  const pinocchio::SE3 oMdes(target_pose.rotation(), target_pose.translation());

  // 预分配计算变量 (只针对 6 DoF 机械臂)
  Eigen::Matrix<double, common::ARM_JOINT_NUM, 1> v_arm;
  Eigen::Matrix<double, 6, common::JOINT_NUM> J_full; // 完整 6x9 雅可比
  Eigen::Matrix<double, 6, common::ARM_JOINT_NUM>
      J_arm; // 机械臂部分 6x6 雅可比
  Eigen::Matrix<double, common::ARM_JOINT_NUM, common::ARM_JOINT_NUM> H;
  Eigen::Matrix<double, common::ARM_JOINT_NUM, 1> g;

  for (auto restart = 0; restart < MAX_RETRIES; ++restart) {
    // 初始化当前机械臂关节角
    common::VectorArm arm_q =
        (restart == 0) ? arm_q_guess : generateRandomArmPositions();
    bool collision_detected = false;

    // 迭代循环
    for (int i = 0; i < max_iter; ++i) {
      // 组装完整关节向量 (使用当前云台状态)
      const common::VectorJ full_q =
          common::combineJoints(arm_q, m_current_gimbal_q);

      // 计算正向运动学和关节雅可比
      pinocchio::computeJointJacobians(m_model, m_data, full_q);
      pinocchio::updateFramePlacements(m_model, m_data);

      // 计算当前位姿和目标位姿之间的 6D 误差 (在末端局部坐标系下)
      const pinocchio::SE3 &oMcurr = m_data.oMf[m_tcp_frame_id];
      const pinocchio::SE3 iMd = oMcurr.actInv(oMdes);
      Eigen::Matrix<double, 6, 1> err = pinocchio::log6(iMd).toVector();

      const double current_err_norm = err.norm();
      if (current_err_norm < min_err) {
        min_err = current_err_norm;
        best_arm_q = arm_q;
      }

      // 判断是否收敛
      if (current_err_norm < tol) {
        // Check collision (使用完整 9D 配置)
        bool collision = pinocchio::computeCollisions(
            m_model, m_data, m_geom_model, m_geom_data, full_q, true);

        if (!collision) {
          return arm_q;
        } else {
          collision_detected = true;
          break;
        }
      }

      // 计算完整 6xN 雅可比矩阵 (LOCAL frame to match log6 error)
      pinocchio::getFrameJacobian(m_model, m_data, m_tcp_frame_id,
                                  pinocchio::LOCAL, J_full);

      // 按正确的 Pinocchio 索引提取机械臂部分的雅可比列
      for (int j = 0; j < common::ARM_JOINT_NUM; ++j) {
        J_arm.col(j) = J_full.col(common::ARM_Q_INDICES[j]);
      }

      // 自适应阻尼
      constexpr double base_lambda = 1e-3;
      const double adaptive_lambda = base_lambda + 0.05 * current_err_norm;
      const double lambda_sq = adaptive_lambda * adaptive_lambda;

      // 标准 DLS 求解: (J^T * J + lambda^2 * I) * v = J^T * err
      H = J_arm.transpose() * J_arm;
      H.diagonal().array() += lambda_sq;
      g = J_arm.transpose() * err;

      // 使用 LDLT 分解求解线性方程
      v_arm = H.ldlt().solve(g);

      // 步长限制
      if (constexpr double max_step = 0.5; v_arm.norm() > max_step) {
        v_arm = v_arm.normalized() * max_step;
      }

      // 早停
      if (v_arm.norm() < 1e-6) {
        break;
      }

      // 更新机械臂关节角
      arm_q += v_arm;

      // 关节限位夹钳 (使用正确的 Pinocchio 索引)
      for (int j = 0; j < common::ARM_JOINT_NUM; ++j) {
        const int idx = common::ARM_Q_INDICES[j];
        arm_q[j] = std::clamp(arm_q[j], m_model.lowerPositionLimit[idx],
                              m_model.upperPositionLimit[idx]);
      }
    }

    // 如果第一次尝试结果尚可，则不再尝试随机重启
    if (constexpr double loose_tol = 1e-2;
        restart == 0 && min_err < loose_tol && !collision_detected) {
      const common::VectorJ best_full_q =
          common::combineJoints(best_arm_q, m_current_gimbal_q);
      bool best_q_collision = pinocchio::computeCollisions(
          m_model, m_data, m_geom_model, m_geom_data, best_full_q, true);
      if (!best_q_collision) {
        return tl::unexpected(best_arm_q);
      }
    }
  }

  return tl::unexpected(best_arm_q);
}

tl::expected<common::VectorArm, common::VectorArm>
DynamicsSolver::solveIK5DoF(const Eigen::Vector3d &target_position,
                            const Eigen::Vector3d &approach_direction,
                            const common::VectorArm &arm_q_guess, double tol,
                            int max_iter) {
  constexpr int MAX_RETRIES = 5;
  common::VectorArm best_arm_q = arm_q_guess;
  double min_err = 1e9;

  // 归一化逼近方向
  const Eigen::Vector3d z_des = approach_direction.normalized();

  // 预分配计算变量
  Eigen::Matrix<double, common::ARM_JOINT_NUM, 1> v_arm;
  Eigen::Matrix<double, 6, common::JOINT_NUM> J_full;
  Eigen::Matrix<double, 6, common::ARM_JOINT_NUM> J_arm;
  Eigen::Matrix<double, 5, common::ARM_JOINT_NUM> J_5dof; // 5DoF 雅可比
  Eigen::Matrix<double, common::ARM_JOINT_NUM, common::ARM_JOINT_NUM> H;
  Eigen::Matrix<double, common::ARM_JOINT_NUM, 1> g;
  Eigen::Matrix<double, 5, 1> err_5dof;

  // 构造与 z_des 垂直的两个正交基向量 (e1, e2)
  // ori_cross 投影到 {e1, e2} 上作为方向误差，避免固定取 x,y 分量在
  // z_des ≈ [1,0,0] 时退化（ori_cross[0] 恒为 0）
  Eigen::Vector3d e1, e2;
  if (std::abs(z_des[0]) < 0.9) {
    e1 = (Eigen::Vector3d::UnitX() - z_des[0] * z_des).normalized();
  } else {
    e1 = (Eigen::Vector3d::UnitY() - z_des[1] * z_des).normalized();
  }
  e2 = z_des.cross(e1); // 已是单位向量

  for (int restart = 0; restart < MAX_RETRIES; ++restart) {
    common::VectorArm arm_q =
        (restart == 0) ? arm_q_guess : generateRandomArmPositions();
    bool collision_detected = false;

    for (int i = 0; i < max_iter; ++i) {
      const common::VectorJ full_q =
          common::combineJoints(arm_q, m_current_gimbal_q);

      pinocchio::computeJointJacobians(m_model, m_data, full_q);
      pinocchio::updateFramePlacements(m_model, m_data);

      // 获取当前末端位姿
      const pinocchio::SE3 &oMcurr = m_data.oMf[m_tcp_frame_id];
      const Eigen::Vector3d p_curr = oMcurr.translation();
      const Eigen::Matrix3d R_curr = oMcurr.rotation();
      const Eigen::Vector3d z_curr = R_curr.col(2); // 末端 Z 轴

      // 计算 5D 误差:
      // [0-2]: 位置误差 (世界系)
      // [3-4]: 方向误差 (用叉积的 x, y 分量表示 Z 轴偏差)
      const Eigen::Vector3d pos_err = target_position - p_curr;

      // 方向误差: z_curr × z_des 给出旋转轴和角度的 sin
      // 只取 x, y 分量 (在世界系下) 来约束 roll, pitch
      // 当 z_curr ≈ z_des 时，叉积趋近于零
      const Eigen::Vector3d ori_cross = z_curr.cross(z_des);

      err_5dof.head<3>() = pos_err;
      err_5dof[3] = ori_cross.dot(e1); // 投影到 e1（与 z_des 垂直）
      err_5dof[4] = ori_cross.dot(e2); // 投影到 e2（与 z_des 垂直）
      // 沿 z_des 方向的分量对应绕逼近轴旋转（yaw 自由度），故意忽略

      const double current_err_norm = err_5dof.norm();
      if (current_err_norm < min_err) {
        min_err = current_err_norm;
        best_arm_q = arm_q;
      }

      // 收敛判断
      if (current_err_norm < tol) {
        bool collision = pinocchio::computeCollisions(
            m_model, m_data, m_geom_model, m_geom_data, full_q, true);
        if (!collision) {
          return arm_q;
        } else {
          collision_detected = true;
          break;
        }
      }

      // 获取世界系雅可比 (6 x 9)
      pinocchio::getFrameJacobian(m_model, m_data, m_tcp_frame_id,
                                  pinocchio::WORLD, J_full);

      // 提取机械臂部分 (6 x 6)
      for (int j = 0; j < common::ARM_JOINT_NUM; ++j) {
        J_arm.col(j) = J_full.col(common::ARM_Q_INDICES[j]);
      }

      // 构建 5DoF 雅可比:
      // 前 3 行: 位置雅可比 (直接使用)
      // 后 2 行: 方向雅可比 (从角速度到 z 轴变化率的映射)
      //
      // d(z_curr)/dt = omega × z_curr
      // d(z_curr × z_des)/dt = (omega × z_curr) × z_des
      //                      = omega (z_curr · z_des) - z_curr (omega · z_des)
      //                      = [z_des × z_curr]_× omega + (z_curr · z_des)
      //                      omega
      // 取 x, y 分量: 投影到世界 X, Y 轴
      //
      // 简化: 当 z_curr ≈ z_des 时，d(cross)/dt ≈ [0 0 -1; 0 0 0; 1 0 0] @
      // d(z_curr)/dt 的变化 更直接的方法: 使用角速度雅可比的前两行 (绕世界 X, Y
      // 轴的角速度)

      J_5dof.topRows<3>() = J_arm.topRows<3>(); // 位置雅可比

      // 方向雅可比: 我们需要 d(err_5dof[3:5])/d(q)
      // err[3] = (z_curr × z_des).x = z_curr.y * z_des.z - z_curr.z * z_des.y
      // err[4] = (z_curr × z_des).y = z_curr.z * z_des.x - z_curr.x * z_des.z
      //
      // d(z_curr)/d(q) 需要从角速度雅可比推导:
      // z_curr = R_curr * [0,0,1]^T
      // d(z_curr) = [omega]_× * z_curr, 其中 omega = J_ang * dq
      //
      // 因此 d(err[3])/d(q) 和 d(err[4])/d(q) 可以通过链式法则计算
      // 但为了简化，我们使用近似：当 z_curr ≈ z_des 时，
      // 角速度雅可比的 (wx, wy) 分量直接对应 roll/pitch 变化

      // 更精确的做法：直接计算
      const Eigen::Matrix<double, 3, common::ARM_JOINT_NUM> J_ang =
          J_arm.bottomRows<3>();

      // d(z_curr)/d(q) = [z_curr]_× * J_ang^T 的转置 = -[z_curr]_× * J_ang
      // 其中 [z_curr]_× 是反对称矩阵
      Eigen::Matrix3d z_curr_skew;
      z_curr_skew << 0, -z_curr[2], z_curr[1], z_curr[2], 0, -z_curr[0],
          -z_curr[1], z_curr[0], 0;

      // d(z_curr)/d(q) 是 3 x 6 矩阵
      const Eigen::Matrix<double, 3, common::ARM_JOINT_NUM> dz_dq =
          -z_curr_skew * J_ang;

      // d(z_curr × z_des)/d(q)
      // = d(z_curr)/d(q) × z_des (叉积逐列进行)
      // 取 x, y 分量
      Eigen::Matrix3d z_des_skew;
      z_des_skew << 0, -z_des[2], z_des[1], z_des[2], 0, -z_des[0], -z_des[1],
          z_des[0], 0;

      // d(cross)/d(q) = -[z_des]_× * d(z_curr)/d(q) = z_des_skew * z_curr_skew * J_ang
      const Eigen::Matrix<double, 3, common::ARM_JOINT_NUM> dcross_dq =
          -z_des_skew * dz_dq;

      // 投影到 {e1, e2}（与 z_des 垂直的正交基），避免固定取 x,y 分量时的退化
      J_5dof.row(3) = e1.transpose() * dcross_dq;
      J_5dof.row(4) = e2.transpose() * dcross_dq;

      // 自适应阻尼 DLS
      constexpr double base_lambda = 1e-3;
      const double adaptive_lambda = base_lambda + 0.05 * current_err_norm;
      const double lambda_sq = adaptive_lambda * adaptive_lambda;

      // (J^T J + λ²I) v = J^T err
      H = J_5dof.transpose() * J_5dof;
      H.diagonal().array() += lambda_sq;
      g = J_5dof.transpose() * err_5dof;

      // 添加次要目标：最小化关节移动量 (在零空间中)
      // v = v_primary + (I - J^+ J) * v_secondary
      // 这里简化为在 H 中添加小的正则项，偏向 q_guess
      constexpr double null_space_weight = 0.01;
      g += null_space_weight * (arm_q_guess - arm_q);

      v_arm = H.ldlt().solve(g);

      // 步长限制
      if (constexpr double max_step = 0.5; v_arm.norm() > max_step) {
        v_arm = v_arm.normalized() * max_step;
      }

      if (v_arm.norm() < 1e-6) {
        break;
      }

      arm_q += v_arm;

      // 关节限位
      for (int j = 0; j < common::ARM_JOINT_NUM; ++j) {
        const int idx = common::ARM_Q_INDICES[j];
        arm_q[j] = std::clamp(arm_q[j], m_model.lowerPositionLimit[idx],
                              m_model.upperPositionLimit[idx]);
      }
    }

    // 宽松收敛判断
    if (constexpr double loose_tol = 1e-2;
        restart == 0 && min_err < loose_tol && !collision_detected) {
      const common::VectorJ best_full_q =
          common::combineJoints(best_arm_q, m_current_gimbal_q);
      bool best_q_collision = pinocchio::computeCollisions(
          m_model, m_data, m_geom_model, m_geom_data, best_full_q, true);
      if (!best_q_collision) {
        return tl::unexpected(best_arm_q);
      }
    }
  }

  // 诊断：区分"未收敛"和"收敛但碰撞"
  const common::VectorJ best_full_q_diag = common::combineJoints(best_arm_q, m_current_gimbal_q);
  const bool final_collision = pinocchio::computeCollisions(
      m_model, m_data, m_geom_model, m_geom_data, best_full_q_diag, true);
  m_logger->warn(
      "solveIK5DoF failed: min_err={:.4f}, collision={}, "
      "target=[{:.3f},{:.3f},{:.3f}], approach=[{:.3f},{:.3f},{:.3f}]",
      min_err, final_collision,
      target_position.x(), target_position.y(), target_position.z(),
      approach_direction.x(), approach_direction.y(), approach_direction.z());
  return tl::unexpected(best_arm_q);
}

bool DynamicsSolver::checkPathCollision(const common::VectorArm &arm_q_start,
                                        const common::VectorArm &arm_q_goal,
                                        int num_samples) {
  for (int i = 1; i <= num_samples; ++i) {
    const double t = static_cast<double>(i) / num_samples;
    const common::VectorArm arm_q_sample =
        (1.0 - t) * arm_q_start + t * arm_q_goal;
    // 使用当前云台状态进行碰撞检测
    const common::VectorJ full_q =
        common::combineJoints(arm_q_sample, m_current_gimbal_q);
    if (pinocchio::computeCollisions(m_model, m_data, m_geom_model, m_geom_data,
                                     full_q, true)) {
      return true;
    }
  }
  return false;
}

void DynamicsSolver::setBaseLinkCollisionEnabled(bool enabled) {
  for (size_t idx : m_base_link_pair_indices) {
    m_geom_data.activeCollisionPairs[idx] = enabled;
  }
}

common::VectorArm DynamicsSolver::generateRandomArmPositions() {
  common::VectorArm arm_q;
  arm_q.setZero();

  thread_local std::mt19937_64 rng{std::random_device{}()};

  for (int i = 0; i < common::ARM_JOINT_NUM; ++i) {
    const int idx = common::ARM_Q_INDICES[i];
    const double lower = m_model.lowerPositionLimit[idx];
    const double upper = m_model.upperPositionLimit[idx];
    std::uniform_real_distribution<double> dist(lower, upper);
    arm_q[i] = dist(rng);
  }

  return arm_q;
}
} // namespace yandy::modules

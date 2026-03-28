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
#include <toml++/toml.hpp>
#include "yandy/core/Logger.hpp"

#define YANDY_URDF_PATH   YANDY_CONFIG_PATH "urdf/yandy_urdf.urdf"
#define YANDY_SOLVER_CONFIG YANDY_CONFIG_PATH "solver.toml"

namespace yandy::modules {

// ============================================================================
//  构造函数
// ============================================================================

template<bool kFloating>
DynamicsSolver<kFloating>::DynamicsSolver()
    : m_geom_data(m_geom_model)
{
    m_logger = core::create_logger("Solver", spdlog::level::info);
    m_logger->info("Initializing DynamicsSolver<kFloating={}>", kFloating);

    // ── 读取 solver.toml ────────────────────────────────────────────────────
    try {
        auto tbl = toml::parse_file(YANDY_SOLVER_CONFIG);
        m_mecanum_vel_comp = tbl["mecanum_vel_compensation"].value<bool>().value_or(false);

        // toml 中的 floating_base 为信息性字段，模板参数才是权威来源
        bool toml_floating = tbl["floating_base"].value<bool>().value_or(false);
        if (toml_floating != kFloating) {
            m_logger->warn(
                "solver.toml floating_base={} but DynamicsSolver<{}> was instantiated "
                "— template parameter takes precedence",
                toml_floating, kFloating);
        }
        m_logger->info("solver.toml loaded: mecanum_vel_comp={}", m_mecanum_vel_comp);
    } catch (...) {
        m_logger->warn("solver.toml not found or parse error, using defaults "
                       "(mecanum_vel_comp=false)");
    }

    // ── 加载 URDF，按 kFloating 选择 pinocchio 模型构建方式 ─────────────────
    m_logger->info("Loading URDF from {}", YANDY_URDF_PATH);
    try {
        if constexpr (kFloating) {
            pinocchio::urdf::buildModel(YANDY_URDF_PATH,
                                        pinocchio::JointModelFreeFlyer(), m_model);
            m_logger->info("Floating-base model built (JointModelFreeFlyer).");
        } else {
            pinocchio::urdf::buildModel(YANDY_URDF_PATH, m_model);
            m_logger->info("Fixed-base model built.");
        }

        // ── 构建几何模型 (碰撞检测) ────────────────────────────────────────
        std::vector<std::string> package_dirs;
        std::string config_path = YANDY_CONFIG_PATH;
        package_dirs.push_back(config_path + "urdf");

        pinocchio::urdf::buildGeom(m_model, YANDY_URDF_PATH, pinocchio::COLLISION,
                                   m_geom_model, package_dirs);
        m_geom_model.addAllCollisionPairs();

        // 移除距离 ≤ 2 的相邻关节碰撞对 (自身、父子、祖孙关系)
        auto jointDistance = [this](pinocchio::JointIndex j1,
                                    pinocchio::JointIndex j2) -> int {
            if (j1 == j2) return 0;
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

        for (size_t i = 0; i < m_geom_model.collisionPairs.size();) {
            const auto& cp   = m_geom_model.collisionPairs[i];
            const auto joint1 = m_geom_model.geometryObjects[cp.first].parentJoint;
            const auto joint2 = m_geom_model.geometryObjects[cp.second].parentJoint;
            if (jointDistance(joint1, joint2) <= 2) {
                m_geom_model.removeCollisionPair(cp);
            } else {
                i++;
            }
        }

        m_geom_data = pinocchio::GeometryData(m_geom_model);

        // 缓存涉及 base_link 的碰撞对索引
        for (size_t i = 0; i < m_geom_model.collisionPairs.size(); ++i) {
            const auto& cp    = m_geom_model.collisionPairs[i];
            const auto& name1 = m_geom_model.geometryObjects[cp.first].name;
            const auto& name2 = m_geom_model.geometryObjects[cp.second].name;
            if (name1.find("base_link") != std::string::npos ||
                name2.find("base_link") != std::string::npos) {
                m_base_link_pair_indices.push_back(i);
            }
        }

        m_logger->info("Geometry model loaded. Collision pairs: {}",
                       m_geom_model.collisionPairs.size());
    } catch (const std::exception& e) {
        m_logger->error("Error loading URDF: {}", e.what());
        throw;
    }

    m_data = pinocchio::Data(m_model);

    // ── 帧/关节 ID 查找 ────────────────────────────────────────────────────
    if (m_model.existJointName("joint_6")) {
        m_ee_joint_id = m_model.getJointId("joint_6");
    } else if (m_model.existJointName("joint_5")) {
        m_logger->warn("joint_6 not found, falling back to joint_5");
        m_ee_joint_id = m_model.getJointId("joint_5");
    } else {
        m_logger->error("No valid end-effector joint found.");
        throw std::runtime_error("No valid end-effector joint found");
    }
    if (m_model.existFrame("gripper_tcp")) {
        m_tcp_frame_id = m_model.getFrameId("gripper_tcp");
    } else {
        m_logger->error("No valid end-effector frame (gripper_tcp) found.");
        throw std::runtime_error("No valid end-effector frame found");
    }
    if (m_model.existFrame("camera_optical_frame")) {
        m_camera_frame_id = m_model.getFrameId("camera_optical_frame");
    } else {
        m_logger->error("No valid camera frame (camera_optical_frame) found.");
        throw std::runtime_error("No valid camera frame found");
    }
    if (m_model.existFrame("store_frame_1") && m_model.existFrame("store_frame_2")) {
        m_store_frames[0] = m_model.getFrameId("store_frame_1");
        m_store_frames[1] = m_model.getFrameId("store_frame_2");
    } else {
        m_logger->error("No valid store frames found.");
        throw std::runtime_error("No valid store frames found");
    }

    // ── 动态计算关节索引 ────────────────────────────────────────────────────
    // 使用 m_model.joints[id].idx_q() / idx_v() 计算正确的索引
    // 固定基座: arm_v_idx = ARM_Q_INDICES (一致)
    // 浮动基座: arm_v_idx = ARM_Q_INDICES + BASE_V_OFFSET (每个均偏移 6)
    //
    // joint 名与 URDF 树结构对应:
    //   pinocchio DFS 顺序 (固定基座): 0=universe, 1=joint_1,
    //   2=gimbal_joint_1(prismatic), 3=gimbal_joint_2, 4=gimbal_joint_3,
    //   5=joint_2, ..., 9=joint_6
    //   对应 ARM_Q_INDICES = {0, 4, 5, 6, 7, 8} (v 空间索引)

    const char* arm_joint_names[common::ARM_JOINT_NUM] = {
        "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
    };
    const char* gimbal_joint_names[common::GIMBAL_JOINT_NUM] = {
        "gimbal_joint_1", "gimbal_joint_2", "gimbal_joint_3"
    };

    for (int i = 0; i < common::ARM_JOINT_NUM; ++i) {
        if (!m_model.existJointName(arm_joint_names[i])) {
            m_logger->error("Arm joint '{}' not found in model!", arm_joint_names[i]);
            throw std::runtime_error(std::string("Arm joint not found: ") + arm_joint_names[i]);
        }
        auto jid = m_model.getJointId(arm_joint_names[i]);
        m_arm_q_idx[i] = static_cast<int>(m_model.joints[jid].idx_q());
        m_arm_v_idx[i] = static_cast<int>(m_model.joints[jid].idx_v());
    }

    for (int i = 0; i < common::GIMBAL_JOINT_NUM; ++i) {
        if (!m_model.existJointName(gimbal_joint_names[i])) {
            m_logger->error("Gimbal joint '{}' not found in model!", gimbal_joint_names[i]);
            throw std::runtime_error(std::string("Gimbal joint not found: ") + gimbal_joint_names[i]);
        }
        auto jid = m_model.getJointId(gimbal_joint_names[i]);
        m_gimbal_q_idx[i] = static_cast<int>(m_model.joints[jid].idx_q());
        m_gimbal_v_idx[i] = static_cast<int>(m_model.joints[jid].idx_v());
    }

    // 正确性断言: 固定基座的索引应与 ARM_Q_INDICES 一致
    if constexpr (!kFloating) {
        for (int i = 0; i < common::ARM_JOINT_NUM; ++i) {
            if (m_arm_v_idx[i] != common::ARM_Q_INDICES[i]) {
                m_logger->error(
                    "arm_v_idx[{}]={} != ARM_Q_INDICES[{}]={} — URDF tree order changed?",
                    i, m_arm_v_idx[i], i, common::ARM_Q_INDICES[i]);
                throw std::runtime_error("Arm joint v-index mismatch (fixed-base)");
            }
        }
    } else {
        // 浮动基座: 每个 v 索引应等于固定基座索引 + BASE_V_OFFSET (=6)
        for (int i = 0; i < common::ARM_JOINT_NUM; ++i) {
            if (m_arm_v_idx[i] != common::ARM_Q_INDICES[i] + BASE_V_OFFSET) {
                m_logger->error(
                    "arm_v_idx[{}]={} != ARM_Q_INDICES[{}]+{} ({}) — floating offset mismatch",
                    i, m_arm_v_idx[i], i, BASE_V_OFFSET,
                    common::ARM_Q_INDICES[i] + BASE_V_OFFSET);
                throw std::runtime_error("Arm joint v-index mismatch (floating-base)");
            }
        }
    }

    // ── 预分配缓冲区 ────────────────────────────────────────────────────────
    f_ext_.resize(m_model.njoints, pinocchio::Force::Zero());
    m_J_full.resize(6, m_model.nv);  // 固定基座: 6×9; 浮动基座: 6×15

    m_logger->info(
        "Model loaded. nq={}, nv={}, njoints={}, arm_v_idx=[{},{},{},{},{},{}], "
        "gimbal_v_idx=[{},{},{}]",
        m_model.nq, m_model.nv, m_model.njoints,
        m_arm_v_idx[0], m_arm_v_idx[1], m_arm_v_idx[2],
        m_arm_v_idx[3], m_arm_v_idx[4], m_arm_v_idx[5],
        m_gimbal_v_idx[0], m_gimbal_v_idx[1], m_gimbal_v_idx[2]);
}

// ============================================================================
//  内部辅助: 构建完整 pinocchio 向量
// ============================================================================

template<bool kFloating>
Eigen::VectorXd DynamicsSolver<kFloating>::buildFullQ(
    const common::VectorArm& arm_q,
    const common::VectorGimbal& gimbal_q) const
{
    Eigen::VectorXd q = Eigen::VectorXd::Zero(m_model.nq);

    if constexpr (kFloating) {
        // pinocchio FreeFlyer 配置约定: [x, y, z, qx, qy, qz, qw]
        // 底盘置于 origin，identity 旋转: q[3..5]=0, q[6]=1
        q[6] = 1.0;  // qw = 1 (identity quaternion)
    }

    for (int i = 0; i < common::ARM_JOINT_NUM; ++i)
        q[m_arm_q_idx[i]] = arm_q[i];
    for (int i = 0; i < common::GIMBAL_JOINT_NUM; ++i)
        q[m_gimbal_q_idx[i]] = gimbal_q[i];

    return q;
}

template<bool kFloating>
Eigen::VectorXd DynamicsSolver<kFloating>::buildFullQ(
    const common::VectorArm& arm_q,
    const common::VectorGimbal& gimbal_q,
    const Eigen::Quaterniond& base_quat) const
{
    Eigen::VectorXd q = buildFullQ(arm_q, gimbal_q);  // base = identity

    if constexpr (kFloating) {
        // 覆盖底盘旋转部分: pinocchio 约定 [qx, qy, qz, qw]
        q[3] = base_quat.x();
        q[4] = base_quat.y();
        q[5] = base_quat.z();
        q[6] = base_quat.w();
    }

    return q;
}

template<bool kFloating>
Eigen::VectorXd DynamicsSolver<kFloating>::buildFullV(
    const common::VectorArm& arm_v,
    const common::VectorGimbal& gimbal_v,
    const Eigen::Vector3d& base_lin_vel,
    const Eigen::Vector3d& base_ang_vel) const
{
    Eigen::VectorXd v = Eigen::VectorXd::Zero(m_model.nv);

    if constexpr (kFloating) {
        // pinocchio FreeFlyer 速度约定: [vx, vy, vz, wx, wy, wz] (LOCAL frame)
        v[0] = base_lin_vel[0];
        v[1] = base_lin_vel[1];
        v[2] = base_lin_vel[2];
        v[3] = base_ang_vel[0];
        v[4] = base_ang_vel[1];
        v[5] = base_ang_vel[2];
    }

    for (int i = 0; i < common::ARM_JOINT_NUM; ++i)
        v[m_arm_v_idx[i]] = arm_v[i];
    for (int i = 0; i < common::GIMBAL_JOINT_NUM; ++i)
        v[m_gimbal_v_idx[i]] = gimbal_v[i];

    return v;
}

// ============================================================================
//  运动学更新
// ============================================================================

template<bool kFloating>
void DynamicsSolver<kFloating>::updateKinematics(
    const common::VectorArm& arm_q,
    const common::VectorArm& arm_v,
    const common::VectorGimbal& gimbal_q)
{
    m_current_arm_q    = arm_q;
    m_current_arm_v    = arm_v;
    m_current_gimbal_q = gimbal_q;

    // 底盘始终设为 identity (FK 用于求末端位姿，与底盘绝对运动无关)
    const Eigen::VectorXd full_q = buildFullQ(arm_q, gimbal_q);
    const Eigen::VectorXd full_v = buildFullV(arm_v, common::VectorGimbal::Zero(),
                                               Eigen::Vector3d::Zero(),
                                               Eigen::Vector3d::Zero());

    pinocchio::forwardKinematics(m_model, m_data, full_q, full_v);
    pinocchio::updateFramePlacements(m_model, m_data);
}

// ============================================================================
//  动力学计算
// ============================================================================

template<bool kFloating>
common::VectorArm DynamicsSolver<kFloating>::computeGravity()
{
    const Eigen::VectorXd full_q = buildFullQ(m_current_arm_q, m_current_gimbal_q);
    const Eigen::VectorXd zero_v = Eigen::VectorXd::Zero(m_model.nv);

    const Eigen::VectorXd full_tau =
        pinocchio::rnea(m_model, m_data, full_q, zero_v, zero_v);

    // 提取机械臂力矩 (使用 m_arm_v_idx: RNEA 输出与 v 空间索引对齐)
    common::VectorArm tau_arm;
    for (int i = 0; i < common::ARM_JOINT_NUM; ++i)
        tau_arm[i] = full_tau[m_arm_v_idx[i]];
    return tau_arm;
}

template<bool kFloating>
common::VectorArm DynamicsSolver<kFloating>::computeDynamics(
    const Eigen::Vector3d& chassis_acc,
    const Eigen::Vector3d& chassis_omega,
    const Eigen::Quaterniond& chassis_quat)
{
    if constexpr (!kFloating) {
        // 固定基座: 底盘参数无意义，退化为纯重力补偿
        return computeGravity();
    } else {
        // ── 1. 构建带底盘姿态的配置向量 ────────────────────────────────────
        const Eigen::VectorXd full_q =
            buildFullQ(m_current_arm_q, m_current_gimbal_q, chassis_quat);

        // ── 2. 构建速度向量 ─────────────────────────────────────────────────
        // 底盘角速度来自 IMU (body frame)
        // 底盘线速度: 若启用 mecanum 补偿则可加入线速度 (暂留零，待扩展)
        const Eigen::Vector3d base_lin_vel = Eigen::Vector3d::Zero();
        const Eigen::VectorXd full_v = buildFullV(
            m_current_arm_v, common::VectorGimbal::Zero(),
            base_lin_vel, chassis_omega);

        // ── 3. 构建加速度向量 ───────────────────────────────────────────────
        // 重力约定:
        //   m_model.gravity 保持 pinocchio 默认 {0,0,-9.81,...} (处理 g(q) 项)
        //   RNEA: tau = M(q)*ddq + C(q,v)*v + g(q)
        //
        // 对于浮动基座, base acc 输入是底盘在 LOCAL (body) frame 下的真实加速度.
        //
        // IMU 测量的是比力 (specific force):
        //   a_IMU = a_real - g_world (body frame)
        //   ⟹ a_real_body = a_IMU + R^T * g_world
        //      其中 R = R_chassis_to_world (chassis_quat.toRotationMatrix())
        //          g_world = [0, 0, -9.81]
        //
        // 这是底盘在其自身 body frame 下相对于惯性系的真实线性加速度,
        // 正是 pinocchio RNEA FreeFlyer 所需要的 (ddq[0:3], LOCAL frame).
        //
        // 在 RNEA 内部 g(q) 已通过 model.gravity 计入, 而 M(q)*ddq 项中
        // base 的 ddq 贡献仅为惯性力 (已正确与 g(q) 分离), 不会重复.
        //
        // 静止验证: 底盘水平静止时:
        //   a_IMU ≈ [0, 0, +9.81] (body frame, Z朝上)
        //   R^T * g_world = R^T * [0,0,-9.81] ≈ [0,0,-9.81] (chassis 水平时 R=I)
        //   a_real_body ≈ [0,0,0]
        //   ⟹ base ddq[0:3] = 0 ⟹ RNEA = g(q) only = computeGravity() ✓

        const Eigen::Matrix3d R_world_to_chassis =
            chassis_quat.toRotationMatrix().transpose();  // R_chassis_to_world^T
        constexpr double GRAVITY = 9.81;
        const Eigen::Vector3d g_world(0.0, 0.0, -GRAVITY);
        const Eigen::Vector3d a_real_body = chassis_acc + R_world_to_chassis * g_world;

        // 组装完整加速度向量 (nv 维)
        // base ddq[0:2] = a_real_body (线性); base ddq[3:5] = 0 (角加速度忽略)
        // arm joint ddq = 0 (准静态补偿, 不前馈关节加速度)
        Eigen::VectorXd full_acc = Eigen::VectorXd::Zero(m_model.nv);
        full_acc[0] = a_real_body[0];
        full_acc[1] = a_real_body[1];
        full_acc[2] = a_real_body[2];

        // ── 4. 运行 RNEA ────────────────────────────────────────────────────
        const Eigen::VectorXd full_tau =
            pinocchio::rnea(m_model, m_data, full_q, full_v, full_acc);

        // ── 5. 提取机械臂力矩 ───────────────────────────────────────────────
        common::VectorArm tau_arm;
        for (int i = 0; i < common::ARM_JOINT_NUM; ++i)
            tau_arm[i] = full_tau[m_arm_v_idx[i]];
        return tau_arm;
    }
}

template<bool kFloating>
common::VectorArm DynamicsSolver<kFloating>::computeRNEA(
    const common::VectorArm& acc_des,
    const common::Vector6& ext_wrench_world)
{
    std::ranges::fill(f_ext_, pinocchio::Force::Zero());
    if (!ext_wrench_world.isZero()) {
        const auto& iso_world_to_local = m_data.oMi[m_ee_joint_id].inverse();
        const pinocchio::Force f_world(ext_wrench_world.head<3>(),
                                       ext_wrench_world.tail<3>());
        f_ext_[m_ee_joint_id] = iso_world_to_local.act(f_world);
    }

    const Eigen::VectorXd full_q = buildFullQ(m_current_arm_q, m_current_gimbal_q);
    const Eigen::VectorXd full_v = buildFullV(m_current_arm_v,
                                               common::VectorGimbal::Zero(),
                                               Eigen::Vector3d::Zero(),
                                               Eigen::Vector3d::Zero());

    Eigen::VectorXd full_acc = Eigen::VectorXd::Zero(m_model.nv);
    for (int i = 0; i < common::ARM_JOINT_NUM; ++i)
        full_acc[m_arm_v_idx[i]] = acc_des[i];

    const Eigen::VectorXd full_tau =
        pinocchio::rnea(m_model, m_data, full_q, full_v, full_acc, f_ext_);

    common::VectorArm tau_arm;
    for (int i = 0; i < common::ARM_JOINT_NUM; ++i)
        tau_arm[i] = full_tau[m_arm_v_idx[i]];
    return tau_arm;
}

// ============================================================================
//  位姿查询
// ============================================================================

template<bool kFloating>
Eigen::Isometry3d DynamicsSolver<kFloating>::getEndEffectorPose() const
{
    const pinocchio::SE3& se3 = m_data.oMf[m_tcp_frame_id];
    Eigen::Isometry3d pose   = Eigen::Isometry3d::Identity();
    pose.translation()       = se3.translation();
    pose.linear()            = se3.rotation();
    return pose;
}

template<bool kFloating>
Eigen::Isometry3d DynamicsSolver<kFloating>::getCameraPose() const
{
    return Eigen::Isometry3d(m_data.oMf[m_camera_frame_id].toHomogeneousMatrix());
}

template<bool kFloating>
Eigen::Isometry3d DynamicsSolver<kFloating>::getStoreFrame(size_t index) const
{
    return Eigen::Isometry3d(m_data.oMf[m_store_frames[index]].toHomogeneousMatrix());
}

template<bool kFloating>
Eigen::Isometry3d DynamicsSolver<kFloating>::transformObjectToBase(
    const Eigen::Isometry3d& T_cam_obj) const
{
    return getCameraPose() * T_cam_obj;
}

// ============================================================================
//  逆运动学: solveIK (6-DoF)
// ============================================================================

template<bool kFloating>
tl::expected<common::VectorArm, common::VectorArm>
DynamicsSolver<kFloating>::solveIK(
    const Eigen::Isometry3d& target_pose,
    const common::VectorArm& arm_q_guess,
    double tol,
    int max_iter)
{
    constexpr int MAX_RETRIES = 5;
    common::VectorArm best_arm_q = arm_q_guess;
    double min_err = 1e9;

    const pinocchio::SE3 oMdes(target_pose.rotation(), target_pose.translation());

    // 预分配局部变量 (只针对 ARM_JOINT_NUM DoF)
    Eigen::Matrix<double, common::ARM_JOINT_NUM, 1> v_arm;
    Eigen::Matrix<double, 6, common::ARM_JOINT_NUM> J_arm;
    Eigen::Matrix<double, common::ARM_JOINT_NUM, common::ARM_JOINT_NUM> H;
    Eigen::Matrix<double, common::ARM_JOINT_NUM, 1> g;

    // m_J_full (6 × nv) 已在构造时预分配

    for (int restart = 0; restart < MAX_RETRIES; ++restart) {
        common::VectorArm arm_q =
            (restart == 0) ? arm_q_guess : generateRandomArmPositions();
        bool collision_detected = false;

        for (int i = 0; i < max_iter; ++i) {
            // 构建完整配置向量 (底盘 = identity, 固定在 origin)
            const Eigen::VectorXd full_q = buildFullQ(arm_q, m_current_gimbal_q);

            pinocchio::computeJointJacobians(m_model, m_data, full_q);
            pinocchio::updateFramePlacements(m_model, m_data);

            // 计算 6D 误差 (LOCAL frame, 与 log6 一致)
            const pinocchio::SE3& oMcurr = m_data.oMf[m_tcp_frame_id];
            const pinocchio::SE3  iMd    = oMcurr.actInv(oMdes);
            Eigen::Matrix<double, 6, 1> err = pinocchio::log6(iMd).toVector();

            const double current_err_norm = err.norm();
            if (current_err_norm < min_err) {
                min_err    = current_err_norm;
                best_arm_q = arm_q;
            }

            if (current_err_norm < tol) {
                bool collision = pinocchio::computeCollisions(
                    m_model, m_data, m_geom_model, m_geom_data, full_q, true);
                if (!collision) return arm_q;
                collision_detected = true;
                break;
            }

            // 计算 LOCAL frame Jacobian (6 × nv)
            pinocchio::getFrameJacobian(m_model, m_data, m_tcp_frame_id,
                                        pinocchio::LOCAL, m_J_full);

            // 提取机械臂列 (使用 m_arm_v_idx，固定/浮动均正确)
            for (int j = 0; j < common::ARM_JOINT_NUM; ++j)
                J_arm.col(j) = m_J_full.col(m_arm_v_idx[j]);

            // 自适应阻尼 DLS
            constexpr double base_lambda = 1e-3;
            const double adaptive_lambda = base_lambda + 0.05 * current_err_norm;
            const double lambda_sq       = adaptive_lambda * adaptive_lambda;

            H = J_arm.transpose() * J_arm;
            H.diagonal().array() += lambda_sq;
            g = J_arm.transpose() * err;

            v_arm = H.ldlt().solve(g);

            if (constexpr double max_step = 0.5; v_arm.norm() > max_step)
                v_arm = v_arm.normalized() * max_step;
            if (v_arm.norm() < 1e-6) break;

            arm_q += v_arm;

            // 关节限位 (使用 m_arm_q_idx)
            for (int j = 0; j < common::ARM_JOINT_NUM; ++j) {
                arm_q[j] = std::clamp(arm_q[j],
                    m_model.lowerPositionLimit[m_arm_q_idx[j]],
                    m_model.upperPositionLimit[m_arm_q_idx[j]]);
            }
        }

        if (constexpr double loose_tol = 1e-2;
            restart == 0 && min_err < loose_tol && !collision_detected) {
            const Eigen::VectorXd best_full_q = buildFullQ(best_arm_q, m_current_gimbal_q);
            if (!pinocchio::computeCollisions(
                    m_model, m_data, m_geom_model, m_geom_data, best_full_q, true)) {
                return tl::unexpected(best_arm_q);
            }
        }
    }

    return tl::unexpected(best_arm_q);
}

// ============================================================================
//  逆运动学: solveIK5DoF (5-DoF, 位置 + 逼近方向)
// ============================================================================

template<bool kFloating>
tl::expected<common::VectorArm, common::VectorArm>
DynamicsSolver<kFloating>::solveIK5DoF(
    const Eigen::Vector3d& target_position,
    const Eigen::Vector3d& approach_direction,
    const common::VectorArm& arm_q_guess,
    double tol,
    int max_iter)
{
    constexpr int MAX_RETRIES = 5;
    common::VectorArm best_arm_q = arm_q_guess;
    double min_err = 1e9;

    const Eigen::Vector3d z_des = approach_direction.normalized();

    // 预分配局部变量
    Eigen::Matrix<double, common::ARM_JOINT_NUM, 1> v_arm;
    Eigen::Matrix<double, 6, common::ARM_JOINT_NUM> J_arm;
    Eigen::Matrix<double, 5, common::ARM_JOINT_NUM> J_5dof;
    Eigen::Matrix<double, common::ARM_JOINT_NUM, common::ARM_JOINT_NUM> H;
    Eigen::Matrix<double, common::ARM_JOINT_NUM, 1> g;
    Eigen::Matrix<double, 5, 1> err_5dof;

    // 构造与 z_des 垂直的正交基 {e1, e2} (避免固定取 x,y 时退化)
    Eigen::Vector3d e1, e2;
    if (std::abs(z_des[0]) < 0.9) {
        e1 = (Eigen::Vector3d::UnitX() - z_des[0] * z_des).normalized();
    } else {
        e1 = (Eigen::Vector3d::UnitY() - z_des[1] * z_des).normalized();
    }
    e2 = z_des.cross(e1);

    for (int restart = 0; restart < MAX_RETRIES; ++restart) {
        common::VectorArm arm_q =
            (restart == 0) ? arm_q_guess : generateRandomArmPositions();
        bool collision_detected = false;

        for (int i = 0; i < max_iter; ++i) {
            const Eigen::VectorXd full_q = buildFullQ(arm_q, m_current_gimbal_q);

            pinocchio::computeJointJacobians(m_model, m_data, full_q);
            pinocchio::updateFramePlacements(m_model, m_data);

            const pinocchio::SE3& oMcurr = m_data.oMf[m_tcp_frame_id];
            const Eigen::Vector3d p_curr = oMcurr.translation();
            const Eigen::Matrix3d R_curr = oMcurr.rotation();
            const Eigen::Vector3d z_curr = R_curr.col(2);

            // 5D 误差: [位置 3D, 方向 2D]
            const Eigen::Vector3d pos_err   = target_position - p_curr;
            const Eigen::Vector3d ori_cross = z_curr.cross(z_des);

            err_5dof.head<3>() = pos_err;
            err_5dof[3]        = ori_cross.dot(e1);
            err_5dof[4]        = ori_cross.dot(e2);

            const double current_err_norm = err_5dof.norm();
            if (current_err_norm < min_err) {
                min_err    = current_err_norm;
                best_arm_q = arm_q;
            }

            if (current_err_norm < tol) {
                bool collision = pinocchio::computeCollisions(
                    m_model, m_data, m_geom_model, m_geom_data, full_q, true);
                if (!collision) return arm_q;
                collision_detected = true;
                break;
            }

            // WORLD frame Jacobian (6 × nv)
            pinocchio::getFrameJacobian(m_model, m_data, m_tcp_frame_id,
                                        pinocchio::WORLD, m_J_full);

            // 提取机械臂列
            for (int j = 0; j < common::ARM_JOINT_NUM; ++j)
                J_arm.col(j) = m_J_full.col(m_arm_v_idx[j]);

            // 构建 5DoF Jacobian
            J_5dof.topRows<3>() = J_arm.topRows<3>();  // 位置行

            // 方向行: d(err[3:4])/dq 由链式法则推导
            const Eigen::Matrix<double, 3, common::ARM_JOINT_NUM> J_ang =
                J_arm.bottomRows<3>();

            Eigen::Matrix3d z_curr_skew;
            z_curr_skew <<        0, -z_curr[2],  z_curr[1],
                            z_curr[2],         0, -z_curr[0],
                           -z_curr[1],  z_curr[0],         0;

            const Eigen::Matrix<double, 3, common::ARM_JOINT_NUM> dz_dq =
                -z_curr_skew * J_ang;

            Eigen::Matrix3d z_des_skew;
            z_des_skew <<       0, -z_des[2],  z_des[1],
                           z_des[2],        0, -z_des[0],
                          -z_des[1],  z_des[0],        0;

            const Eigen::Matrix<double, 3, common::ARM_JOINT_NUM> dcross_dq =
                -z_des_skew * dz_dq;

            J_5dof.row(3) = e1.transpose() * dcross_dq;
            J_5dof.row(4) = e2.transpose() * dcross_dq;

            // 自适应阻尼 DLS
            constexpr double base_lambda = 1e-3;
            const double adaptive_lambda = base_lambda + 0.05 * current_err_norm;
            const double lambda_sq       = adaptive_lambda * adaptive_lambda;

            H = J_5dof.transpose() * J_5dof;
            H.diagonal().array() += lambda_sq;
            g = J_5dof.transpose() * err_5dof;

            // 零空间次要目标: 最小化与 q_guess 的偏差
            constexpr double null_space_weight = 0.01;
            g += null_space_weight * (arm_q_guess - arm_q);

            v_arm = H.ldlt().solve(g);

            if (constexpr double max_step = 0.5; v_arm.norm() > max_step)
                v_arm = v_arm.normalized() * max_step;
            if (v_arm.norm() < 1e-6) break;

            arm_q += v_arm;

            for (int j = 0; j < common::ARM_JOINT_NUM; ++j) {
                arm_q[j] = std::clamp(arm_q[j],
                    m_model.lowerPositionLimit[m_arm_q_idx[j]],
                    m_model.upperPositionLimit[m_arm_q_idx[j]]);
            }
        }

        if (constexpr double loose_tol = 1e-2;
            restart == 0 && min_err < loose_tol && !collision_detected) {
            const Eigen::VectorXd best_full_q = buildFullQ(best_arm_q, m_current_gimbal_q);
            if (!pinocchio::computeCollisions(
                    m_model, m_data, m_geom_model, m_geom_data, best_full_q, true)) {
                return tl::unexpected(best_arm_q);
            }
        }
    }

    // 诊断日志
    const Eigen::VectorXd best_full_q_diag = buildFullQ(best_arm_q, m_current_gimbal_q);
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

// ============================================================================
//  碰撞检测
// ============================================================================

template<bool kFloating>
bool DynamicsSolver<kFloating>::checkPathCollision(
    const common::VectorArm& arm_q_start,
    const common::VectorArm& arm_q_goal,
    int num_samples)
{
    for (int i = 1; i <= num_samples; ++i) {
        const double t = static_cast<double>(i) / num_samples;
        const common::VectorArm arm_q_sample = (1.0 - t) * arm_q_start + t * arm_q_goal;
        const Eigen::VectorXd full_q = buildFullQ(arm_q_sample, m_current_gimbal_q);
        if (pinocchio::computeCollisions(
                m_model, m_data, m_geom_model, m_geom_data, full_q, true)) {
            return true;
        }
    }
    return false;
}

template<bool kFloating>
void DynamicsSolver<kFloating>::setBaseLinkCollisionEnabled(bool enabled)
{
    for (size_t idx : m_base_link_pair_indices)
        m_geom_data.activeCollisionPairs[idx] = enabled;
}

// ============================================================================
//  末端负载设置
// ============================================================================

template<bool kFloating>
void DynamicsSolver<kFloating>::setEndEffectorMass(double mass)
{
    // 获取 link_6 的索引 (gripper_tcp 所在线性)
    // 使用 frame 来获取 link_6 的父关节
    const std::string link6_name = "link_6";
    
    if (!m_model.existFrame(link6_name)) {
        m_logger->warn("link_6 frame not found, cannot set end-effector mass");
        return;
    }
    
    const auto link6_frame_id = m_model.getFrameId(link6_name);
    const auto& frame = m_model.frames[link6_frame_id];
    const auto link6_joint_id = frame.parentJoint;
    
    // 获取 link_6 的惯性参数
    auto& link6_inertia = m_model.inertias[link6_joint_id];
    
    // 保存原始质量 (首次调用时记录)
    // 使用 static 变量 + once_flag 确保只初始化一次
    static std::once_flag init_flag;
    static double original_mass = 0.0;
    static pinocchio::Inertia original_inertia;
    
    std::call_once(init_flag, [&]() {
        original_mass = link6_inertia.mass();
        original_inertia = link6_inertia;
        m_logger->info("End-effector original inertia saved: mass={:.3f} kg", original_mass);
    });
    
    // 构建负载的 Spatial Inertia 对象
    // 假设负载质心相对于 link_6 中心偏移约 10cm (沿末端 Z 轴方向)
    // 矿石被夹持时，重心在末端前方
    Eigen::Vector3d payload_com_offset(0.0, 0.0, 0.10); 
    // 矿石自身的转动惯量 (简单视作小球)
    Eigen::Matrix3d payload_rot_inertia = Eigen::Matrix3d::Identity() * 0.001; 
    pinocchio::Inertia payload_inertia(mass, payload_com_offset, payload_rot_inertia);
    
    // 关键操作：直接把负载的惯量"加"到关节上
    link6_inertia = original_inertia + payload_inertia;
    
    // 注意：Pinocchio 的 RNEA 每次都会从 model.inertias 重新计算，
    // 所以修改 inertias 后不需要额外更新 data
    
    m_logger->info("End-effector inertia updated: mass={:.3f} kg (original: {:.3f}, load: {:.3f})",
                   original_mass + mass, original_mass, mass);
}

// ============================================================================
//  随机关节角生成
// ============================================================================

template<bool kFloating>
common::VectorArm DynamicsSolver<kFloating>::generateRandomArmPositions()
{
    common::VectorArm arm_q;
    arm_q.setZero();
    thread_local std::mt19937_64 rng{std::random_device{}()};
    for (int i = 0; i < common::ARM_JOINT_NUM; ++i) {
        const double lower = m_model.lowerPositionLimit[m_arm_q_idx[i]];
        const double upper = m_model.upperPositionLimit[m_arm_q_idx[i]];
        std::uniform_real_distribution<double> dist(lower, upper);
        arm_q[i] = dist(rng);
    }
    return arm_q;
}

// ============================================================================
//  显式模板实例化 (两种变体均在此 TU 中实例化)
// ============================================================================

template class DynamicsSolver<false>;
template class DynamicsSolver<true>;

} // namespace yandy::modules

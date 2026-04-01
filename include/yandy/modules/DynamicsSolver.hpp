#ifndef YANDY_ARM_DYNAMICSSOLVER_HPP
#define YANDY_ARM_DYNAMICSSOLVER_HPP

#include <array>
#include <yandy/common/Types.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/collision/collision.hpp>
#include <eigen3/Eigen/Dense>
#include <spdlog/spdlog.h>
#include <tl/expected.hpp>

namespace yandy::modules
{

/**
 * @brief 机器人动力学求解器
 *
 * @tparam kFloating 是否启用浮动底盘模型 (pinocchio::JointModelFreeFlyer)
 *   - false (默认): 固定基座模型，computeDynamics() 等同于 computeGravity()
 *   - true:         浮动基座模型，computeDynamics() 将 IMU 数据纳入 RNEA
 *
 * 两种模板实例均在 DynamicsSolver.cpp 末尾显式实例化。
 * 运行时通过 config/solver.toml 中的 floating_base 选项决定使用哪个实例，
 * 在 Robot.cpp 中通过 std::variant<DynamicsSolverFixed, DynamicsSolverFloating> 选择。
 */
template<bool kFloating = false>
class DynamicsSolver
{
public:
    // ── 编译期常量 ────────────────────────────────────────────────────────
    /// FreeFlyer 在 pinocchio q 向量中额外占用的维度 (xyz + qxyz + qw)
    static constexpr int BASE_Q_OFFSET = kFloating ? 7 : 0;
    /// FreeFlyer 在 pinocchio v 向量中额外占用的维度 (vxyz + wxyz)
    static constexpr int BASE_V_OFFSET = kFloating ? 6 : 0;

    DynamicsSolver();
    DynamicsSolver(const DynamicsSolver&) = delete;
    DynamicsSolver& operator=(const DynamicsSolver&) = delete;

    // ── 运动学更新 ────────────────────────────────────────────────────────

    /**
     * @brief 更新正向运动学 (每控制周期调用一次)
     * @param arm_q    机械臂关节位置 (6 DoF, rad)
     * @param arm_v    机械臂关节速度 (6 DoF, rad/s)
     * @param gimbal_q 云台关节位置 (3 DoF, rad/m)
     *
     * 底盘姿态/速度通过 computeDynamics() 传入，不在此处更新，
     * 保证 FK (末端位姿、相机位姿) 始终在固定底盘假设下计算。
     */
    void updateKinematics(const common::VectorArm& arm_q,
                          const common::VectorArm& arm_v,
                          const common::VectorGimbal& gimbal_q);

    // ── 动力学计算 ────────────────────────────────────────────────────────

    /**
     * @brief 仅计算重力补偿力矩 (零底盘速度/加速度假设)
     *
     * 对两种模板实例均有效，行为与旧版 computeGravity() 完全一致。
     * @return 机械臂各关节重力补偿力矩 (6 DoF, Nm)
     */
    common::VectorArm computeGravity();

    /**
     * @brief 计算包含底盘运动惯性效应的完整逆动力学力矩
     *
     * 当 kFloating=false 时，直接调用 computeGravity() 返回，忽略底盘参数。
     * 当 kFloating=true 时，将底盘 IMU 数据代入浮动底盘 RNEA。
     *
     * @param chassis_acc   底盘 IMU 比力 (body frame, m/s²)
     *                      = a_real - g，静止水平时约为 [0, 0, +9.81]
     * @param chassis_omega 底盘角速度 (body frame, rad/s)
     * @param chassis_quat  底盘姿态四元数 (world → chassis)
     *                      传入 identity 时结果等同于 computeGravity()
     * @return 机械臂各关节补偿力矩 (6 DoF, Nm)
     */
    common::VectorArm computeDynamics(
        const Eigen::Vector3d& chassis_acc,
        const Eigen::Vector3d& chassis_omega,
        const Eigen::Quaterniond& chassis_quat);

    /**
     * @brief 计算完整逆动力学 (RNEA，含期望加速度前馈和末端外力)
     * @param acc_des          机械臂期望关节加速度 (6 DoF)
     * @param ext_wrench_world 末端外力旋量 (世界系, [Fx,Fy,Fz,Tx,Ty,Tz])
     * @return 机械臂各关节所需力矩 (6 DoF, Nm)
     */
    common::VectorArm computeRNEA(const common::VectorArm& acc_des,
                                   const common::Vector6& ext_wrench_world);

    // ── 逆运动学 ─────────────────────────────────────────────────────────

    /**
     * @brief 6-DoF 数值逆运动学 (DLS 迭代，带碰撞检测和随机重启)
     *
     * IK 求解时底盘始终视为静止 (origin + identity)，与底盘实际运动无关。
     * @return 成功: 关节角解; 失败: unexpected(最优近似解)
     */
    tl::expected<common::VectorArm, common::VectorArm> solveIK(
        const Eigen::Isometry3d& target_pose,
        const common::VectorArm& arm_q_guess,
        double tol = 1e-4,
        int max_iter = 100);

    /**
     * @brief 5-DoF 数值逆运动学 (位置 + 逼近方向，绕逼近轴自由旋转)
     *
     * 用于圆柱形矿石抓取：约束末端位置和末端 Z 轴方向，yaw 自由浮动。
     * @return 成功: 关节角解; 失败: unexpected(最优近似解)
     */
    tl::expected<common::VectorArm, common::VectorArm> solveIK5DoF(
        const Eigen::Vector3d& target_position,
        const Eigen::Vector3d& approach_direction,
        const common::VectorArm& arm_q_guess,
        double tol = 1e-4,
        int max_iter = 100);

    // ── 碰撞检测 ─────────────────────────────────────────────────────────

    /// 直线路径碰撞检测 (在两配置间均匀采样，使用当前云台状态)
    bool checkPathCollision(const common::VectorArm& arm_q_start,
                            const common::VectorArm& arm_q_goal,
                            int num_samples = 5);

    /// 启用/禁用 base_link 相关碰撞对 (存取矿时临时禁用)
    void setBaseLinkCollisionEnabled(bool enabled);

    // ── 末端负载设置 ─────────────────────────────────────────────────────

    /**
     * @brief 设置末端执行器负载质量 (用于动力学补偿)
     * @param mass 负载质量 (kg)，默认 0.0 表示无额外负载
     * 
     * 该方法会修改 gripper_tcp 帧所在线性 link_6 的惯性参数，
     * 在夹爪夹持物体时调用以增加 600g 负载的动力学补偿。
     */
    void setEndEffectorMass(double mass);

    // ── 位姿查询 ─────────────────────────────────────────────────────────

    /// 获取末端执行器 (gripper_tcp) 在基座系下的位姿
    Eigen::Isometry3d getEndEffectorPose() const;
    
    /// 计算给定关节位置的末端位姿 (用于 IK 误差验证)
    /// @return {position, rotation_matrix}
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> computeFK(const common::VectorArm& arm_q);

    /// 获取相机光心 (camera_optical_frame) 在基座系下的位姿
    Eigen::Isometry3d getCameraPose() const;

    /// 获取存取矿框架位姿 (index: 0 或 1)
    Eigen::Isometry3d getStoreFrame(size_t index) const;

    /// 将 PnP 结果 (相机系) 转换到基座坐标系
    Eigen::Isometry3d transformObjectToBase(const Eigen::Isometry3d& T_cam_obj) const;

    // ── 模型访问 ─────────────────────────────────────────────────────────
    pinocchio::Model&        getModel()       { return m_model; }
    pinocchio::Data&         getData()        { return m_data; }
    const pinocchio::GeometryModel& getGeometryModel() const { return m_geom_model; }
    const common::VectorGimbal& getCurrentGimbalQ() const { return m_current_gimbal_q; }

private:
    // ── 内部辅助函数 ─────────────────────────────────────────────────────

    /// 随机生成合法机械臂关节角 (IK 随机重启用)
    common::VectorArm generateRandomArmPositions();

    /**
     * @brief 构建完整 pinocchio 配置向量 (底盘 = origin + identity 旋转)
     * 固定基座: nq = JOINT_NUM (9)
     * 浮动基座: nq = JOINT_NUM + 7 (16), 底盘部分 = [0,0,0, 0,0,0,1]
     */
    Eigen::VectorXd buildFullQ(const common::VectorArm& arm_q,
                                const common::VectorGimbal& gimbal_q) const;

    /**
     * @brief 构建完整 pinocchio 配置向量 (底盘带指定姿态)
     * kFloating=false 时与无参版本等价，kFloating=true 时覆盖底盘旋转部分。
     */
    Eigen::VectorXd buildFullQ(const common::VectorArm& arm_q,
                                const common::VectorGimbal& gimbal_q,
                                const Eigen::Quaterniond& base_quat) const;

    /**
     * @brief 构建完整 pinocchio 速度向量 (nv 维)
     * 固定基座: nv = JOINT_NUM (9)
     * 浮动基座: nv = JOINT_NUM + 6 (15), 底盘部分 = [base_lin_vel; base_ang_vel]
     */
    Eigen::VectorXd buildFullV(const common::VectorArm& arm_v,
                                const common::VectorGimbal& gimbal_v,
                                const Eigen::Vector3d& base_lin_vel,
                                const Eigen::Vector3d& base_ang_vel) const;

    // ── Pinocchio 模型 ────────────────────────────────────────────────────
    pinocchio::Model         m_model;
    pinocchio::Data          m_data;
    pinocchio::GeometryModel m_geom_model;
    pinocchio::GeometryData  m_geom_data;

    // ── 关节索引缓存 (构造时计算，对固定/浮动两种模型均正确) ────────────────
    // 使用 m_model.joints[id].idx_q() / idx_v() 计算，避免硬编码 ARM_Q_INDICES
    std::array<int, common::ARM_JOINT_NUM>    m_arm_q_idx{};     // 配置空间 (q) 索引
    std::array<int, common::ARM_JOINT_NUM>    m_arm_v_idx{};     // 速度空间 (v) 索引 = Jacobian 列
    std::array<int, common::GIMBAL_JOINT_NUM> m_gimbal_q_idx{};
    std::array<int, common::GIMBAL_JOINT_NUM> m_gimbal_v_idx{};

    /// base_link 相关碰撞对索引缓存
    std::vector<size_t> m_base_link_pair_indices;

    // ── 当前运动状态 ──────────────────────────────────────────────────────
    common::VectorArm    m_current_arm_q   {common::VectorArm::Zero()};
    common::VectorArm    m_current_arm_v   {common::VectorArm::Zero()};
    common::VectorGimbal m_current_gimbal_q{common::VectorGimbal::Zero()};

    // ── 帧/关节索引 ───────────────────────────────────────────────────────
    pinocchio::JointIndex m_ee_joint_id    {static_cast<pinocchio::JointIndex>(-1)};
    pinocchio::FrameIndex m_tcp_frame_id   {static_cast<pinocchio::FrameIndex>(-1)};
    pinocchio::FrameIndex m_camera_frame_id{static_cast<pinocchio::FrameIndex>(-1)};
    pinocchio::FrameIndex m_store_frames[2]{
        static_cast<pinocchio::FrameIndex>(-1),
        static_cast<pinocchio::FrameIndex>(-1)
    };

    // ── 预分配缓冲区 (避免热路径中的堆分配) ─────────────────────────────
    pinocchio::container::aligned_vector<pinocchio::Force> f_ext_;

    /// 预分配 Jacobian 矩阵: 6 × m_model.nv
    /// 固定基座: 6×9; 浮动基座: 6×15
    Eigen::MatrixXd m_J_full;

    // ── 配置选项 ─────────────────────────────────────────────────────────
    bool m_mecanum_vel_comp{false}; ///< 从 solver.toml 读取

    std::shared_ptr<spdlog::logger> m_logger;
};

// ── 类型别名 ──────────────────────────────────────────────────────────────
using DynamicsSolverFixed    = DynamicsSolver<false>;
using DynamicsSolverFloating = DynamicsSolver<true>;

// ── 显式实例化声明 (抑制其他 TU 的隐式实例化) ─────────────────────────────
extern template class DynamicsSolver<false>;
extern template class DynamicsSolver<true>;

} // namespace yandy::modules

#endif // YANDY_ARM_DYNAMICSSOLVER_HPP

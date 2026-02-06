#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <vector>
#include <mutex>
#include <iomanip> // 用于 std::setprecision
#include <filesystem>
#include <algorithm>
#include <fstream>

#include <curses.h>
#include <rerun.hpp>

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/algorithm/geometry.hpp>

#include "yandy/core/Logger.hpp"
#include "yandy/modules/IArmHW.hpp"
#include "yandy/modules/DynamicsSolver.hpp"

using namespace yandy::modules;
using namespace yandy::common;

// 虚拟硬件层 (FakeArmHW)
class FakeArmHW : public IArmHW
{
public:
    FakeArmHW(const std::string& urdf_path)
    {
        pinocchio::urdf::buildModel(urdf_path, m_sim_model);
        m_sim_data = pinocchio::Data(m_sim_model);

        m_state.q = VectorJ::Zero();
        m_state.v = VectorJ::Zero();
        m_state.tau = VectorJ::Zero();

        // 初始姿态：避开奇异点，同时落在关节限位内
        m_state.q << 0.0, 0.6, -1.5, 0.0, 0.8;

        // 初始化 last_cmd，避免第一帧飞车
        m_last_cmd.q_des = m_state.q;
        m_last_cmd.kp.fill(20.0); // 提高刚度
        m_last_cmd.kd.fill(1.0);
    }

    void read(JointState& out_state) override
    {
        out_state = m_state;
    }

    void write(const JointCommand& cmd) override
    {
        m_last_cmd = cmd;
    }

    void step(double dt)
    {
        // 计算重力 (G)
        VectorJ tau_grav = pinocchio::rnea(m_sim_model, m_sim_data, m_state.q, VectorJ::Zero(), VectorJ::Zero());

        // 计算电机力矩 (MIT Mode)
        VectorJ tau_motor = m_last_cmd.kp.cwiseProduct(m_last_cmd.q_des - m_state.q) +
            m_last_cmd.kd.cwiseProduct(m_last_cmd.v_des - m_state.v) +
            m_last_cmd.tau_ff;

        // 简化的物理积分 (F = ma)
        // 减小模拟惯量，让它响应更快
        VectorJ inertia;
        inertia.fill(0.2);

        VectorJ damping;
        damping.fill(0.5); // 稍微加点阻尼防止震荡

        const VectorJ acc = (tau_motor - tau_grav - damping.cwiseProduct(m_state.v)).cwiseQuotient(inertia);

        // 半隐式欧拉积分
        m_state.v += acc * dt;
        m_state.q += m_state.v * dt;
        m_state.tau = tau_motor;
    }

    void enable() override
    {
    }

    void disable() override
    {
    }

private:
    pinocchio::Model m_sim_model;
    pinocchio::Data m_sim_data;
    JointState m_state;
    JointCommand m_last_cmd;
};

// 输入处理
struct InputState
{
    Eigen::Vector3d target_pos;
    Eigen::Vector3d target_rpy;
    bool running = true;
    bool ik_success = false; // 增加状态标志
};

void handle_input(InputState& input)
{
    int ch = getch();
    if (ch == ERR) return;

    // 加大一点步长，让你能更快看到效果
    constexpr double move_step = 0.02;
    constexpr double rot_step = 0.05;

    switch (ch)
    {
    case 'w': input.target_pos.x() += move_step;
        break;
    case 's': input.target_pos.x() -= move_step;
        break;
    case 'a': input.target_pos.y() += move_step;
        break;
    case 'd': input.target_pos.y() -= move_step;
        break;
    case 'q': input.target_pos.z() += move_step;
        break;
    case 'e': input.target_pos.z() -= move_step;
        break;

    case 'i': input.target_rpy.y() += rot_step;
        break;
    case 'k': input.target_rpy.y() -= rot_step;
        break;
    case 'j': input.target_rpy.z() += rot_step;
        break;
    case 'l': input.target_rpy.z() -= rot_step;
        break;
    case 'u': input.target_rpy.x() += rot_step;
        break;
    case 'o': input.target_rpy.x() -= rot_step;
        break;

    case 27:
    case 'x': input.running = false;
        break;
    default: ;
    }
}

// ==========================================
// 3. 主程序
// ==========================================
int main(int argc, char** argv)
{
    yandy::core::init_logging();

    const std::string urdf_file = YANDY_CONFIG_PATH "urdf/yandy_urdf.urdf";
    constexpr double dt = 0.004; // 250Hz

    // --- Init Ncurses ---
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    timeout(0);
    curs_set(0);

    // --- Init Rerun ---
    auto rec = rerun::RecordingStream("robomaster_arm_sim");
    rec.spawn().exit_on_failure();

    // 设置世界坐标系为 Z-Up
    rec.log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);

    // --- Init Robot ---
    FakeArmHW hw(urdf_file);
    DynamicsSolver solver;

    // 加载几何模型 (专门用于管理 Visual Mesh)
    pinocchio::GeometryModel geom_model;
    pinocchio::urdf::buildGeom(solver.getModel(), urdf_file, pinocchio::VISUAL, geom_model,
                               std::filesystem::path(urdf_file).parent_path().string());
    pinocchio::GeometryData geom_data(geom_model);

    const auto urdf_dir = std::filesystem::path(urdf_file).parent_path();

    // 将可能的相对路径改为绝对路径
    const auto resolve_mesh_path = [&](const std::string& raw_path) -> std::filesystem::path
    {
        if (raw_path.empty())
        {
            return {};
        }
        std::filesystem::path path = raw_path;
        if (path.is_absolute())
        {
            return path;
        }
        return urdf_dir / path;
    };

    // 预先在 Rerun 中注册所有 Mesh 文件
    const std::string viz_root = "world/robot";
    for (const auto& geom : geom_model.geometryObjects)
    {
        // 构造唯一的实体路径，例如 "world/robot/link_1_0"
        // 直接用 geom.name，Pinocchio 生成什么就用什么
        std::string entity_path = viz_root + "/" + geom.name;

        // 获取 Mesh 文件的绝对路径
        const auto mesh_path = resolve_mesh_path(geom.meshPath);

        // 让 Rerun 加载 3D 模型
        // 文件是否存在
        if (mesh_path.empty() || !std::filesystem::exists(mesh_path))
        {
            throw std::runtime_error("Mesh file not found: " + mesh_path.string());
        }
        // 文件是否能打开
        std::ifstream file(mesh_path, std::ios::binary);
        if (!file)
        {
            throw std::runtime_error("Failed to open mesh file: " + mesh_path.string());
        }
        // 文件是否为空
        std::vector<uint8_t> bytes((std::istreambuf_iterator(file)),
                                   std::istreambuf_iterator<char>());
        if (bytes.empty())
        {
            throw std::runtime_error("Mesh file is empty: " + mesh_path.string());
        }
        std::string ext = mesh_path.extension().string();
        std::ranges::transform(ext, ext.begin(), [](const unsigned char c)
        {
            return static_cast<char>(std::tolower(c));
        });

        auto asset = rerun::Asset3D::from_file_contents(
            bytes,
            ext == ".stl"
                ? std::optional(rerun::components::MediaType::stl())
                : std::optional<rerun::components::MediaType>{}
        );
        auto err = rec.try_log_static(entity_path, asset);
        if (err.is_err())
        {
            throw std::runtime_error("Failed to log Asset3D: " + err.description);
        }
    }

    InputState input;

    // 获取初始位置，把目标点设在当前手所在的位置，防止一上来就飞车
    JointState init_state;
    hw.read(init_state);

    // 用初始位置先解算一遍
    solver.updateKinematics(init_state.q, init_state.v);
    auto init_pose = solver.getEndEffectorPose();

    // 解算出来的值作为输入系统的起始值
    input.target_pos = init_pose.translation();
    input.target_rpy = init_pose.rotation().eulerAngles(0, 1, 2);

    JointState current_state;
    JointCommand cmd;
    cmd.q_des = init_state.q;
    cmd.v_des.setZero();
    cmd.kp.fill(20.0);
    cmd.kd.fill(1.0);

    while (input.running)
    {
        auto loop_start = std::chrono::steady_clock::now();
        handle_input(input);

        // --- 物理 & 算法更新 ---
        hw.read(current_state);
        solver.updateKinematics(current_state.q, current_state.v);
        auto pose_ee = solver.getEndEffectorPose(); // 当前实际位姿

        // --- 可视化---

        // A. 更新 Pinocchio 的几何体位置
        // 它会计算 result = oMf * geom_placement
        // 自动把 URDF 里的 <origin> 偏移加上
        pinocchio::updateGeometryPlacements(solver.getModel(), solver.getData(), geom_model, geom_data,
                                            current_state.q);

        // B. 遍历所有几何体并发送给 Rerun
        for (pinocchio::GeomIndex i = 0; i < geom_model.ngeoms; ++i)
        {
            const auto& geom = geom_model.geometryObjects[i];
            const auto& oMg = geom_data.oMg[i]; // 绝对位姿

            // 必须使用和 Init 阶段完全一样的路径字符串！
            std::string entity_path = viz_root + "/" + geom.name;

            const auto& t = oMg.translation();
            const auto& r = oMg.rotation();
            const Eigen::Quaterniond q(r);

            // 3. Log Transform
            // Rerun 会把这个 Transform 应用到之前加载的 Asset3D 上
            rec.log(entity_path,
                    rerun::Transform3D::from_translation_rotation(
                        rerun::components::Translation3D((float)t.x(), (float)t.y(), (float)t.z()),
                        rerun::Rotation3D(rerun::Quaternion::from_xyzw(
                            (float)q.x(), (float)q.y(), (float)q.z(), (float)q.w()))
                    )
            );
        }

        // Log 目标点 (Ghost)
        Eigen::Quaterniond q_target =
            Eigen::AngleAxisd(input.target_rpy.x(), Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(input.target_rpy.y(), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(input.target_rpy.z(), Eigen::Vector3d::UnitZ());

        Eigen::Isometry3d target_iso = Eigen::Isometry3d::Identity();
        target_iso.pretranslate(input.target_pos);
        target_iso.rotate(q_target);

        // 目标点用绿色球显示
        rec.log("world/target_ghost",
                rerun::Transform3D(
                    {(float)input.target_pos.x(), (float)input.target_pos.y(), (float)input.target_pos.z()},
                    rerun::Quaternion::from_xyzw(q_target.x(), q_target.y(), q_target.z(), q_target.w())
                )
        );
        rec.log("world/target_ghost",
                rerun::Points3D({{0.0f, 0.0f, 0.0f}}).with_radii({0.05f}).with_colors({0x00FF00FF})
        );

        // Log 实际末端点 (红色球)
        rec.log("world/ee_actual",
                rerun::Points3D({
                    {
                        (float)pose_ee.translation().x(), (float)pose_ee.translation().y(),
                        (float)pose_ee.translation().z()
                    }
                }).with_radii({0.03f}).with_colors({0xFF0000FF})
        );

        // --- 3. 控制解算 ---
        // 补上重力补偿
        cmd.tau_ff = solver.computeGravity();

        // 传递 current_state.q 作为初始猜测，放宽 IK 求解条件以提高成功率
        auto q_sol = solver.solveIK(target_iso, current_state.q, 1e-4, 100);

        if (q_sol.has_value())
        {
            input.ik_success = true;
            cmd.q_des = q_sol.value();
            // 简单的速度前馈
            cmd.v_des = (cmd.q_des - current_state.q) / dt;
            // 速度限幅 (Safety)
            cmd.v_des = cmd.v_des.cwiseMin(5.0).cwiseMax(-5.0);
        }
        else
        {
            input.ik_success = false;
            // IK 失败时，保持当前位置，或者仅依靠重力补偿
            cmd.q_des = current_state.q;
            cmd.v_des.setZero();
        }


        // 发送给虚拟硬件
        hw.write(cmd);
        hw.step(dt); // 物理步进

        // --- 终端 UI 更新 ---
        // 使用 \r 刷新行
        mvprintw(0, 0, "=== SIMULATION RUNNING ===");
        mvprintw(2, 0, "Target X: %.3f Y: %.3f Z: %.3f",
                 input.target_pos.x(), input.target_pos.y(), input.target_pos.z());
        mvprintw(3, 0, "Actual X: %.3f Y: %.3f Z: %.3f",
                 pose_ee.translation().x(), pose_ee.translation().y(), pose_ee.translation().z());
        mvprintw(5, 0, "Target R: %.3f P: %.3f Y: %.3f",
                 input.target_rpy.x(), input.target_rpy.y(), input.target_rpy.z());
        Eigen::Vector3d actual_rpy = pose_ee.rotation().eulerAngles(0, 1, 2);
        mvprintw(6, 0, "Actual R: %.3f P: %.3f Y: %.3f",
                 actual_rpy.x(), actual_rpy.y(), actual_rpy.z());

        mvprintw(8, 0, "IK Status: %s", input.ik_success ? "[OK]" : "[FAIL]");
        mvprintw(9, 0, "Motor Tau: %.2f %.2f %.2f %.2f %.2f",
                 current_state.tau(0), current_state.tau(1), current_state.tau(2), current_state.tau(3),
                 current_state.tau(4));

        refresh();
        std::this_thread::sleep_until(loop_start + std::chrono::duration<double>(dt));
    }

    endwin();
    return 0;
}

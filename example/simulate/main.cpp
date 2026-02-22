#include <thread>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <csignal>
#include <atomic>

#include <rerun.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/algorithm/geometry.hpp>

#include <yandy/Robot.hpp>

using namespace yandy::common;

static std::atomic<bool> g_running{true};

static void signal_handler(int) { g_running.store(false, std::memory_order_release); }

// 加载 STL/mesh 文件并注册到 Rerun
static void register_asset(rerun::RecordingStream& rec, const std::string& entity_path,
                           const std::filesystem::path& mesh_path)
{
    if (mesh_path.empty() || !std::filesystem::exists(mesh_path))
        throw std::runtime_error("Mesh file not found: " + mesh_path.string());

    std::ifstream file(mesh_path, std::ios::binary);
    if (!file)
        throw std::runtime_error("Failed to open mesh file: " + mesh_path.string());

    std::vector<uint8_t> bytes((std::istreambuf_iterator(file)),
                               std::istreambuf_iterator<char>());
    if (bytes.empty())
        throw std::runtime_error("Mesh file is empty: " + mesh_path.string());

    std::string ext = mesh_path.extension().string();
    std::ranges::transform(ext, ext.begin(), [](unsigned char c)
    {
        return static_cast<char>(std::tolower(c));
    });

    auto asset = rerun::Asset3D::from_file_contents(
        bytes,
        ext == ".stl"
            ? std::optional(rerun::components::MediaType::stl())
            : std::optional<rerun::components::MediaType>{});
    auto err = rec.try_log_static(entity_path, asset);
    if (err.is_err())
        throw std::runtime_error("Failed to log Asset3D: " + err.description);
}

int main()
{
    yandy::core::init_logging();
    auto diag_logger = yandy::core::create_logger("SimViz", spdlog::level::info);
    std::signal(SIGINT, signal_handler);

    const std::string urdf_file = YANDY_CONFIG_PATH "urdf/yandy_urdf.urdf";
    const std::string energy_unit_stl = YANDY_CONFIG_PATH "urdf/meshes/energy_unit.stl";
    constexpr double viz_dt = 0.016; // ~60Hz 可视化刷新率

    // ---- Init Rerun ----
    auto rec = rerun::RecordingStream("robomaster_arm_sim");
    rec.spawn().exit_on_failure();
    rec.log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Z_UP);

    // ---- 加载几何模型 (独立于 Robot，线程安全) ----
    pinocchio::Model viz_model;
    pinocchio::urdf::buildModel(urdf_file, viz_model);
    pinocchio::Data viz_data(viz_model);

    pinocchio::GeometryModel geom_model;
    pinocchio::urdf::buildGeom(viz_model, urdf_file, pinocchio::VISUAL, geom_model,
                               std::filesystem::path(urdf_file).parent_path().string());
    pinocchio::GeometryData geom_data(geom_model);

    const auto urdf_dir = std::filesystem::path(urdf_file).parent_path();
    const auto resolve_mesh_path = [&](const std::string& raw_path) -> std::filesystem::path
    {
        if (raw_path.empty()) return {};
        std::filesystem::path path = raw_path;
        return path.is_absolute() ? path : urdf_dir / path;
    };

    // 注册机器人 mesh 资产
    const std::string viz_root = "world/robot";
    for (const auto& geom : geom_model.geometryObjects)
    {
        register_asset(rec, viz_root + "/" + geom.name, resolve_mesh_path(geom.meshPath));
    }

    // 注册能量单元 mesh 资产
    register_asset(rec, "world/energy_unit", energy_unit_stl);

    // 注册静态形状 (只需发送一次，后续只更新 Transform)
    rec.log_static("world/target_ghost",
                   rerun::Points3D({{0.0f, 0.0f, 0.0f}}).with_radii({0.05f}).with_colors({0x00FF00FF}));
    rec.log_static("world/ee_actual",
                   rerun::Points3D({{0.0f, 0.0f, 0.0f}}).with_radii({0.03f}).with_colors({0xFF0000FF}));

    // ---- 启动 Robot (后台线程) ----
    yandy::Robot robot;
    std::thread robot_thread([&robot] { robot.start(); });

    // ---- 可视化主循环 ----
    int frame_count = 0;
    while (g_running.load(std::memory_order_relaxed))
    {
        const auto loop_start = std::chrono::steady_clock::now();

        auto vd = robot.vizBuf().read();

        const auto t0 = std::chrono::steady_clock::now();

        // A. 更新机器人几何体位姿
        pinocchio::updateGeometryPlacements(viz_model, viz_data, geom_model, geom_data, vd.q);

        for (pinocchio::GeomIndex i = 0; i < geom_model.ngeoms; ++i)
        {
            const auto& geom = geom_model.geometryObjects[i];
            const auto& oMg = geom_data.oMg[i];

            const auto& t = oMg.translation();
            const Eigen::Quaterniond q(oMg.rotation());

            rec.log_static(viz_root + "/" + geom.name,
                           rerun::Transform3D::from_translation_rotation(
                               rerun::components::Translation3D((float)t.x(), (float)t.y(), (float)t.z()),
                               rerun::Rotation3D(rerun::Quaternion::from_xyzw(
                                   (float)q.x(), (float)q.y(), (float)q.z(), (float)q.w()))));
        }

        const auto t1 = std::chrono::steady_clock::now();

        // B. 目标点 (绿色球) — 只更新 Transform，形状已 log_static
        {
            const auto& tp = vd.target_pose;
            const auto& t = tp.translation();
            const Eigen::Quaterniond q(tp.rotation());

            rec.log_static("world/target_ghost",
                           rerun::Transform3D(
                               {(float)t.x(), (float)t.y(), (float)t.z()},
                               rerun::Quaternion::from_xyzw(q.x(), q.y(), q.z(), q.w())));
        }

        // C. 实际末端点 (红色球) — 只更新 Transform
        {
            const auto& t = vd.ee_pose.translation();
            rec.log_static("world/ee_actual",
                           rerun::Transform3D(
                               rerun::components::Translation3D((float)t.x(), (float)t.y(), (float)t.z())));
        }

        // D. 视觉检测到的能量单元
        if (vd.vision_valid)
        {
            const auto& vp = vd.vision_unit_pose_base;
            const auto& t = vp.translation();
            const Eigen::Quaterniond q(vp.rotation());

            rec.log_static("world/energy_unit",
                           rerun::Transform3D::from_translation_rotation_scale(
                               rerun::components::Translation3D((float)t.x(), (float)t.y(), (float)t.z()),
                               rerun::Rotation3D(rerun::Quaternion::from_xyzw(
                                   (float)q.x(), (float)q.y(), (float)q.z(), (float)q.w())),
                               rerun::components::Scale3D(0.001f)));
        }

        // E. 状态文本 (TextDocument 替换式更新，不累加)
        rec.log_static("state", rerun::TextDocument(std::string(format_as(vd.state))));

        // 输出 target 位姿的数值（XYZ + RPY in degrees）
        {
            const auto& tp = vd.target_pose;
            const auto t = tp.translation();
            const Eigen::Matrix3d R = tp.rotation();
            Eigen::Vector3d rpy = R.eulerAngles(0, 1, 2); // roll, pitch, yaw (radians)
            constexpr double rad2deg = 180.0 / M_PI;
            double roll_deg = rpy[0] * rad2deg;
            double pitch_deg = rpy[1] * rad2deg;
            double yaw_deg = rpy[2] * rad2deg;

            diag_logger->info("current target xyz:{:.3f} {:.3f} {:.3f}, rpy:{:.1f} {:.1f} {:.1f}",
                              t.x(), t.y(), t.z(), roll_deg, pitch_deg, yaw_deg);
        }

        std::this_thread::sleep_until(loop_start + std::chrono::duration<double>(viz_dt));
    }

    robot.stop();
    robot_thread.join();
    return 0;
}

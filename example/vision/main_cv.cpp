#include <thread>
#include <yandy/modules/VisionSystem.hpp>
#include <yandy/core/Logger.hpp>

// --- 辅助工具：将 Eigen 转换为 OpenCV 格式以便绘图 ---
void eigen2cv(const Eigen::Isometry3d& pose, cv::Mat& rvec, cv::Mat& tvec)
{
    Eigen::Matrix3d R = pose.rotation();
    Eigen::Vector3d t = pose.translation();

    cv::Mat R_cv(3, 3, CV_64F);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            R_cv.at<double>(i, j) = R(i, j);

    cv::Rodrigues(R_cv, rvec); // 旋转矩阵转旋转向量
    tvec = (cv::Mat_<double>(3, 1) << t(0), t(1), t(2));
}

// --- 辅助工具：在画布上画出 3D 圆柱体和坐标轴 ---
void draw3DVisualizer(cv::Mat& img, const Eigen::Isometry3d& pose, const cv::Mat& K, const cv::Mat& D)
{
    cv::Mat rvec, tvec;
    eigen2cv(pose, rvec, tvec);

    // 1. 定义 3D 模型点 (标准单位: 米)
    // 根据你的定义，物体是一个宽60mm(半径30mm)，高75mm的物体
    std::vector<cv::Point3f> object_pts;
    // A. 画坐标轴 (Axis) - 长度 0.05m
    object_pts.push_back(cv::Point3f(0, 0, 0)); // 原点
    object_pts.push_back(cv::Point3f(0.05, 0, 0)); // X轴 (红)
    object_pts.push_back(cv::Point3f(0, 0.05, 0)); // Y轴 (绿)
    object_pts.push_back(cv::Point3f(0, 0, 0.05)); // Z轴 (蓝)

    // B. 画圆柱体 (简化为两个八边形 + 连线)
    float r = 0.030f; // 半径 30mm
    float h = 0.0375f; // 半高 37.5mm (总高75)
    // 你的PnP定义里，TL=(-30, -37.5), BR=(30, 37.5)，说明原点在几何中心

    // 生成上底面 (z=0, 实际上根据你的定义是平面物体，但为了好看我们画个立体的)
    // 假设你的能量单元有一定的厚度，或者我们只是为了可视化其朝向
    // 这里我们画你定义的那 4 个角点形成的矩形
    object_pts.push_back(cv::Point3f(-r, -h, 0)); // TL
    object_pts.push_back(cv::Point3f(r, -h, 0)); // TR
    object_pts.push_back(cv::Point3f(r, h, 0)); // BR
    object_pts.push_back(cv::Point3f(-r, h, 0)); // BL

    // 2. 投影到 2D 像素坐标
    std::vector<cv::Point2f> image_pts;
    cv::projectPoints(object_pts, rvec, tvec, K, D, image_pts);

    // 3. 绘制
    // A. 坐标轴
    cv::line(img, image_pts[0], image_pts[1], cv::Scalar(0, 0, 255), 3); // X Red
    cv::line(img, image_pts[0], image_pts[2], cv::Scalar(0, 255, 0), 3); // Y Green
    cv::line(img, image_pts[0], image_pts[3], cv::Scalar(255, 0, 0), 3); // Z Blue

    // B. 矩形框 (模拟能量单元面板)
    // index 4,5,6,7 对应 TL, TR, BR, BL
    for (int i = 0; i < 4; i++)
    {
        cv::line(img, image_pts[4 + i], image_pts[4 + (i + 1) % 4], cv::Scalar(0, 255, 255), 2);
    }

    // 标出原点
    cv::circle(img, image_pts[0], 3, cv::Scalar(255, 255, 255), -1);

    // 显示坐标数值 (XYZ)
    char text[100];
    sprintf(text, "X:%.2f Y:%.2f Z:%.2f", tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    cv::putText(img, text, cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 1);
}

using namespace std::chrono_literals;

int main()
{
    yandy::core::init_logging();
    yandy::modules::HikDriver camera;
    if (!camera.init()) return -1;

    yandy::modules::EnergyDetector detector;
    if (!detector.init()) return -1;

    cv::Mat vis_3d_canvas; // 用于可视化的画布
    cv::Mat combined_display; // 最终合并的图
    cv::Mat frame;

    yandy::modules::EnergyPoseSolver solver;

    auto K = solver.get_camera_matrix();
    auto D = solver.get_dist_coeffs();

    while (true)
    {
        // 2. 读图
        if (camera.getLatestFrame(frame))
        {
            // 3. 推理
            auto start = std::chrono::high_resolution_clock::now();
            auto results = detector.detect(frame);
            vis_3d_canvas = cv::Mat::zeros(frame.size(), CV_8UC3);
            if (!results.empty())
            {
                const auto& target = results[0];

                // --- 可视化左侧：YOLO 关键点 ---
                detector.drawResults(frame, results);

                // --- PnP 解算 ---
                Eigen::Isometry3d T_cam_obj;
                bool pnp_ok = solver.solve(target, T_cam_obj);

                if (pnp_ok)
                {
                    // --- 可视化右侧：3D 重投影 ---
                    // 在纯黑背景上画出“理想”的 3D 线框
                    // 如果 PnP 算得准，这个线框应该看起来和左图物体的朝向一致
                    draw3DVisualizer(vis_3d_canvas, T_cam_obj, K, D);

                    // 可选：在左图上也画坐标轴 (AR效果)
                    draw3DVisualizer(frame, T_cam_obj, K, D);
                }
            }


            auto end = std::chrono::high_resolution_clock::now();

            // 计算耗时
            const double latency = std::chrono::duration<double, std::milli>(end - start).count();
            cv::putText(frame, std::to_string(latency) + "ms", cv::Point(10, 30), 0, 1, cv::Scalar(0, 255, 0));

            // --- 拼接图像 ---
            // 左边是原图+检测，右边是纯黑背景+3D线框
            cv::hconcat(frame, vis_3d_canvas, combined_display);

            cv::Mat display_resized;
            cv::resize(combined_display, display_resized, cv::Size(), 0.8, 0.8);

            cv::imshow("Energy Unit Detect", display_resized);
        }

        if (cv::waitKey(1) == 27) break; // ESC 退出
    }
}

#include <iostream>
#include <yandy/modules/VisionSystem.hpp>
#include <yandy/core/Logger.hpp>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Eigen>

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

// --- 辅助工具：在画布上画出坐标轴 ---
void drawAxes(cv::Mat& img, const Eigen::Isometry3d& pose, const cv::Mat& K, const cv::Mat& D)
{
    cv::Mat rvec, tvec;
    eigen2cv(pose, rvec, tvec);

    // 1. 定义 3D 模型点 (标准单位: 米)
    std::vector<cv::Point3f> object_pts;
    // A. 画坐标轴 (Axis) - 长度 0.05m
    object_pts.emplace_back(0, 0, 0); // 原点
    object_pts.emplace_back(0.05, 0, 0); // X轴 (红)
    object_pts.emplace_back(0, 0.05, 0); // Y轴 (绿)
    object_pts.emplace_back(0, 0, 0.05); // Z轴 (蓝)

    // 2. 投影到 2D 像素坐标
    std::vector<cv::Point2f> image_pts;
    cv::projectPoints(object_pts, rvec, tvec, K, D, image_pts);

    // 3. 绘制
    cv::line(img, image_pts[0], image_pts[1], cv::Scalar(0, 0, 255), 3); // X Red
    cv::line(img, image_pts[0], image_pts[2], cv::Scalar(0, 255, 0), 3); // Y Green
    cv::line(img, image_pts[0], image_pts[3], cv::Scalar(255, 0, 0), 3); // Z Blue
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <video_path>" << std::endl;
        return -1;
    }

    std::string input_path = argv[1];
    std::string output_path = input_path + "_output.mp4";

    yandy::core::init_logging();

    yandy::modules::EnergyDetector detector;
    if (!detector.init()) return -1;

    yandy::modules::EnergyPoseSolver solver;
    auto K = solver.get_camera_matrix();
    auto D = solver.get_dist_coeffs();

    cv::VideoCapture cap(input_path);
    if (!cap.isOpened())
    {
        std::cerr << "Failed to open video: " << input_path << std::endl;
        return -1;
    }

    int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    double fps = cap.get(cv::CAP_PROP_FPS);
    int total_frames = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));

    cv::VideoWriter writer(output_path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps, cv::Size(width, height));
    if (!writer.isOpened())
    {
        std::cerr << "Failed to open video writer: " << output_path << std::endl;
        return -1;
    }

    cv::Mat frame;
    int frame_count = 0;
    while (cap.read(frame))
    {
        auto results = detector.detect(frame);

        if (!results.empty())
        {
            // 标记框、关键点、置信度
            detector.drawResults(frame, results);

            for (const auto& target : results)
            {
                // 坐标轴
                Eigen::Isometry3d T_cam_obj;
                if (solver.solve(target, T_cam_obj))
                {
                    drawAxes(frame, T_cam_obj, K, D);
                }
            }
        }

        writer.write(frame);
        frame_count++;
        if (frame_count % 10 == 0)
        {
            std::cout << "\rProcessing frame " << frame_count << " / " << total_frames << std::flush;
        }
    }
    std::cout << "\nDone. Output saved to: " << output_path << std::endl;

    return 0;
}


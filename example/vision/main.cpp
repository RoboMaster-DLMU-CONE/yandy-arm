#include <thread>
#include <yandy/modules/VisionSystem.hpp>
#include <yandy/core/Logger.hpp>
using namespace std::chrono_literals;

int main()
{
    yandy::core::init_logging();
    yandy::module::HikDriver camera;
    if (!camera.init()) return -1;

    yandy::module::EnergyDetector detector;
    if (!detector.init()) return -1;

    cv::Mat frame;

    while (true)
    {
        // 2. 读图
        if (camera.getFrame(frame))
        {
            // 3. 推理
            auto start = std::chrono::high_resolution_clock::now();
            auto results = detector.detect(frame);
            auto end = std::chrono::high_resolution_clock::now();

            // 计算耗时
            const double latency = std::chrono::duration<double, std::milli>(end - start).count();

            // 4. 导出关键点给位姿解算 (Solver)
            if (!results.empty())
            {
                // 取置信度最高的一个
                const auto& target = results[0];

                // 将 target.keypoints (vector<Point3f>) 传递给你的 PnP Solver
                // solver.solve(target.keypoints);
                // 5. 可视化
                detector.drawResults(frame, results);
            }

            cv::putText(frame, std::to_string(latency) + "ms", cv::Point(10, 30), 0, 1, cv::Scalar(0, 255, 0));
            cv::imshow("Energy Detector", frame);
        }

        if (cv::waitKey(1) == 27) break; // ESC 退出
    }
}

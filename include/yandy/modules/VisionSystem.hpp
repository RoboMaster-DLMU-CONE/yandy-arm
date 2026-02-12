#ifndef YANDY_ARM_VISIONSYSTEM_HPP
#define YANDY_ARM_VISIONSYSTEM_HPP

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include "MvCameraControl.h"
#include <vector>
#include <iostream>
#include <spdlog/logger.h>
#include <eigen3/Eigen/Eigen>

namespace yandy::module
{
    struct EnergyUnit
    {
        cv::Rect box; // 包围盒
        float confidence; // 置信度
        int class_id; // 类别
        // 6个关键点: [x, y, conf]
        // 0:上左, 1:上中, 2:上右, 3:下左, 4:下中, 5:下右
        std::vector<cv::Point3f> keypoints;
    };

    class EnergyPoseSolver
    {
    public:
        // 构造函数：传入相机内参
        // camera_matrix: 3x3 内参矩阵
        // dist_coeffs: 畸变系数 (D)
        EnergyPoseSolver(std::optional<cv::Mat> camera_matrix = std::nullopt,
                         std::optional<cv::Mat> dist_coeffs = std::nullopt);

        // 核心解算函数
        // 输入: 识别到的能量单元
        // 输出: 4x4 变换矩阵 (Eigen::Isometry3d)，代表物体在相机坐标系下的位姿
        // 返回: 是否解算成功
        bool solve(const EnergyUnit& unit, Eigen::Isometry3d& output_pose) const;

    private:
        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;

        // 3D 物体坐标 (世界坐标系，以能量单元中心为原点)
        std::vector<cv::Point3f> object_points_;

        std::shared_ptr<spdlog::logger> m_logger;
    };

    class HikDriver
    {
    public:
        HikDriver();
        ~HikDriver();

        bool init();
        bool getFrame(cv::Mat& frame) const;
        void close();

    private:
        void* handle_ = nullptr;
        unsigned char* pData_ = nullptr; // 原始数据缓存
        unsigned char* pDataForRGB_ = nullptr; // RGB转换缓存
        MV_CC_DEVICE_INFO_LIST stDeviceList_{};
        bool is_open_ = false;
        int payload_size_ = 0;
        std::shared_ptr<spdlog::logger> m_logger;
    };

    class EnergyDetector
    {
    public:
        EnergyDetector();
        bool init();

        // 推理函数
        std::vector<EnergyUnit> detect(const cv::Mat& raw_img);

        // 绘制结果 (用于调试/显示)
        void drawResults(cv::Mat& img, const std::vector<EnergyUnit>& results);

    private:
        float scale_factor_ = 1.0f;
        cv::Point2f pad_offset_ = {0, 0};
        ov::Core core_;
        ov::CompiledModel compiled_model_;
        ov::InferRequest infer_request_;

        float conf_threshold_{};
        cv::Size model_input_shape_ = cv::Size(416, 416);

        // 预处理辅助
        cv::Mat letterbox(const cv::Mat& source);
        std::shared_ptr<spdlog::logger> m_logger;
    };
}

#endif //YANDY_ARM_VISIONSYSTEM_HPP

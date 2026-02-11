#ifndef YANDY_ARM_VISIONSYSTEM_HPP
#define YANDY_ARM_VISIONSYSTEM_HPP

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include "MvCameraControl.h"
#include <vector>
#include <iostream>
#include <spdlog/logger.h>

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
        cv::Size model_input_shape_ = cv::Size(416, 416); // 根据你的ONNX截图

        // 预处理辅助
        cv::Mat letterbox(const cv::Mat& source);
        std::shared_ptr<spdlog::logger> m_logger;
    };
}

#endif //YANDY_ARM_VISIONSYSTEM_HPP

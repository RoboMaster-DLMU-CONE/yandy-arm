#include <yandy/modules/VisionSystem.hpp>
#include <yandy/core/Logger.hpp>
#include <toml++/toml.hpp>

#define YANDY_MODEL_PATH YANDY_CONFIG_PATH "yolo/best.onnx"
#define YANDY_VISION_CONFIG YANDY_CONFIG_PATH "vision.toml"

namespace yandy::module
{
    HikDriver::HikDriver()
    {
        m_logger = core::create_logger("HikDriver", spdlog::level::info);
        m_logger->info("HikDriver constructed.");
    }

    HikDriver::~HikDriver()
    {
    }

    bool HikDriver::init()
    {
        int nRet = MV_OK;
        // 枚举设备
        nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList_);
        if (MV_OK != nRet || stDeviceList_.nDeviceNum == 0)
        {
            m_logger->error("No camera found!");
            return false;
        }
        // 选择第一个设备创建句柄
        nRet = MV_CC_CreateHandle(&handle_, stDeviceList_.pDeviceInfo[0]);
        if (MV_OK != nRet) return false;

        // 打开设备
        nRet = MV_CC_OpenDevice(handle_);
        if (MV_OK != nRet) return false;

        // 获取 Payload Size (用于申请内存)
        MVCC_INTVALUE stParam{};
        nRet = MV_CC_GetIntValue(handle_, "PayloadSize", &stParam);
        if (MV_OK != nRet) return false;
        payload_size_ = stParam.nCurValue;

        // 申请缓存
        pData_ = static_cast<unsigned char*>(malloc(sizeof(unsigned char) * payload_size_));
        // 预估RGB缓存大小 (最大分辨率 x 3)
        pDataForRGB_ = static_cast<unsigned char*>(malloc(sizeof(unsigned char) * payload_size_ * 3));

        // 开始取流
        nRet = MV_CC_StartGrabbing(handle_);
        if (MV_OK != nRet) return false;

        is_open_ = true;
        m_logger->info("Camera initialized successfully.");
        return true;
    }

    bool HikDriver::getFrame(cv::Mat& frame) const
    {
        if (!is_open_) return false;

        MV_FRAME_OUT_INFO_EX stImageInfo = {0};
        memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));

        // 获取一帧 (超时时间 1000ms)
        int nRet = MV_CC_GetOneFrameTimeout(handle_, pData_, payload_size_, &stImageInfo, 1000);
        if (nRet != MV_OK)
        {
            m_logger->error("Get frame failed: {}", nRet);
            return false;
        }

        // 像素格式转换 (转为 BGR 给 OpenCV 使用)
        // 注意：海康相机通常输出 Bayer 格式，需要转换
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam{};
        stConvertParam.nWidth = stImageInfo.nWidth;
        stConvertParam.nHeight = stImageInfo.nHeight;
        stConvertParam.pSrcData = pData_;
        stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;
        stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
        stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed; // 转为BGR
        stConvertParam.pDstBuffer = pDataForRGB_;
        stConvertParam.nDstBufferSize = stImageInfo.nWidth * stImageInfo.nHeight * 3;

        nRet = MV_CC_ConvertPixelType(handle_, &stConvertParam);
        if (MV_OK != nRet) return false;

        // 构造 cv::Mat (这是浅拷贝，注意生命周期，这里拷贝一份出去比较安全)
        cv::Mat temp(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pDataForRGB_);
        temp.copyTo(frame); // 深拷贝

        return true;
    }

    void HikDriver::close()
    {
        if (is_open_)
        {
            MV_CC_StopGrabbing(handle_);
            MV_CC_CloseDevice(handle_);
            MV_CC_DestroyHandle(handle_);
            is_open_ = false;
            m_logger->info("Camera closed successfully.");
        }
        m_logger->warn("Camera has already been closed.");
    }

    EnergyDetector::EnergyDetector()
    {
        m_logger = core::create_logger("EnergyDetector", spdlog::level::info);
        m_logger->info("EnergyDetector constructed.");
    }

    bool EnergyDetector::init()
    {
        const std::string model_path = YANDY_MODEL_PATH;
        auto tbl = toml::parse_file(YANDY_VISION_CONFIG);
        auto device = tbl["device"].value<std::string>().value();
        const auto thres = tbl["thres"].value<float>().value();
        conf_threshold_ = thres;

        try
        {
            // 读取模型
            std::shared_ptr<ov::Model> model = core_.read_model(model_path);

            // 预处理配置 (OpenVINO PPP)
            ov::preprocess::PrePostProcessor ppp(model);

            // 设定输入信息: 传入 uint8 BGR [1, H, W, 3] 的 OpenCV Mat
            ppp.input().tensor()
               .set_element_type(ov::element::u8)
               .set_layout("NHWC")
               .set_color_format(ov::preprocess::ColorFormat::BGR);

            // 设定模型期望: fp32 RGB [1, 3, 416, 416]
            // 输入是 1x3x416x416
            ppp.input().model()
               .set_layout("NCHW");

            // 预处理步骤: 转换颜色 -> 转换精度 -> 归一化 (0-255 -> 0-1)
            ppp.input().preprocess()
               .convert_color(ov::preprocess::ColorFormat::RGB)
               .convert_element_type(ov::element::f32)
               .scale(255.0f);

            model = ppp.build();

            // 编译模型
            compiled_model_ = core_.compile_model(model, device);
            infer_request_ = compiled_model_.create_infer_request();

            m_logger->info("Model loaded on {}", device);
            return true;
        }
        catch (const std::exception& e)
        {
            m_logger->error(" Init failed: {}", e.what());
            return false;
        }
    }

    std::vector<EnergyUnit> EnergyDetector::detect(const cv::Mat& raw_img)
    {
        std::vector<EnergyUnit> detections;
        if (raw_img.empty()) return detections;

        // 图像缩放 (Letterbox)
        // 为了保持长宽比，通常需要加黑边。这里为了简化，直接resize，
        // 但对于RoboMaster能量机关，建议使用Letterbox以保证精度。
        // 这里使用简单的Resize演示，实际建议替换为Letterbox逻辑。
        cv::Mat input_img;
        cv::resize(raw_img, input_img, model_input_shape_);

        // 准备输入 Tensor
        // 使用 input_img 的数据指针创建一个 Tensor (Zero-copy)
        ov::Tensor input_tensor(
            compiled_model_.input().get_element_type(),
            compiled_model_.input().get_shape(),
            input_img.data
        );
        infer_request_.set_input_tensor(input_tensor);

        // 推理
        infer_request_.infer();

        // 解析输出
        // 输出形状: 1x300x24 (根据你的Netron分析)
        const auto& output_tensor = infer_request_.get_output_tensor(0);
        const float* output_buffer = output_tensor.data<float>();

        // 计算缩放比例 (用于将坐标映射回原图)
        const float scale_x = static_cast<float>(raw_img.cols) / model_input_shape_.width;
        const float scale_y = static_cast<float>(raw_img.rows) / model_input_shape_.height;

        // 输出是一维数组排列的，stride = 24
        // [cx, cy, w, h, score, class, kpt1_x, kpt1_y, kpt1_v, ...]
        constexpr int rows = 300;
        constexpr int dimensions = 24;

        for (int i = 0; i < rows; ++i)
        {
            const float* data = output_buffer + i * dimensions;

            float score = data[4];
            if (score < conf_threshold_) continue;

            EnergyUnit unit;
            unit.confidence = score;
            unit.class_id = static_cast<int>(data[5]);

            // 解析 Box (cx, cy, w, h) -> Rect (x, y, w, h)
            float cx = data[0];
            float cy = data[1];
            float w = data[2];
            float h = data[3];

            int left = int((cx - 0.5 * w) * scale_x);
            int top = int((cy - 0.5 * h) * scale_y);
            int width = int(w * scale_x);
            int height = int(h * scale_y);

            unit.box = cv::Rect(left, top, width, height);

            // 解析 6 个关键点
            // 从索引 6 开始, 每 3 个一组 (x, y, conf)
            for (int k = 0; k < 6; ++k)
            {
                float kpt_x = data[6 + k * 3] * scale_x;
                float kpt_y = data[6 + k * 3 + 1] * scale_y;
                float kpt_s = data[6 + k * 3 + 2];

                unit.keypoints.emplace_back(kpt_x, kpt_y, kpt_s);
            }

            detections.push_back(unit);
        }

        return detections;
    }

    void EnergyDetector::drawResults(cv::Mat& img, const std::vector<EnergyUnit>& results)
    {
        for (const auto& res : results)
        {
            // 画框
            cv::rectangle(img, res.box, cv::Scalar(0, 255, 0), 2);

            std::string label = "Conf: " + std::to_string(res.confidence).substr(0, 4);
            cv::putText(img, label, res.box.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1);

            // 检查关键点数量是否足够
            if (res.keypoints.size() < 6) continue;

            const auto& pts = res.keypoints;
            // 假设模型训练时的关键点顺序为:
            // 0:上盖左, 1:上盖中, 2:上盖右
            // 3:下盖左, 4:下盖中, 5:下盖右

            // 1. 上盖连线 (黑白交界+中点) -> 0-1-2
            cv::line(img, cv::Point(pts[0].x, pts[0].y), cv::Point(pts[1].x, pts[1].y), cv::Scalar(0, 0, 255), 2);
            cv::line(img, cv::Point(pts[1].x, pts[1].y), cv::Point(pts[2].x, pts[2].y), cv::Scalar(0, 0, 255), 2);

            // 2. 下盖连线 -> 3-4-5
            cv::line(img, cv::Point(pts[3].x, pts[3].y), cv::Point(pts[4].x, pts[4].y), cv::Scalar(255, 0, 0), 2);
            cv::line(img, cv::Point(pts[4].x, pts[4].y), cv::Point(pts[5].x, pts[5].y), cv::Scalar(255, 0, 0), 2);

            // 3. 中点连线 (圆柱轴线) -> 1-4
            cv::line(img, cv::Point(pts[1].x, pts[1].y), cv::Point(pts[4].x, pts[4].y), cv::Scalar(0, 255, 255), 2);

            // 画出点
            for (int k = 0; k < 6; k++)
            {
                cv::circle(img, cv::Point(pts[k].x, pts[k].y), 4, cv::Scalar(255, 255, 0), -1);
                // 标号便于调试
                cv::putText(img, std::to_string(k), cv::Point(pts[k].x, pts[k].y), 0, 0.5, cv::Scalar(255, 255, 255));
            }
        }
    }
}


#include <yandy/modules/VisionSystem.hpp>
#include <yandy/core/Logger.hpp>
#include <toml++/toml.hpp>

#define YANDY_MODEL_PATH YANDY_CONFIG_PATH "yolo/best.onnx"
#define YANDY_VISION_CONFIG YANDY_CONFIG_PATH "vision.toml"

namespace yandy::module
{
    EnergyPoseSolver::EnergyPoseSolver(std::optional<cv::Mat> camera_matrix, std::optional<cv::Mat> dist_coeffs)
    {
        m_logger = core::create_logger("EnergyPoseSolver", spdlog::level::info);
        m_logger->info("Constructing EnergyPoseSolver...");
        auto tbl = toml::parse_file(YANDY_VISION_CONFIG);
        if (camera_matrix)
        {
            camera_matrix_ = camera_matrix.value();
        }
        else
        {
            m_logger->info("try loading camera matrix from {}", YANDY_VISION_CONFIG);
            camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
            auto intrinsics_node = tbl["camera"]["intrinsics"].as_array();
            int idx = 0;
            for (auto&& val : *intrinsics_node)
            {
                // 将一维数组填入 3x3 矩阵 (行优先)
                // at<double>(行, 列)
                camera_matrix_.at<double>(idx / 3, idx % 3) = val.value<double>().value();
                idx++;
            }
        }
        if (dist_coeffs)
        {
            dist_coeffs_ = dist_coeffs.value();
        }
        else
        {
            m_logger->info("try loading camera distortion from {}", YANDY_VISION_CONFIG);
            dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
            auto distortion_node = tbl["camera"]["distortion"].as_array();
            int idx = 0;
            for (auto&& val : *distortion_node)
            {
                dist_coeffs_.at<double>(0, idx) = val.value<double>().value();
                idx++;
            }
        }

        float w = 30.0f / 1000.0f; // 30mm -> 0.03m
        float h = 37.5f / 1000.0f; // 37.5mm -> 0.0375m

        object_points_ = {
            {-w, -h, 0}, // Top-Left  (2)
            {w, -h, 0}, // Top-Right (3)
            {w, h, 0}, // Bottom-Right (5)
            {-w, h, 0} // Bottom-Left (4)
        };
        m_logger->info("EnergyPoseSolver constructed.");
    }

    bool EnergyPoseSolver::solve(const EnergyUnit& unit, Eigen::Isometry3d& output_pose) const
    {
        if (unit.keypoints.size() < 6) return false;

        std::vector<cv::Point2f> image_points;
        const auto& kpts = unit.keypoints;

        // 2D 点的提取顺序必须与 3D 点 object_points_ 一一对应
        // 0:下中, 1:上中, 2:上左, 3:上右, 4:下左, 5:下右
        // 对应 object_points 的顺序 (TL, TR, BR, BL):

        image_points.emplace_back(kpts[2].x, kpts[2].y); // Top-Left
        image_points.emplace_back(kpts[3].x, kpts[3].y); // Top-Right
        image_points.emplace_back(kpts[5].x, kpts[5].y); // Bottom-Right
        image_points.emplace_back(kpts[4].x, kpts[4].y); // Bottom-Left

        // 解算 PnP
        cv::Mat rvec, tvec;
        // SOLVEPNP_IPPE 是专门针对平面物体 (Planar targets) 的高精度算法，非常适合能量机关
        const bool success = cv::solvePnP(object_points_, image_points,
                                          camera_matrix_, dist_coeffs_,
                                          rvec, tvec, false, cv::SOLVEPNP_IPPE);

        if (!success) return false;

        // 将 OpenCV 格式转换为 Eigen 格式 (给 Pinocchio 用)
        cv::Mat R_cv;
        cv::Rodrigues(rvec, R_cv); // 旋转向量 -> 旋转矩阵

        Eigen::Matrix3d R_eigen;
        Eigen::Vector3d t_eigen;

        // OpenCV 是 double 还是 float 取决于输入，这里为了保险做一下 cast
        for (int i = 0; i < 3; i++)
        {
            t_eigen(i) = tvec.at<double>(i);
            for (int j = 0; j < 3; j++)
            {
                R_eigen(i, j) = R_cv.at<double>(i, j);
            }
        }

        // 组装成 Isometry3d (T_camera_object)
        output_pose = Eigen::Isometry3d::Identity();
        output_pose.linear() = R_eigen;
        output_pose.translation() = t_eigen;

        return true;
    }

    cv::Mat& EnergyPoseSolver::get_camera_matrix()
    {
        return camera_matrix_;
    }

    cv::Mat& EnergyPoseSolver::get_dist_coeffs()
    {
        return dist_coeffs_;
    }

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
        const cv::Mat temp(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pDataForRGB_);
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
        auto device = tbl["yolo"]["device"].value<std::string>().value();
        const auto thres = tbl["yolo"]["thres"].value<float>().value();
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
        cv::Mat input_img = this->letterbox(raw_img);

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
            float x1 = data[0];
            float y1 = data[1];
            float x2 = data[2];
            float y2 = data[3];

            auto restore_coord = [&](float x, float y) -> cv::Point2f
            {
                float rx = (x - pad_offset_.x) / scale_factor_;
                float ry = (y - pad_offset_.y) / scale_factor_;
                return cv::Point2f(rx, ry);
            };

            cv::Point2f tl = restore_coord(x1, y1);
            cv::Point2f br = restore_coord(x2, y2);
            unit.box = cv::Rect(tl, br);

            for (int k = 0; k < 6; ++k)
            {
                float kpt_x = data[6 + k * 3];
                float kpt_y = data[6 + k * 3 + 1];
                float kpt_s = data[6 + k * 3 + 2];

                // 同样需要做坐标反算
                cv::Point2f pt = restore_coord(kpt_x, kpt_y);

                // 将 z 轴位置存放 confidence
                unit.keypoints.emplace_back(pt.x, pt.y, kpt_s);
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

            int i_top_left = 2;
            int i_top_center = 1;
            int i_top_right = 3;
            int i_btm_left = 4;
            int i_btm_center = 0;
            int i_btm_right = 5;
            auto draw_pt = [&](int idx, cv::Scalar color)
            {
                cv::circle(img, cv::Point(pts[idx].x, pts[idx].y), 5, color, -1);
                // 调试：把序号写在点旁边，确认没搞错
                // cv::putText(img, std::to_string(idx), cv::Point(pts[idx].x, pts[idx].y), 0, 0.5, cv::Scalar(255,255,255), 1);
            };

            // 上排用红色系，下排用蓝色系，中间用黄色
            draw_pt(i_top_left, cv::Scalar(0, 0, 255)); // Red
            draw_pt(i_top_center, cv::Scalar(0, 255, 255)); // Yellow
            draw_pt(i_top_right, cv::Scalar(0, 0, 255)); // Red

            draw_pt(i_btm_left, cv::Scalar(255, 0, 0)); // Blue
            draw_pt(i_btm_center, cv::Scalar(0, 255, 255)); // Yellow
            draw_pt(i_btm_right, cv::Scalar(255, 0, 0)); // Blue

            // 3. 画线 (工字型)
            // 为了线比较顺滑，我们定义一个画线lambda
            auto draw_line = [&](int idx1, int idx2, cv::Scalar color)
            {
                cv::line(img, cv::Point(pts[idx1].x, pts[idx1].y),
                         cv::Point(pts[idx2].x, pts[idx2].y), color, 2);
            };

            // --- 上横线 (Left -> Center -> Right) ---
            draw_line(i_top_left, i_top_center, cv::Scalar(0, 255, 0)); // Green
            draw_line(i_top_center, i_top_right, cv::Scalar(0, 255, 0)); // Green

            // --- 下横线 (Left -> Center -> Right) ---
            draw_line(i_btm_left, i_btm_center, cv::Scalar(0, 165, 255)); // Orange
            draw_line(i_btm_center, i_btm_right, cv::Scalar(0, 165, 255)); // Orange

            // --- 中竖线 (Top Center -> Bottom Center) ---
            draw_line(i_top_center, i_btm_center, cv::Scalar(255, 0, 255)); // Purple
        }
    }

    cv::Mat EnergyDetector::letterbox(const cv::Mat& source)
    {
        int col = source.cols;
        int row = source.rows;
        int _max = MAX(col, row);

        // 计算缩放比例，取较小值以适应目标尺寸
        // 注意：这里目标是正方形 416x416
        float scale = std::min((float)model_input_shape_.width / col, (float)model_input_shape_.height / row);

        // 保存缩放因子供后续还原坐标使用
        this->scale_factor_ = scale;

        int new_w = round(col * scale);
        int new_h = round(row * scale);

        // 计算需要填充的黑边大小
        int dw = (model_input_shape_.width - new_w) / 2;
        int dh = (model_input_shape_.height - new_h) / 2;

        this->pad_offset_ = cv::Point2f((float)dw, (float)dh);

        // 1. 缩放
        cv::Mat resized_img;
        cv::resize(source, resized_img, cv::Size(new_w, new_h));
        // 2. 填充边缘 (填充灰色 114)
        cv::Mat dst_img;
        cv::copyMakeBorder(resized_img, dst_img, dh, dh, dw, dw, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

        // 此时 dst_img 大小可能因为 round 误差差 1 个像素，强制 resize 修正一下
        if (dst_img.size() != model_input_shape_)
        {
            cv::resize(dst_img, dst_img, model_input_shape_);
        }

        return dst_img;
    }
}


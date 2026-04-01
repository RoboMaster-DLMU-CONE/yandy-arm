#include <yandy/utils/UsbReset.hpp>
#include <yandy/core/Logger.hpp>

#include <fstream>
#include <thread>
#include <chrono>
#include <filesystem>
#include <system_error>

namespace yandy::utils {

UsbReset::UsbReset(std::string usb_port)
    : m_usb_port(std::move(usb_port))
    , m_logger(core::create_logger("UsbReset", spdlog::level::info))
{
    m_logger->info("UsbReset initialized for port: {}", m_usb_port);
}

bool UsbReset::reset() {
    if (m_usb_port.empty()) {
        m_logger->error("USB port not configured");
        return false;
    }

    m_logger->warn("Attempting USB soft reset for port: {}", m_usb_port);

    // Step 1: Unbind (断开设备)
    {
        std::ofstream unbind_file(UNBIND_PATH);
        if (!unbind_file.is_open()) {
            m_logger->error("Failed to open unbind file: {}. Need root permission!", UNBIND_PATH);
            return false;
        }
        
        unbind_file << m_usb_port;
        unbind_file.close();
        
        if (unbind_file.fail()) {
            m_logger->error("Failed to write to unbind file");
            return false;
        }
        
        m_logger->info("USB port {} unbound successfully", m_usb_port);
    }

    // Step 2: 等待内核处理断开
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Step 3: Bind (重新连接设备)
    {
        std::ofstream bind_file(BIND_PATH);
        if (!bind_file.is_open()) {
            m_logger->error("Failed to open bind file: {}", BIND_PATH);
            return false;
        }
        
        bind_file << m_usb_port;
        bind_file.close();
        
        if (bind_file.fail()) {
            m_logger->error("Failed to write to bind file");
            return false;
        }
        
        m_logger->info("USB port {} bound successfully", m_usb_port);
    }

    // Step 4: 等待设备重新枚举
    std::this_thread::sleep_for(std::chrono::seconds(2));

    m_logger->info("USB soft reset completed for port: {}", m_usb_port);
    return true;
}

std::optional<std::string> UsbReset::detect_usb_port(const std::string& device_path) {
    namespace fs = std::filesystem;
    
    try {
        // 构建 sysfs 路径: /sys/class/tty/ttyACM0
        std::string device_name = fs::path(device_path).filename();
        fs::path sysfs_path = fs::path("/sys/class/tty") / device_name / "device";
        
        if (!fs::exists(sysfs_path)) {
            return std::nullopt;
        }
        
        // 解析符号链接，找到实际的 USB 设备路径
        fs::path real_path = fs::read_symlink(sysfs_path);
        
        // 向上遍历目录树，找到包含 busnum 的目录（这是 USB 设备节点）
        fs::path current = sysfs_path.parent_path() / real_path;
        current = fs::canonical(current);
        
        while (current != current.root_path()) {
            // 检查是否存在 busnum 文件（USB 设备的标志）
            if (fs::exists(current / "busnum")) {
                // 获取 USB 端口号（目录名的最后部分）
                std::string port = current.filename().string();
                
                // 验证格式（应该类似 "1-2" 或 "2-1.3"）
                if (port.find('-') != std::string::npos) {
                    return port;
                }
            }
            current = current.parent_path();
        }
        
        return std::nullopt;
    } catch (const std::exception& e) {
        return std::nullopt;
    }
}

} // namespace yandy::utils

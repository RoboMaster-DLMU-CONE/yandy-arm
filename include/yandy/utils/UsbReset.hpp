#ifndef YANDY_ARM_USB_RESET_HPP
#define YANDY_ARM_USB_RESET_HPP

#include <string>
#include <memory>
#include <optional>

namespace spdlog {
    class logger;
}

namespace yandy::utils {

/**
 * @brief USB 软重置工具类
 * 
 * 通过 Linux sysfs 接口执行 USB 设备的 unbind/bind 操作，
 * 实现不拔插硬件的软重置。需要 root 权限。
 */
class UsbReset {
public:
    /**
     * @brief 构造函数
     * @param usb_port USB 端口路径，例如 "1-2" 或 "2-1.3"
     */
    explicit UsbReset(std::string usb_port);

    /**
     * @brief 执行 USB 软重置
     * @return true 表示重置成功，false 表示失败（可能是权限不足）
     */
    bool reset();

    /**
     * @brief 设置 USB 端口
     * @param port USB 端口路径
     */
    void set_port(const std::string& port) { m_usb_port = port; }

    /**
     * @brief 获取当前配置的 USB 端口
     * @return USB 端口路径
     */
    [[nodiscard]] const std::string& get_port() const { return m_usb_port; }

    /**
     * @brief 尝试从设备路径自动检测 USB 端口
     * @param device_path 设备路径，如 "/dev/ttyACM0"
     * @return USB 端口路径，如果检测失败则返回 nullopt
     */
    static std::optional<std::string> detect_usb_port(const std::string& device_path);

private:
    std::string m_usb_port;
    std::shared_ptr<spdlog::logger> m_logger;

    // sysfs 路径
    static constexpr const char* UNBIND_PATH = "/sys/bus/usb/drivers/usb/unbind";
    static constexpr const char* BIND_PATH = "/sys/bus/usb/drivers/usb/bind";
};

} // namespace yandy::utils

#endif // YANDY_ARM_USB_RESET_HPP

#include <yandy/core/Logger.hpp>
#include <yandy/modules/InputProvider.hpp>
#include <yandy/utils/UsbReset.hpp>

#include <toml++/toml.hpp>

#define YANDY_INPUT_CONFIG YANDY_CONFIG_PATH "input.toml"

namespace yandy::modules {
namespace detail {
IInputProvider::IInputProvider() : m_par(m_des) {}

void IInputProvider::setCommandCb(
    const std::function<void(YandyControlCmd)> &func) {
  m_func = func;
}

YandyControlPack IInputProvider::getLatestCommand() { return m_buf.read(); }

void IInputProvider::update_cmd(const YandyControlCmd cmd) {
  if (cmd == m_candidate_cmd) {
    if (m_stability_count < m_filter_threshold) {
      ++m_stability_count;
    }
  } else {
    m_candidate_cmd = cmd;
    m_stability_count = 1;
  }
  if (m_stability_count >= m_filter_threshold) {
    if (m_candidate_cmd != m_current_stable_cmd) {
      m_current_stable_cmd = m_candidate_cmd;
      m_func(m_current_stable_cmd);
    }
  }
}

UdpProvider::UdpProvider() : m_socket(m_io_context), m_running(true) {
  m_logger = core::create_logger("UdpInputProvider", spdlog::level::info);
  m_logger->info("Constructing UdpProvider, loading config from {}",
                 YANDY_INPUT_CONFIG);

  auto tbl = toml::parse_file(YANDY_INPUT_CONFIG);
  auto address = tbl["udp"]["address"].value<std::string>().value();
  auto port = tbl["udp"]["port"].value<uint16_t>().value();

  udp::endpoint endpoint(boost::asio::ip::make_address(address), port);
  m_socket.open(endpoint.protocol());
  m_socket.bind(endpoint);

  m_logger->info("UdpProvider bound to {}:{}", address, port);

  startReceiveThread();
}

void UdpProvider::startReceiveThread() {
  m_receive_thread = std::thread([this]() {
    doReceive();
    m_io_context.run();
  });
}

void UdpProvider::doReceive() {
  m_socket.async_receive_from(boost::asio::buffer(m_recv_buffer),
                              m_remote_endpoint,
                              [this](const boost::system::error_code &error,
                                     std::size_t bytes_transferred) {
                                receiveHandler(error, bytes_transferred);
                              });
}

void UdpProvider::receiveHandler(const boost::system::error_code &error,
                                 std::size_t bytes_transferred) {
  if (!error && m_running) {
    const auto data_ptr =
        reinterpret_cast<const uint8_t *>(m_recv_buffer.data());

    auto result = m_par.push_data(data_ptr, bytes_transferred);
    if (result) {
      const auto packet = m_des.get<YandyControlPack>();
      update_cmd(packet.cmd);
      m_buf.write(packet);
    }

    // 重新开始接收下一个数据包
    doReceive();
  } else if (error != boost::asio::error::operation_aborted && m_running) {
    m_logger->error("UDP receive error: {}", error.message());

    // 出错后继续尝试接收
    doReceive();
  }
}

UsbProvider::UsbProvider() {
  m_logger = core::create_logger("UsbInputProvider", spdlog::level::info);
  m_logger->info("Constructing UsbProvider, loading config from {}",
                 YANDY_INPUT_CONFIG);

  auto tbl = toml::parse_file(YANDY_INPUT_CONFIG);
  auto device = tbl["usb"]["device"].value<std::string>().value();
  auto baud_rate = tbl["usb"]["baud_rate"].value<uint32_t>().value();

  // 加载 USB 重置配置
  m_reset_enabled = tbl["usb"]["reset_enabled"].value_or(false);
  m_usb_port = tbl["usb"]["usb_port"].value_or(std::string("auto"));
  m_check_interval_sec = tbl["usb"]["check_interval_sec"].value_or(5);
  m_zero_packet_threshold = tbl["usb"]["zero_packet_threshold"].value_or(5);
  m_timeout_sec = tbl["usb"]["timeout_sec"].value_or(10);

  m_logger->info("USB Reset config: enabled={}, port={}, check_interval={}s, "
                 "zero_threshold={}, timeout={}s",
                 m_reset_enabled, m_usb_port, m_check_interval_sec,
                 m_zero_packet_threshold, m_timeout_sec);

  // 如果配置为 auto，尝试自动检测
  if (m_reset_enabled && m_usb_port == "auto") {
    auto detected_port = utils::UsbReset::detect_usb_port(device);
    if (detected_port) {
      m_usb_port = *detected_port;
      m_logger->info("Auto-detected USB port: {}", m_usb_port);
    } else {
      m_logger->warn("Failed to auto-detect USB port, reset will be disabled");
      m_reset_enabled = false;
    }
  }

  m_cfg.device_path = device;
  m_cfg.baud_rate = baud_rate;
  m_cfg.data_bits = HySerial::DataBits::BITS_8;
  m_cfg.parity = HySerial::Parity::NONE;
  m_cfg.stop_bits = HySerial::StopBits::ONE;

  HySerial::Builder builder;
  builder.device(device)
      .baud_rate(baud_rate)
      .data_bits(HySerial::DataBits::BITS_8)
      .parity(HySerial::Parity::NONE)
      .stop_bits(HySerial::StopBits::ONE);

  builder.on_read(
      [this](const std::span<const std::byte> data) { on_serial_read(data); });
  builder.on_error([this](const ssize_t e) { on_serial_error(e); });

  auto serial_or_err = builder.build();
  if (!serial_or_err) {
    m_logger->error("Failed to create Serial: {}",
                    serial_or_err.error().message);
    throw std::runtime_error("Failed to create Usb Serial port");
  }
  m_serial = std::move(serial_or_err.value());
  m_serial->start_read();
  
  m_reconnect_thread = std::thread(&UsbProvider::reconnect_worker, this);
  
  // 启动监控线程
  if (m_reset_enabled) {
    m_monitor_thread = std::thread(&UsbProvider::monitor_worker, this);
    m_logger->info("USB quality monitor started");
  }
}

UsbProvider::~UsbProvider() {
  m_running.store(false);
  if (m_reconnect_thread.joinable()) {
      m_reconnect_thread.join();
  }
  if (m_monitor_thread.joinable()) {
      m_monitor_thread.join();
  }
}

void UsbProvider::on_serial_read(std::span<const std::byte> data) {
  const auto size = data.size();
  static int read_count = 0;
  if (++read_count % 100 == 0) {
      m_logger->debug("UsbProvider received {} bytes", size);
  }
  const auto u8_data = reinterpret_cast<const uint8_t *>(data.data());
  
  auto res = m_par.push_data(u8_data, size);
  if (!res) {
      m_logger->error("Parser push_data failed: {}", res.error().message);
  } else {
      const auto packet = m_des.get<YandyControlPack>();
      
      // 记录数据包质量
      m_quality.record_packet(packet);
      
      update_cmd(packet.cmd);
      m_buf.write(packet);
  }
}

void UsbProvider::on_serial_error(const ssize_t e) {
  if (e == -EINTR || e == -EAGAIN || e == -EWOULDBLOCK) {
      return;
  }
  m_logger->error("UsbProvider error: {}", e);
  m_need_reconnect.store(true);
}

void UsbProvider::reconnect_worker() {
    while (m_running.load()) {
        if (m_need_reconnect.load()) {
            m_logger->warn("Serial connection lost, attempting to reconnect in 1s...");
            m_serial->stop_read();
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            // Re-create the serial instance using the builder
            HySerial::Builder builder;
            builder.device(m_cfg.device_path)
                  .baud_rate(m_cfg.baud_rate)
                  .data_bits(m_cfg.data_bits)
                  .parity(m_cfg.parity)
                  .stop_bits(m_cfg.stop_bits);
                  
            builder.on_read([this](const std::span<const std::byte> data) { on_serial_read(data); });
            builder.on_error([this](const ssize_t e) { on_serial_error(e); });
            
            auto serial_or_err = builder.build();
            if (serial_or_err) {
                m_serial = std::move(serial_or_err.value());
                m_serial->start_read();
                m_logger->info("Serial connection restored successfully.");
                m_need_reconnect.store(false);
                
                // 重置数据质量监控
                m_quality.reset();
            } else {
                m_logger->error("Failed to reconnect: {}", serial_or_err.error().message);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// ===== 数据质量监控实现 =====

bool UsbProvider::DataQualityMonitor::is_data_all_zero(const YandyControlPack& pack) {
    // 检查所有浮点数字段是否都是0
    return pack.x == 0.0f && pack.y == 0.0f && pack.z == 0.0f &&
           pack.ee_qw == 0.0f && pack.ee_qx == 0.0f && 
           pack.ee_qy == 0.0f && pack.ee_qz == 0.0f &&
           pack.gimbal_z == 0.0f && pack.gimbal_yaw == 0.0f && 
           pack.gimbal_pitch == 0.0f &&
           pack.ax == 0.0f && pack.ay == 0.0f && pack.az == 0.0f &&
           pack.gx == 0.0f && pack.gy == 0.0f && pack.gz == 0.0f &&
           pack.qw == 0.0f && pack.qx == 0.0f && 
           pack.qy == 0.0f && pack.qz == 0.0f;
}

void UsbProvider::DataQualityMonitor::record_packet(const YandyControlPack& pack) {
    total_packets.fetch_add(1, std::memory_order_relaxed);
    
    if (is_data_all_zero(pack)) {
        zero_packets.fetch_add(1, std::memory_order_relaxed);
        consecutive_zero_packets.fetch_add(1, std::memory_order_relaxed);
    } else {
        consecutive_zero_packets.store(0, std::memory_order_relaxed);
        last_valid_data = std::chrono::steady_clock::now();
    }
}

bool UsbProvider::DataQualityMonitor::is_unhealthy(int zero_threshold, int timeout_sec) {
    auto now = std::chrono::steady_clock::now();
    auto silence_duration = std::chrono::duration_cast<std::chrono::seconds>(
        now - last_valid_data).count();
    
    // 获取当前连续全0包计数
    uint64_t consecutive_zeros = consecutive_zero_packets.load(std::memory_order_relaxed);
    
    // 条件1: 连续全0数据包过多
    if (consecutive_zeros >= static_cast<uint64_t>(zero_threshold)) {
        return true;
    }
    
    // 条件2: 超时无有效数据（并且总包数大于0，避免刚启动时误触发）
    if (silence_duration > timeout_sec && total_packets.load(std::memory_order_relaxed) > 10) {
        return true;
    }
    
    return false;
}

void UsbProvider::DataQualityMonitor::reset() {
    total_packets.store(0, std::memory_order_relaxed);
    zero_packets.store(0, std::memory_order_relaxed);
    consecutive_zero_packets.store(0, std::memory_order_relaxed);
    last_valid_data = std::chrono::steady_clock::now();
}

void UsbProvider::monitor_worker() {
    m_logger->info("Data quality monitor thread started");
    
    while (m_running.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(m_check_interval_sec));
        
        if (m_quality.is_unhealthy(m_zero_packet_threshold, m_timeout_sec)) {
            auto consecutive = m_quality.consecutive_zero_packets.load();
            auto total = m_quality.total_packets.load();
            auto now = std::chrono::steady_clock::now();
            auto silence = std::chrono::duration_cast<std::chrono::seconds>(
                now - m_quality.last_valid_data).count();
            
            m_logger->error("Data quality unhealthy! consecutive_zero={}, total={}, silence={}s", 
                          consecutive, total, silence);
            trigger_recovery();
            
            // 触发恢复后等待一段时间，避免重复触发
            std::this_thread::sleep_for(std::chrono::seconds(10));
        }
    }
    
    m_logger->info("Data quality monitor thread stopped");
}

void UsbProvider::trigger_recovery() {
    m_logger->warn("Triggering recovery strategy...");
    
    // 直接执行 USB 软重置（Level 2）
    if (m_reset_enabled && !m_usb_port.empty()) {
        m_logger->warn("Performing USB soft reset for port: {}", m_usb_port);
        
        // 先停止当前的串口读取
        if (m_serial) {
            m_serial->stop_read();
        }
        
        // 执行 USB unbind/bind
        utils::UsbReset usb_reset(m_usb_port);
        if (usb_reset.reset()) {
            m_logger->info("USB reset successful, device should re-enumerate now");
            
            // 等待设备重新枚举（USB reset 内部已经等待2秒）
            // 再额外等待1秒确保设备完全就绪
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            // 触发 HySerial 重连以使用新的设备
            m_need_reconnect.store(true);
            m_quality.reset();
            
            m_logger->info("Recovery completed, waiting for reconnection");
        } else {
            m_logger->error("USB reset failed! Falling back to HySerial reconnect");
            // USB 重置失败，回退到 HySerial 重连
            m_need_reconnect.store(true);
        }
    } else {
        // USB 重置未启用，使用 HySerial 重连
        m_logger->info("USB reset disabled, using HySerial reconnect only");
        m_need_reconnect.store(true);
    }
}

} // namespace detail

InputProvider::InputProvider() {
  const auto temp_logger = core::get_default_logger();
  temp_logger->info("Constructing InputProvider, loading config from {}",
                    YANDY_INPUT_CONFIG);
  auto tbl = toml::parse_file(YANDY_INPUT_CONFIG);
  if (auto type = tbl["type"].value<std::string_view>().value();
      type == "usb") {
    temp_logger->info("Construction UsbInputProvider...");
    m_provider = std::make_unique<detail::UsbProvider>();
  } else if (type == "udp") {
    temp_logger->info("Construction UdpInputProvider...");
    m_provider = std::make_unique<detail::UdpProvider>();
  } else {
    temp_logger->error("Invalid type: {}, please use 'udp' or 'usb'", type);
    throw std::runtime_error("invalid type");
  }
}

void InputProvider::setCommandCb(
    const std::function<void(YandyControlCmd)> &func) const {
  m_provider->setCommandCb(func);
}

YandyControlPack InputProvider::getLatestCommand() const {
  return m_provider->getLatestCommand();
}
} // namespace yandy::modules

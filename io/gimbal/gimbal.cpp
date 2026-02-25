#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

#include <cstdint>
#include <cstring>
#include <fmt/format.h>

namespace io
{

static inline uint32_t float_bits(float v)
{
  uint32_t u = 0;
  std::memcpy(&u, &v, sizeof(u));
  return u;
}

Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");//读取配置文件中的串口

  // uint32_t baudrate = 115200u;
  uint32_t timeout_ms = 50u; 
  
  try 
  {
    serial_.setPort(com_port);
    // serial_.setBaudrate(baudrate);
    auto time_out = serial::Timeout::simpleTimeout(timeout_ms);
    serial_.setTimeout(time_out);
    serial_.open();

  } 
  catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    //exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  //queue_.pop();
  //tools::logger()->info("[Gimbal] First q received.");
  if (!queue_.empty()) {
    queue_.pop(); // 仅当队列有数据时pop
    tools::logger()->info("[Gimbal] First q received.");
  } else {
    tools::logger()->warn("[Gimbal] No valid data in 3s, skip pop.");
  }
   this->send(false, false, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);//新增
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  auto gs = state();
  VisionToGimbal.yaw = tools::limit_rad(VisionToGimbal.yaw - gs.yaw);
  VisionToGimbal.pitch = tools::limit_rad(VisionToGimbal.pitch - gs.pitch);

  if (VisionToGimbal.mode == 0) { // mode=0对应IDLE，无目标
    VisionToGimbal.yaw = 0.0f;
    VisionToGimbal.pitch = 0.0f;
  }

  tx_data_.mode = VisionToGimbal.mode;
  tx_data_.yaw = VisionToGimbal.yaw;
  tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
  tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
  tx_data_.pitch = VisionToGimbal.pitch;
  tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
  tx_data_.pitch_acc = VisionToGimbal.pitch_acc;
  
  tx_data_.crc8 = tools::get_crc8(
  reinterpret_cast<uint8_t *>(&tx_data_),sizeof(tx_data_) - sizeof(tx_data_.crc8) - sizeof(tx_data_.tail));
  tx_data_.tail = 0x0d;

  //日志
  tools::logger()->info(
    "[Gimbal] Send data | "
    "mode={:02x} yaw={:.2f} yaw_vel={:.2f} yaw_acc={:.2f} "
    "pitch={:.2f} pitch_vel={:.2f} pitch_acc={:.2f} "
    "crc8={:02x} tail={:02x}",
    static_cast<uint32_t>(tx_data_.mode),
    tx_data_.yaw, tx_data_.yaw_vel, tx_data_.yaw_acc,
    tx_data_.pitch, tx_data_.pitch_vel, tx_data_.pitch_acc,
    static_cast<uint32_t>(tx_data_.crc8),
    static_cast<uint32_t>(tx_data_.tail)
  );

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
    sent_once_.store(true);//新增
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  auto gs = state();
  
  double delta_yaw = yaw - gs.yaw;
  double delta_pitch = pitch - gs.pitch;

  delta_yaw = tools::limit_rad(delta_yaw);
  delta_pitch = tools::limit_rad(delta_pitch);

  if (!control) 
  { 
    delta_yaw = 0.0f;
    delta_pitch = 0.0f;
    yaw_vel = 0.0f;
    yaw_acc = 0.0f;
    pitch_vel = 0.0f;
    pitch_acc = 0.0f;
  }

  yaw = static_cast<float>(delta_yaw);
  pitch = static_cast<float>(delta_pitch);

  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
  tx_data_.crc8 = tools::get_crc8(
  reinterpret_cast<uint8_t *>(&tx_data_),sizeof(tx_data_) - sizeof(tx_data_.crc8) - sizeof(tx_data_.tail));
  tx_data_.tail = 0x0d;

  tools::logger()->info(
    "[Gimbal] Send data | "
    "control={} fire={} mode={:02x} "
    "yaw={:.2f} yaw_vel={:.2f} yaw_acc={:.2f} "
    "pitch={:.2f} pitch_vel={:.2f} pitch_acc={:.2f} "
    "crc8={:02x} tail={:02x}",
    control, fire, static_cast<uint32_t>(tx_data_.mode),
    tx_data_.yaw, tx_data_.yaw_vel, tx_data_.yaw_acc,
    tx_data_.pitch, tx_data_.pitch_vel, tx_data_.pitch_acc,
    static_cast<uint32_t>(tx_data_.crc8),
    static_cast<uint32_t>(tx_data_.tail)
  );

  try 
  {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
    sent_once_.store(true);//新增
  } catch (const std::exception & e) 
  {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
   tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

void Gimbal::read_thread()
{
    tools::logger()->info("[Gimbal] read_thread started.");
  
  const int PACKET_SIZE = 42;
  uint8_t buffer[PACKET_SIZE];
  int error_count = 0;

  bool has_received_data = false; // 核心：是否已收到云台的有效回调数据
  const int DEFAULT_SEND_INTERVAL_MS = 50; // 自动发默认值的间隔，避免刷屏

   // 启动后立即标记为需要发送，触发自动发默认值
  sent_once_.store(true);

  while (!quit_) 
  {
    
    // 未收到有效数据时，循环发送默认控制帧
    if (!has_received_data && sent_once_.load()) {
      // 调用send发默认值
      this->send(false, false, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      // 短暂休眠
      std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_SEND_INTERVAL_MS));
    }

    if (error_count > 50) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, reconnecting...");
      reconnect();
      continue;
    }
    
    // 1. 寻找帧头
    uint8_t head_byte;
    while (!quit_) {
      if (!read(&head_byte, 1)) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        continue;
      }
      if (head_byte == 0xff) {
        buffer[0] = head_byte;
        //tools::logger()->debug("[Gimbal] Found frame head 0xff");
        break;
      }
    }
    
    if (quit_) break;
    
    auto receive_time = std::chrono::steady_clock::now();
    
    // 读取剩余数据
    bool read_ok = true;
    for (int i = 1; i < PACKET_SIZE; i++) {
      if (!read(&buffer[i], 1)) {
        read_ok = false;
        break;
      }
    }
    
    if (!read_ok) {
      error_count++;
      continue;
    }
    
    // 检查帧尾
    if (buffer[PACKET_SIZE - 1] != 0x0d) {
      error_count++;
      tools::logger()->debug("[Gimbal] Tail check failed: got {:#04x}, expected 0x0d", static_cast<int>(buffer[PACKET_SIZE - 1]));
      tools::logger()->debug("[Gimbal] Buffer dump: {:02x}", fmt::join(buffer, buffer + PACKET_SIZE, " "));
      continue;
    }
    
    // 4. 解析数据包
    memcpy(&rx_data_, buffer, PACKET_SIZE);
    auto t = std::chrono::steady_clock::now();

    //auto t = std::chrono::steady_clock::now();
    /*if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.head))) {
      error_count++;
      continue;
      }

    if (rx_data_.head != 0xff) 
    {
      error_count++;
      tools::logger()->warn(
          "[Gimbal] Head mismatch: expected 0xff, got {:#04x}", static_cast<int>(rx_data_.head));
      continue;
    }

    if (!read(reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
              sizeof(rx_data_) - sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }


    auto t = std::chrono::steady_clock::now();*/
    //const auto crc8 = tools::get_crc8(reinterpret_cast<uint8_t *>(&rx_data_),sizeof(rx_data_) - sizeof(rx_data_.crc8) - sizeof(rx_data_.tail));
    //tools::logger()->debug("[Gimbal] crc8={:#04x}", static_cast<int>(crc8));
    //if (crc8 != rx_data_.crc8) 
    //{
      //tools::logger()->debug("[Gimbal] CRC8 check failed.");
     // continue;
    //}

    if (!has_received_data) {
      has_received_data = true;
    }

    error_count = 0;
    tools::logger()->debug(
      "[Gimbal] OK | "
      "head={} mode={} "
      "q=[{},{},{},{}] "
      "yaw={} yaw_vel={} pitch={} pitch_vel={} bullet_speed={} "
      "bullet_count={} crc8={} tail={} ",
      static_cast<uint32_t>(rx_data_.head), 
      static_cast<int>(rx_data_.mode),  
      rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3], 
      rx_data_.yaw, rx_data_.yaw_vel,  
      rx_data_.pitch, rx_data_.pitch_vel, 
      rx_data_.bullet_speed,                
      static_cast<int>(rx_data_.bullet_count), 
      static_cast<uint32_t>(rx_data_.crc8),  
      static_cast<uint32_t>(rx_data_.tail)
    );
    Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
    queue_.push({q, receive_time});
    
    std::lock_guard<std::mutex> lock(mutex_);

    state_.yaw = rx_data_.yaw;
    state_.yaw_vel = rx_data_.yaw_vel;
    state_.pitch = rx_data_.pitch;
    state_.pitch_vel = rx_data_.pitch_vel;
    state_.bullet_speed = rx_data_.bullet_speed;
    state_.bullet_count = rx_data_.bullet_count;

    switch (rx_data_.mode) {
      case 0:
        mode_ = GimbalMode::IDLE;
        break;
      case 1:
        mode_ = GimbalMode::AUTO_AIM;
        break;
      case 2:
        mode_ = GimbalMode::SMALL_BUFF;
        break;
      case 3:
        mode_ = GimbalMode::BIG_BUFF;
        break;
      default:
        mode_ = GimbalMode::IDLE;
        tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
        break;
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      serial_.flushInput();
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}
}//namespace io
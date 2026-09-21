#include "exoskeleton_hardware/exo_hardware_interface.hpp"

#include <fcntl.h>
#include <glob.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace exoskeleton_hardware
{

// ===================== XOR 校验 =====================

uint8_t ExoHardwareInterface::calc_checksum(uint8_t cmd, uint8_t len, const uint8_t * data)
{
  uint8_t chk = cmd ^ len;
  for (uint8_t i = 0; i < len; ++i) {
    chk ^= data[i];
  }
  return chk;
}

int32_t ExoHardwareInterface::ankle_count_from_zero(
  uint16_t raw_count, uint16_t zero_offset)
{
  constexpr int32_t resolution =
    static_cast<int32_t>(usb_protocol::ANKLE_ENCODER_RESOLUTION);
  constexpr int32_t half_resolution = resolution / 2;
  int32_t delta = static_cast<int32_t>(raw_count) - static_cast<int32_t>(zero_offset);
  delta = (delta + half_resolution) % resolution;
  if (delta < 0) {
    delta += resolution;
  }
  return delta - half_resolution;
}

// ===================== 发送命令帧 =====================
// 帧格式: STX_REQ(0xAA) | CMD | LEN | DATA... | CHK

bool ExoHardwareInterface::send_command(uint8_t cmd, const uint8_t * data, uint8_t len)
{
  if (fd_ < 0) return false;

  uint8_t frame[256];
  frame[0] = usb_protocol::STX_REQ;
  frame[1] = cmd;
  frame[2] = len;
  if (len > 0 && data != nullptr) {
    std::memcpy(frame + 3, data, len);
  }
  frame[3 + len] = calc_checksum(cmd, len, data);

  const size_t total = 4 + len;
  ssize_t n = ::write(fd_, frame, total);
  if (n != static_cast<ssize_t>(total)) {
    return false;
  }
  return true;
}

// ===================== 马达使能 =====================

bool ExoHardwareInterface::send_enable_cmd(uint8_t motor_id)
{
  uint8_t data[1] = { motor_id };
  return send_command(usb_protocol::CMD_MOTOR_ENABLE, data, sizeof(data));
}

// ===================== 马达初始化 (清错 + 使能) =====================

void ExoHardwareInterface::init_motors()
{
  if (fd_ < 0) return;

  // 清除所有马达错误
  for (uint8_t mid = 1; mid <= static_cast<uint8_t>(NUM_JOINTS); ++mid) {
    uint8_t data[1] = { mid };
    send_command(usb_protocol::CMD_MOTOR_CLEAR_ERROR, data, sizeof(data));
  }

  // 使能所有马达
  for (uint8_t mid = 1; mid <= static_cast<uint8_t>(NUM_JOINTS); ++mid) {
    send_enable_cmd(mid);
  }

  RCLCPP_INFO(rclcpp::get_logger("ExoHardwareInterface"),
              "已初始化 %zu 个马达 (清错 + 使能)", NUM_JOINTS);
}

// ===================== 端口自动检测 =====================
// 策略:
//   1. 若 serial_port_ 不是 "auto", 直接使用
//   2. 若 serial_port_ == "auto", 扫描 /dev/ttyACM* 和 /dev/ttyUSB*
//   3. 对每个候选端口尝试打开并读取, 验证是否有合法 0x15 反馈帧

std::string ExoHardwareInterface::find_stm32_port()
{
  // 非 auto 模式: 直接返回配置的路径
  if (serial_port_ != "auto") {
    return serial_port_;
  }

  // auto 模式: 扫描常见 CDC ACM 设备
  const char * patterns[] = {"/dev/ttyACM*", "/dev/ttyUSB*"};
  std::vector<std::string> candidates;

  for (const auto & pattern : patterns) {
    glob_t glob_result{};
    int ret = glob(pattern, GLOB_NOSORT, nullptr, &glob_result);
    if (ret == 0) {
      for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
        candidates.push_back(glob_result.gl_pathv[i]);
      }
      globfree(&glob_result);
    }
  }

  if (candidates.empty()) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("ExoHardwareInterface"),
      clock_, 5000,
      "端口自动检测: 未找到 /dev/ttyACM* 或 /dev/ttyUSB* 设备, 请检查 USB 连接");
    return "";
  }

  // 如果之前成功连接过某个端口, 优先尝试它
  if (!current_port_.empty()) {
    auto it = std::find(candidates.begin(), candidates.end(), current_port_);
    if (it != candidates.end()) {
      candidates.erase(it);
      candidates.insert(candidates.begin(), current_port_);
    }
  }

  // 逐一尝试: 打开端口, 读取 200ms, 看是否出现合法 0x15 帧
  for (const auto & port : candidates) {
    int test_fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (test_fd < 0) continue;

    // 配置串口并验证 (要求 200ms 内 ≥50 帧合法反馈, 帧率 ≥250Hz)
    int valid_frames = 0;
    int first_bus = -1, first_motor = -1;
    struct termios tty{};
    if (tcgetattr(test_fd, &tty) == 0) {
      cfsetispeed(&tty, B115200);
      cfsetospeed(&tty, B115200);
      tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
      tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
      tty.c_cflag |= (CLOCAL | CREAD);
      tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP
                       | INLCR | IGNCR | ICRNL);
      tty.c_oflag &= ~OPOST;
      tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
      tty.c_cc[VMIN] = 0;
      tty.c_cc[VTIME] = 0;
      tcsetattr(test_fd, TCSANOW, &tty);
      tcflush(test_fd, TCIOFLUSH);

      auto start = std::chrono::steady_clock::now();
      auto timeout = std::chrono::milliseconds(200);

      enum { S_IDLE, S_GOT_55, S_GOT_CMD, S_RECV_DATA } state = S_IDLE;
      uint8_t rx_cmd = 0, rx_len = 0, rx_idx = 0;
      uint8_t rx_buf[256];

      while (std::chrono::steady_clock::now() - start < timeout) {
        uint8_t byte;
        ssize_t n = ::read(test_fd, &byte, 1);
        if (n <= 0) { usleep(1000); continue; }

        switch (state) {
          case S_IDLE:
            if (byte == usb_protocol::STX_ACK) state = S_GOT_CMD;
            break;
          case S_GOT_CMD:
            rx_cmd = byte; state = S_GOT_55; break;
          case S_GOT_55:
            rx_len = byte; rx_idx = 0;
            state = (rx_len < 250) ? S_RECV_DATA : S_IDLE;
            break;
          case S_RECV_DATA:
            rx_buf[rx_idx++] = byte;
            if (rx_idx >= rx_len + 1) {
              state = S_IDLE;
              uint8_t calc = calc_checksum(rx_cmd, rx_len, rx_buf);
              if (calc != rx_buf[rx_len]) break;
              if (rx_cmd != usb_protocol::CMD_MOTOR_RAW_FEEDBACK) break;
              if (rx_len < 2 || rx_buf[0] == 0 || rx_buf[1] == 0) break;
              // 旧固件只有基础反馈，新固件会在其后附加诊断尾部。
              // 自动识别只要求基础字段完整，避免拒绝向后兼容的扩展帧。
              uint8_t minimum_len = 2 + rx_buf[0] * rx_buf[1] * 10;
              if (rx_len < minimum_len) break;
              if (first_bus < 0) {
                first_bus = rx_buf[0]; first_motor = rx_buf[1];
              } else if (rx_buf[0] != first_bus || rx_buf[1] != first_motor) {
                break;
              }
              valid_frames++;
            }
            break;
        }
      }
    }

    ::close(test_fd);

    if (valid_frames >= 50) {
      RCLCPP_INFO(rclcpp::get_logger("ExoHardwareInterface"),
                  "自动检测到 STM32: %s (%d帧/200ms, %d总线×%d电机)",
                  port.c_str(), valid_frames, first_bus, first_motor);
      return port;
    } else if (valid_frames > 0) {
      RCLCPP_DEBUG(rclcpp::get_logger("ExoHardwareInterface"),
                   "跳过 %s: 仅 %d 帧/200ms (非 STM32)", port.c_str(), valid_frames);
    }
  }

  return "";  // 未找到
}

// ===================== 处理断开 =====================

void ExoHardwareInterface::handle_disconnect()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  connected_ = false;
  hw_commands_.fill(0.0);
  ankle_velocities_.fill(0.0);
  ankle_feedback_valid_.fill(0.0);
  ankle_stale_.fill(1.0);
  ankle_raw_prev_.fill(-1);
  ankle_tick_prev_.fill(0);

  RCLCPP_WARN(rclcpp::get_logger("ExoHardwareInterface"),
              "串口已断开 (%.1fs 无数据), 进入重连模式", data_timeout_);
}

// ===================== 尝试重连 =====================

bool ExoHardwareInterface::try_reconnect()
{
  auto now = clock_.now();

  // 控制重连尝试频率
  if (connected_) return true;
  if ((now - last_reconnect_attempt_).seconds() < reconnect_interval_) {
    return false;
  }
  last_reconnect_attempt_ = now;

  // 查找可用端口
  std::string port = find_stm32_port();
  if (port.empty()) {
    RCLCPP_DEBUG(rclcpp::get_logger("ExoHardwareInterface"),
                 "重连尝试: 未找到 STM32 设备, %.1fs 后重试", reconnect_interval_);
    return false;
  }

  // 打开串口
  if (!open_serial(port)) {
    return false;
  }

  // 配置串口
  if (!configure_serial()) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // 初始化马达
  current_port_ = port;
  connected_ = true;
  last_data_time_ = now;
  init_motors();

  RCLCPP_INFO(rclcpp::get_logger("ExoHardwareInterface"),
              "串口重连成功: %s", port.c_str());
  return true;
}

// ===================== Lifecycle =====================

hardware_interface::CallbackReturn ExoHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  serial_port_ = info.hardware_parameters.count("serial_port")
    ? info.hardware_parameters.at("serial_port") : "auto";

  baud_rate_ = info.hardware_parameters.count("baud_rate")
    ? std::stoi(info.hardware_parameters.at("baud_rate")) : 115200;

  reduction_ratio_ = info.hardware_parameters.count("reduction_ratio")
    ? std::stod(info.hardware_parameters.at("reduction_ratio")) : 36.0;

  torque_limit_ = info.hardware_parameters.count("torque_limit")
    ? std::stod(info.hardware_parameters.at("torque_limit")) : 50.0;

  current_limit_ = info.hardware_parameters.count("current_limit")
    ? std::stod(info.hardware_parameters.at("current_limit")) : 20.0;

  use_mock_ = info.hardware_parameters.count("use_mock")
    ? (info.hardware_parameters.at("use_mock") == "true" ||
       info.hardware_parameters.at("use_mock") == "1") : false;

  ankle_velocity_filter_alpha_ =
    info.hardware_parameters.count("ankle_velocity_filter_alpha") ?
    std::stod(info.hardware_parameters.at("ankle_velocity_filter_alpha")) : 0.2;
  if (ankle_velocity_filter_alpha_ <= 0.0 || ankle_velocity_filter_alpha_ > 1.0) {
    RCLCPP_FATAL(
      rclcpp::get_logger("ExoHardwareInterface"),
      "ankle_velocity_filter_alpha 需在 (0, 1] 内，实际 %.3f",
      ankle_velocity_filter_alpha_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.reserve(info.joints.size());
  for (const auto & joint : info.joints) {
    joint_names_.push_back(joint.name);
  }

  if (joint_names_.size() != NUM_JOINTS) {
    RCLCPP_FATAL(
      rclcpp::get_logger("ExoHardwareInterface"),
      "期望 %zu 个关节，实际 %zu 个", NUM_JOINTS, joint_names_.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info.sensors.size() != NUM_ANKLE_ENCODERS) {
    RCLCPP_FATAL(
      rclcpp::get_logger("ExoHardwareInterface"),
      "期望 %zu 个踝编码器 sensor，实际 %zu 个",
      NUM_ANKLE_ENCODERS, info.sensors.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < NUM_ANKLE_ENCODERS; ++i) {
    if (info.sensors[i].name != ankle_names_[i]) {
      RCLCPP_FATAL(
        rclcpp::get_logger("ExoHardwareInterface"),
        "踝编码器 sensor 顺序应为 [left_ankle, right_ankle]，索引 %zu 实际为 %s",
        i, info.sensors[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // 关节↔电机ID + 方向符号: 从 motor_map_file (YAML) 按关节名加载
  // 无该参数时回退默认 motor_id=i+1, joint_sign=1 (旧行为)
  motor_id_.fill(0);
  joint_sign_.fill(1);
  if (info.hardware_parameters.count("motor_map_file")) {
    const std::string path = info.hardware_parameters.at("motor_map_file");
    if (!load_motor_map(path)) {
      RCLCPP_FATAL(
        rclcpp::get_logger("ExoHardwareInterface"),
        "加载电机映射失败: %s (请检查 exo_controllers.yaml 的 joint_motor_map 段)", path.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else {
    for (size_t i = 0; i < NUM_JOINTS; ++i) { motor_id_[i] = i + 1; }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("ExoHardwareInterface"),
    "on_init: port=%s, baud=%d, reduction=%.1f, "
    "torque_limit=%.1f, current_limit=%.1f, mock=%s, joints=%zu",
    serial_port_.c_str(), baud_rate_, reduction_ratio_,
    torque_limit_, current_limit_, use_mock_ ? "true" : "false", NUM_JOINTS);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ===================== 关节↔电机映射加载 =====================
// 从 YAML (exo_controllers.yaml 的 joint_motor_map 段, 数组形式) 按关节名匹配,
// 加载 motor_id 与 joint_sign。任一关节缺失/非法都返回 false,
// 避免配置错误驱动到错误电机。

bool ExoHardwareInterface::load_motor_map(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ExoHardwareInterface"),
      "motor_map_file 解析失败: %s", e.what());
    return false;
  }

  const YAML::Node map = root["joint_motor_map"]["ros__parameters"];
  if (!map || !map["joints"] || !map["motor_id"]) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ExoHardwareInterface"),
      "motor_map_file 缺少 joint_motor_map.ros__parameters 的 joints/motor_id");
    return false;
  }

  const YAML::Node joints = map["joints"];
  const YAML::Node ids = map["motor_id"];
  const YAML::Node signs = map["joint_sign"];  // 可选, 缺省全 1

  if (!joints.IsSequence() || !ids.IsSequence() ||
      joints.size() != ids.size() ||
      (signs && (!signs.IsSequence() || signs.size() != joints.size())))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("ExoHardwareInterface"),
      "joints/motor_id/joint_sign 需为等长数组");
    return false;
  }

  // 按关节名匹配: 把 joint_names_[i] 在 joints 数组中找到对应下标的 id/sign
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    bool found = false;
    for (size_t j = 0; j < joints.size(); ++j) {
      if (joints[j].as<std::string>() == joint_names_[i]) {
        const uint8_t id = static_cast<uint8_t>(ids[j].as<int>());
        const int sign = signs ? signs[j].as<int>() : 1;
        if (id < 1 || id > NUM_JOINTS) {
          RCLCPP_ERROR(
            rclcpp::get_logger("ExoHardwareInterface"),
            "关节 %s 的 motor_id=%u 越界 (需 1~%zu)",
            joint_names_[i].c_str(), id, NUM_JOINTS);
          return false;
        }
        motor_id_[i] = id;
        joint_sign_[i] = (sign < 0) ? -1 : 1;
        found = true;
        break;
      }
    }
    if (!found) {
      RCLCPP_ERROR(
        rclcpp::get_logger("ExoHardwareInterface"),
        "关节 %s 不在 joint_motor_map.joints 中", joint_names_[i].c_str());
      return false;
    }
  }

  const YAML::Node ankle_map = root["ankle_encoder_map"]["ros__parameters"];
  if (!ankle_map || !ankle_map["joints"] || !ankle_map["node_id"] ||
      !ankle_map["joint_sign"] || !ankle_map["zero_offset_counts"])
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("ExoHardwareInterface"),
      "motor_map_file 缺少 ankle_encoder_map 的 joints/node_id/joint_sign/zero_offset_counts");
    return false;
  }
  const YAML::Node ankle_joints = ankle_map["joints"];
  const YAML::Node ankle_ids = ankle_map["node_id"];
  const YAML::Node ankle_signs = ankle_map["joint_sign"];
  const YAML::Node ankle_zeros = ankle_map["zero_offset_counts"];
  if (!ankle_joints.IsSequence() || !ankle_ids.IsSequence() ||
      !ankle_signs.IsSequence() || !ankle_zeros.IsSequence() ||
      ankle_joints.size() != NUM_ANKLE_ENCODERS ||
      ankle_ids.size() != NUM_ANKLE_ENCODERS ||
      ankle_signs.size() != NUM_ANKLE_ENCODERS ||
      ankle_zeros.size() != NUM_ANKLE_ENCODERS)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("ExoHardwareInterface"),
      "ankle_encoder_map 的四个数组必须均为长度 %zu", NUM_ANKLE_ENCODERS);
    return false;
  }
  for (size_t i = 0; i < NUM_ANKLE_ENCODERS; ++i) {
    const std::string name = ankle_joints[i].as<std::string>();
    const int id = ankle_ids[i].as<int>();
    const int sign = ankle_signs[i].as<int>();
    const int zero = ankle_zeros[i].as<int>();
    if (name != ankle_names_[i] || id < 1 || id > 255 ||
        (sign != -1 && sign != 1) || zero < 0 ||
        zero >= static_cast<int>(usb_protocol::ANKLE_ENCODER_RESOLUTION))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("ExoHardwareInterface"),
        "非法踝编码器配置[%zu]: name=%s id=%d sign=%d zero=%d",
        i, name.c_str(), id, sign, zero);
      return false;
    }
    ankle_node_id_[i] = static_cast<uint8_t>(id);
    ankle_sign_[i] = static_cast<int8_t>(sign);
    ankle_zero_offset_[i] = static_cast<uint16_t>(zero);
  }
  return true;
}

hardware_interface::CallbackReturn ExoHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hw_positions_.fill(0.0);
  hw_velocities_.fill(0.0);
  hw_efforts_.fill(0.0);
  hw_commands_.fill(0.0);
  hw_zero_position_commands_.fill(0.0);
  ankle_positions_.fill(0.0);
  ankle_velocities_.fill(0.0);
  ankle_raw_counts_.fill(0.0);
  ankle_node_ids_.fill(0.0);
  ankle_feedback_valid_.fill(0.0);
  ankle_stale_.fill(1.0);
  ankle_raw_prev_.fill(-1);
  ankle_tick_prev_.fill(0);
  enc_raw_prev_.fill(-1);
  enc_continuous_.fill(0);
  enc_zero_offset_.fill(0);
  zero_captured_ = false;
  rx_state_ = RxState::WAIT_STX;
  rx_idx_ = 0;
  expected_len_ = 0;
  connected_ = false;
  current_port_.clear();
  last_data_time_ = clock_.now();
  // 初始化到过去时刻, 使 on_configure 的首次 try_reconnect() 不被间隔检查跳过
  last_reconnect_attempt_ = rclcpp::Time(0);

  if (use_mock_) {
    RCLCPP_WARN(rclcpp::get_logger("ExoHardwareInterface"), "仿真模式: 串口未打开");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // 尝试首次连接 (失败不阻塞, 后续 read() 会重连)
  try_reconnect();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ExoHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  handle_disconnect();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ExoHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 重置编码器解绕/零位: 每次激活后首个有效反馈重新捕获零位 (软件归零)
  enc_raw_prev_.fill(-1);
  enc_continuous_.fill(0);
  enc_zero_offset_.fill(0);
  zero_captured_ = false;
  ankle_velocities_.fill(0.0);
  ankle_feedback_valid_.fill(0.0);
  ankle_stale_.fill(1.0);
  ankle_raw_prev_.fill(-1);
  ankle_tick_prev_.fill(0);
  hw_zero_position_commands_.fill(0.0);

  // 若未连接, 再尝试一次
  if (!connected_ && !use_mock_) {
    try_reconnect();
  }

  if (connected_) {
    RCLCPP_INFO(rclcpp::get_logger("ExoHardwareInterface"),
                "硬件已激活 (已连接: %s)", current_port_.c_str());
  } else if (use_mock_) {
    RCLCPP_INFO(rclcpp::get_logger("ExoHardwareInterface"), "硬件已激活 (仿真模式)");
  } else {
    RCLCPP_WARN(rclcpp::get_logger("ExoHardwareInterface"),
                "硬件已激活 (等待 STM32 连接, 端口模式: %s)", serial_port_.c_str());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ExoHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 归零所有马达力矩
  if (connected_ && fd_ >= 0) {
    for (uint8_t mid = 1; mid <= static_cast<uint8_t>(NUM_JOINTS); ++mid) {
      uint8_t data[6];
      data[0] = mid;
      data[1] = usb_protocol::MODE_TORQUE;
      float iq_zero = 0.0f;
      std::memcpy(data + 2, &iq_zero, sizeof(float));
      send_command(usb_protocol::CMD_MOTOR_SET_TARGET, data, sizeof(data));
    }
  }

  hw_commands_.fill(0.0);
  hw_zero_position_commands_.fill(0.0);
  RCLCPP_INFO(rclcpp::get_logger("ExoHardwareInterface"), "硬件已去激活，力矩归零");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ===================== 接口导出 =====================

std::vector<hardware_interface::StateInterface>
ExoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]);
    interfaces.emplace_back(
      joint_names_[i], "temperature", &hw_temperatures_[i]);
    interfaces.emplace_back(
      joint_names_[i], "error_state", &hw_error_states_[i]);
    interfaces.emplace_back(
      joint_names_[i], "feedback_age_ms", &hw_feedback_ages_ms_[i]);
    interfaces.emplace_back(
      joint_names_[i], "feedback_valid", &hw_feedback_valid_[i]);
    interfaces.emplace_back(
      joint_names_[i], "stm32_tick_ms", &hw_stm32_ticks_ms_[i]);
    interfaces.emplace_back(
      joint_names_[i], "feedback_sequence", &hw_feedback_sequences_[i]);
  }

  for (size_t i = 0; i < NUM_ANKLE_ENCODERS; ++i) {
    interfaces.emplace_back(
      ankle_names_[i], hardware_interface::HW_IF_POSITION, &ankle_positions_[i]);
    interfaces.emplace_back(
      ankle_names_[i], hardware_interface::HW_IF_VELOCITY, &ankle_velocities_[i]);
    interfaces.emplace_back(
      ankle_names_[i], "raw_count", &ankle_raw_counts_[i]);
    interfaces.emplace_back(
      ankle_names_[i], "node_id", &ankle_node_ids_[i]);
    interfaces.emplace_back(
      ankle_names_[i], "feedback_valid", &ankle_feedback_valid_[i]);
    interfaces.emplace_back(
      ankle_names_[i], "stale", &ankle_stale_[i]);
  }

  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
ExoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;

  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_EFFORT, &hw_commands_[i]);
  }
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    interfaces.emplace_back(
      joint_names_[i], "zero_position", &hw_zero_position_commands_[i]);
  }

  return interfaces;
}

// ===================== 串口操作 =====================

bool ExoHardwareInterface::open_serial(const std::string & port)
{
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ExoHardwareInterface"),
      "无法打开串口 %s: %s", port.c_str(), std::strerror(errno));
    return false;
  }
  return true;
}

void ExoHardwareInterface::close_serial()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool ExoHardwareInterface::configure_serial()
{
  struct termios tty {};
  if (tcgetattr(fd_, &tty) != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ExoHardwareInterface"),
      "tcgetattr 失败: %s", std::strerror(errno));
    return false;
  }

  speed_t speed;
  switch (baud_rate_) {
    case 9600:   speed = B9600;   break;
    case 19200:  speed = B19200;  break;
    case 38400:  speed = B38400;  break;
    case 57600:  speed = B57600;  break;
    case 115200: speed = B115200; break;
    case 230400: speed = B230400; break;
    case 460800: speed = B460800; break;
    case 921600: speed = B921600; break;
    default:
      RCLCPP_ERROR(
        rclcpp::get_logger("ExoHardwareInterface"),
        "不支持的波特率: %d", baud_rate_);
      return false;
  }
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  const auto CSIZE_M = static_cast<tcflag_t>(CSIZE);
  const auto CSTOP_M = static_cast<tcflag_t>(PARENB | CSTOPB | CRTSCTS);
  const auto CSET_M  = static_cast<tcflag_t>(CLOCAL | CREAD);
  const auto IFLAG_M = static_cast<tcflag_t>(
    IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP |
    INLCR | IGNCR | ICRNL);
  const auto OFLAG_M = static_cast<tcflag_t>(OPOST);
  const auto LFLAG_M = static_cast<tcflag_t>(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

  tty.c_cflag = (tty.c_cflag & ~CSIZE_M) | CS8;
  tty.c_cflag &= ~CSTOP_M;
  tty.c_cflag |= CSET_M;
  tty.c_iflag &= ~IFLAG_M;
  tty.c_oflag &= ~OFLAG_M;
  tty.c_lflag &= ~LFLAG_M;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ExoHardwareInterface"),
      "tcsetattr 失败: %s", std::strerror(errno));
    return false;
  }

  tcflush(fd_, TCIOFLUSH);
  return true;
}

// ===================== 反馈数据解析 =====================
// CMD_MOTOR_RAW_FEEDBACK (0x15) 数据布局:
//   bus_count(1B) | motors_per_bus(1B) |
//   [encoder(u16 LE) | speed(i16 LE) | iq(i16 LE) | multi_turn(i32 LE)] × N
//   每马达 10 字节 | 可选电机诊断(18B) |
//   可选 [node_id(u8) | flags(u8) | raw_count(u16 LE)] × 2 踝编码器

bool ExoHardwareInterface::parse_feedback_data(
  const uint8_t * data, uint8_t len,
  std::vector<MotorFeedback> & motors, FeedbackMetadata & metadata)
{
  motors.clear();
  metadata = FeedbackMetadata{};
  if (len < 2) return false;

  uint8_t bus_count = data[0];
  uint8_t motors_per_bus = data[1];
  size_t total_motors = static_cast<size_t>(bus_count) * motors_per_bus;

  if (total_motors == 0) return false;
  if (len < static_cast<uint8_t>(2 + total_motors * 10)) return false;

  motors.resize(total_motors);
  const uint8_t * ptr = data + 2;

  for (size_t i = 0; i < total_motors; ++i) {
    MotorFeedback & m = motors[i];
    m.encoder    = *reinterpret_cast<const uint16_t *>(ptr);      ptr += 2;
    m.speed_dps  = *reinterpret_cast<const int16_t *>(ptr);       ptr += 2;
    m.iq         = *reinterpret_cast<const int16_t *>(ptr);       ptr += 2;
    m.multi_turn = *reinterpret_cast<const int32_t *>(ptr);       ptr += 4;
  }

  // 可选诊断尾部 (18B): sequence(u16) | stm32_tick_ms(u32) |
  // [temperature(i8), error(u8), age_ms(u8)] x 4。
  // 基础 10B/电机布局保持不变，因此仍兼容没有尾部的旧固件。
  constexpr size_t extension_size = 18;
  const size_t extension_offset = 2 + total_motors * 10;
  if (total_motors == NUM_JOINTS &&
      len >= extension_offset + extension_size)
  {
    const uint8_t * ext = data + extension_offset;
    metadata.extension_valid = true;
    metadata.sequence = static_cast<uint16_t>(ext[0]) |
      (static_cast<uint16_t>(ext[1]) << 8);
    metadata.stm32_tick_ms = static_cast<uint32_t>(ext[2]) |
      (static_cast<uint32_t>(ext[3]) << 8) |
      (static_cast<uint32_t>(ext[4]) << 16) |
      (static_cast<uint32_t>(ext[5]) << 24);
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
      const size_t offset = 6 + i * 3;
      metadata.temperature_c[i] = static_cast<int8_t>(ext[offset]);
      metadata.error_state[i] = ext[offset + 1];
      metadata.feedback_age_ms[i] = ext[offset + 2];
    }

    constexpr size_t ankle_extension_size =
      NUM_ANKLE_ENCODERS * sizeof(AnkleFeedback);
    if (len >= extension_offset + extension_size + ankle_extension_size) {
      const uint8_t * ankle_ext = ext + extension_size;
      metadata.ankle_extension_valid = true;
      for (size_t i = 0; i < NUM_ANKLE_ENCODERS; ++i) {
        const size_t offset = i * sizeof(AnkleFeedback);
        metadata.ankles[i].node_id = ankle_ext[offset];
        metadata.ankles[i].flags = ankle_ext[offset + 1];
        metadata.ankles[i].raw_count =
          static_cast<uint16_t>(ankle_ext[offset + 2]) |
          (static_cast<uint16_t>(ankle_ext[offset + 3]) << 8);
      }
    }
  }

  return true;
}

// ===================== 帧接收状态机 =====================
// 反馈帧格式: STX_ACK(0x55) | CMD | LEN | DATA... | CHK
//
// 返回 true: 成功解析一帧反馈数据
// 返回 false: 串口无更多数据 (read 返回 0/EAGAIN)

bool ExoHardwareInterface::recv_feedback(
  std::vector<MotorFeedback> & motors, FeedbackMetadata & metadata)
{
  motors.clear();
  metadata = FeedbackMetadata{};
  uint8_t byte;

  while (true) {
    ssize_t n = ::read(fd_, &byte, 1);
    if (n <= 0) {
      return false;  // 串口缓冲区内无更多数据
    }

    switch (rx_state_) {
      case RxState::WAIT_STX:
        if (byte == usb_protocol::STX_ACK) {
          rx_buf_[0] = byte;
          rx_idx_ = 1;
          rx_state_ = RxState::WAIT_CMD;
        }
        break;

      case RxState::WAIT_CMD:
        rx_cmd_ = byte;
        rx_buf_[1] = byte;
        rx_idx_ = 2;
        rx_state_ = RxState::WAIT_LEN;
        break;

      case RxState::WAIT_LEN:
        expected_len_ = byte;
        rx_buf_[2] = byte;
        rx_idx_ = 3;
        if (expected_len_ < 250) {
          rx_state_ = RxState::RECV_DATA;
        } else {
          rx_state_ = RxState::WAIT_STX;
        }
        break;

      case RxState::RECV_DATA:
        rx_buf_[rx_idx_++] = byte;
        if (rx_idx_ >= static_cast<size_t>(3 + expected_len_ + 1)) {
          rx_state_ = RxState::WAIT_STX;

          const uint8_t * data_ptr = rx_buf_.data() + 3;
          uint8_t calc_chk = calc_checksum(rx_cmd_, expected_len_, data_ptr);
          uint8_t recv_chk = rx_buf_[3 + expected_len_];

          if (calc_chk != recv_chk) {
            continue;  // 丢弃坏帧
          }

          if (rx_cmd_ != usb_protocol::CMD_MOTOR_RAW_FEEDBACK) {
            continue;
          }

          if (parse_feedback_data(data_ptr, expected_len_, motors, metadata)) {
            return true;
          }
        }
        break;
    }
  }
}

// ===================== read() =====================

hardware_interface::return_type ExoHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (use_mock_) {
    return hardware_interface::return_type::OK;
  }

  // 热插拔计时统一使用成员clock_，避免controller_manager传入的ROS时钟
  // 与系统时钟相减导致异常。
  const rclcpp::Time now = clock_.now();

  // 断线时尝试重连
  if (!connected_) {
    try_reconnect();
    if (!connected_) {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("ExoHardwareInterface"),
        clock_, 5000,
        "等待 STM32 连接 (端口模式: %s)... 已等待 %.0f 秒",
        serial_port_.c_str(),
        (now - last_data_time_).seconds());
      return hardware_interface::return_type::OK;
    }
  }

  if (fd_ < 0) {
    handle_disconnect();
    return hardware_interface::return_type::OK;
  }

  // 持续读取直到串口缓冲区清空
  std::vector<MotorFeedback> motors;
  FeedbackMetadata metadata;
  bool got_data = false;
  bool got_complete_frame = false;
  while (recv_feedback(motors, metadata)) {
    got_data = true;
    bool frame_complete = true;
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
      // 反馈帧按电机号升序 [motor1..motor4], 按 motor_id_ 取对应反馈
      const size_t mi = static_cast<size_t>(motor_id_[i] - 1);
      if (mi >= motors.size()) {
        frame_complete = false;
        continue;
      }
      const MotorFeedback & m = motors[mi];

      // 位置反馈: 输出轴单圈编码器 (0~65535 = 0~360°, 手册 §2.6, 始终有效)。
      // multi_turn 字段受固件 MOTOR_POLL_MULTI_ANG 开关控制 (默认关闭→恒0), 不使用。
      const int32_t enc_raw = m.encoder;
      if (enc_raw_prev_[i] >= 0)
      {
        const int32_t enc_diff = enc_raw - enc_raw_prev_[i];
        if (enc_diff > 32768) { enc_continuous_[i] -= 65536; }
        else if (enc_diff < -32768) { enc_continuous_[i] += 65536; }
      }
      enc_raw_prev_[i] = enc_raw;
      const int64_t enc_cont = enc_continuous_[i] + enc_raw;

      // 激活后首个有效反馈捕获零位 → 位置从 0 开始 (软件归零)
      if (!zero_captured_)
      {
        enc_zero_offset_[i] = enc_cont;
      }

      // joint_sign: 电机方向与 URDF 反向的关节设为 -1, 位置/速度/力矩一起翻转
      hw_positions_[i] = joint_sign_[i] *
        (static_cast<double>(enc_cont - enc_zero_offset_[i])
        * (2.0 * M_PI / static_cast<double>(usb_protocol::ENCODER_RESOLUTION)));

      hw_velocities_[i] = joint_sign_[i] *
        (static_cast<double>(m.speed_dps)
        / reduction_ratio_
        * usb_protocol::DPS_TO_RADS);

      hw_efforts_[i] = joint_sign_[i] * static_cast<double>(m.iq);

      if (metadata.extension_valid) {
        hw_temperatures_[i] = static_cast<double>(metadata.temperature_c[mi]);
        hw_error_states_[i] = static_cast<double>(metadata.error_state[mi]);
        hw_feedback_ages_ms_[i] = static_cast<double>(metadata.feedback_age_ms[mi]);
        hw_feedback_valid_[i] =
          metadata.feedback_age_ms[mi] < 255U ? 1.0 : 0.0;
        hw_stm32_ticks_ms_[i] = static_cast<double>(metadata.stm32_tick_ms);
        hw_feedback_sequences_[i] = static_cast<double>(metadata.sequence);
      }
    }
    got_complete_frame = got_complete_frame || frame_complete;

    if (metadata.ankle_extension_valid) {
      for (size_t i = 0; i < NUM_ANKLE_ENCODERS; ++i) {
        const AnkleFeedback & ankle = metadata.ankles[i];
        const bool id_matches = ankle.node_id == ankle_node_id_[i];
        const bool valid = (ankle.flags & 0x01U) != 0U;
        const bool stale = (ankle.flags & 0x02U) != 0U;
        const bool count_in_range =
          ankle.raw_count < usb_protocol::ANKLE_ENCODER_RESOLUTION;
        ankle_node_ids_[i] = static_cast<double>(ankle.node_id);
        ankle_raw_counts_[i] = static_cast<double>(ankle.raw_count);
        ankle_feedback_valid_[i] =
          (id_matches && valid && !stale && count_in_range) ? 1.0 : 0.0;
        ankle_stale_[i] = stale ? 1.0 : 0.0;

        if (!id_matches) {
          RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("ExoHardwareInterface"), clock_, 2000,
            "%s 节点ID不匹配: 期望 %u，实际 %u",
            ankle_names_[i].c_str(), ankle_node_id_[i], ankle.node_id);
          continue;
        }
        if (!valid || stale || !count_in_range)
        {
          continue;
        }

        const int32_t corrected_count =
          ankle_count_from_zero(ankle.raw_count, ankle_zero_offset_[i]);
        ankle_positions_[i] = static_cast<double>(ankle_sign_[i]) *
          static_cast<double>(corrected_count) *
          (2.0 * M_PI / static_cast<double>(usb_protocol::ANKLE_ENCODER_RESOLUTION));

        if (ankle_raw_prev_[i] >= 0 && ankle_tick_prev_[i] != metadata.stm32_tick_ms) {
          const int32_t delta_count = ankle_count_from_zero(
            ankle.raw_count, static_cast<uint16_t>(ankle_raw_prev_[i]));
          const uint32_t delta_ms = metadata.stm32_tick_ms - ankle_tick_prev_[i];
          if (delta_ms > 0U) {
            const double raw_velocity = static_cast<double>(ankle_sign_[i]) *
              static_cast<double>(delta_count) *
              (2.0 * M_PI / static_cast<double>(usb_protocol::ANKLE_ENCODER_RESOLUTION)) *
              (1000.0 / static_cast<double>(delta_ms));
            ankle_velocities_[i] += ankle_velocity_filter_alpha_ *
              (raw_velocity - ankle_velocities_[i]);
          }
        }
        ankle_raw_prev_[i] = ankle.raw_count;
        ankle_tick_prev_[i] = metadata.stm32_tick_ms;
      }
    } else {
      ankle_feedback_valid_.fill(0.0);
      ankle_stale_.fill(1.0);
    }
  }

  if (got_data) {
    last_data_time_ = now;
  }

  // 首个有效反馈帧: 确认零位已捕获
  if (got_complete_frame && !zero_captured_)
  {
    zero_captured_ = true;
    RCLCPP_INFO(
      rclcpp::get_logger("ExoHardwareInterface"),
      "已在上位机捕获编码器零位 (未修改STM32或电机参数)");
  }

  // 断线检测: 超过 data_timeout_ 无数据
  if ((now - last_data_time_).seconds() > data_timeout_) {
    handle_disconnect();
  }

  return hardware_interface::return_type::OK;
}

// ===================== write() =====================

hardware_interface::return_type ExoHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (use_mock_) {
    return hardware_interface::return_type::OK;
  }

  // 未连接时静默跳过
  if (!connected_ || fd_ < 0) {
    return hardware_interface::return_type::OK;
  }

  const bool zero_requested = std::any_of(
    hw_zero_position_commands_.begin(), hw_zero_position_commands_.end(),
    [](double value) {return value > 0.5;});
  if (zero_requested)
  {
    const bool feedback_ready = zero_captured_ && std::all_of(
      enc_raw_prev_.begin(), enc_raw_prev_.end(),
      [](int32_t value) {return value >= 0;});
    if (feedback_ready)
    {
      for (size_t i = 0; i < NUM_JOINTS; ++i)
      {
        enc_zero_offset_[i] = enc_continuous_[i] + enc_raw_prev_[i];
        hw_positions_[i] = 0.0;
      }
      RCLCPP_INFO(
        rclcpp::get_logger("ExoHardwareInterface"),
        "已更新上位机编码器偏置，四关节当前位置为0 (未修改下位机)");
    }
    else
    {
      RCLCPP_WARN(
        rclcpp::get_logger("ExoHardwareInterface"),
        "忽略手动清零: 尚未获得四关节完整编码器反馈");
    }
    hw_zero_position_commands_.fill(0.0);
  }

  // 对每个关节发送力矩控制指令
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    uint8_t data[6];
    data[0] = motor_id_[i];  // motor_id 1-indexed (来自 exo_motor_map.yaml)
    data[1] = usb_protocol::MODE_TORQUE;

    double cmd = std::clamp(hw_commands_[i], -torque_limit_, torque_limit_);
    // joint_sign: 电机方向反向的关节, 力矩指令也翻转 (与 read() 反馈一致)
    float iq_val = static_cast<float>(joint_sign_[i] * cmd);
    std::memcpy(data + 2, &iq_val, sizeof(float));

    send_command(usb_protocol::CMD_MOTOR_SET_TARGET, data, sizeof(data));
  }

  return hardware_interface::return_type::OK;
}

}  // namespace exoskeleton_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  exoskeleton_hardware::ExoHardwareInterface,
  hardware_interface::SystemInterface)

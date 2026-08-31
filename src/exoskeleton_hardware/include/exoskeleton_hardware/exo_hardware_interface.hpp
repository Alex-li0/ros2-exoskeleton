#ifndef EXOSKELETON_HARDWARE__EXO_HARDWARE_INTERFACE_HPP_
#define EXOSKELETON_HARDWARE__EXO_HARDWARE_INTERFACE_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace exoskeleton_hardware
{

static constexpr size_t NUM_JOINTS = 4;  // 左/右 hip3 + knee

// ===================== USB 串口协议常量 =====================
// STM32 ↔ PC 通信协议, 参考 usb_protocol_and_test_report.pdf

namespace usb_protocol
{
// 帧头
constexpr uint8_t STX_REQ = 0xAA;  // PC → STM32 命令帧
constexpr uint8_t STX_ACK = 0x55;  // STM32 → PC 反馈帧

// 命令字
constexpr uint8_t CMD_PING              = 0x01;
constexpr uint8_t CMD_MOTOR_SET_TARGET  = 0x11;
constexpr uint8_t CMD_MOTOR_ENABLE      = 0x12;
constexpr uint8_t CMD_MOTOR_CALIBRATE    = 0x13;
constexpr uint8_t CMD_MOTOR_RAW_FEEDBACK = 0x15;
constexpr uint8_t CMD_MOTOR_CLEAR_ERROR  = 0x9B;

// 控制模式 (CMD_MOTOR_SET_TARGET 的 mode 字段)
constexpr uint8_t MODE_POSITION = 0;  // 位置控制, value = rad
constexpr uint8_t MODE_SPEED    = 1;  // 速度控制, value = rad/s + 2B iq_limit
constexpr uint8_t MODE_TORQUE   = 2;  // 力矩控制, value = iq (float)

// 编码器分辨率: 16-bit 单圈
constexpr uint32_t ENCODER_RESOLUTION = 65536;

// 多圈角度分辨率: 0.01 deg/LSB
constexpr double MULTI_TURN_TO_DEG = 0.01;

// 速度: 1 dps/LSB (电机侧)
constexpr double DPS_TO_RADS = M_PI / 180.0;
}  // namespace usb_protocol

// ===================== 反馈帧数据 =====================

struct __attribute__((packed)) MotorFeedback
{
  uint16_t encoder;        // 单圈编码器 (0-65535), angle_deg = enc * 360/65536
  int16_t  speed_dps;      // 电机侧速度 (dps)
  int16_t  iq;             // 力矩电流
  int32_t  multi_turn;     // 多圈绝对角度 (0.01 deg/LSB)
};

// ===================== 接收状态机 =====================

enum class RxState : uint8_t
{
  WAIT_STX,     // 等待 0x55
  WAIT_CMD,     // 等待命令字
  WAIT_LEN,     // 等待长度
  RECV_DATA,    // 接收数据和校验
};

// ===================== 硬件接口类 =====================

class ExoHardwareInterface : public hardware_interface::SystemInterface
{
public:
  // ---- Lifecycle ----
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // XOR 校验 (public for testing): chk = CMD ^ LEN ^ data[0] ^ ... ^ data[N-1]
  static uint8_t calc_checksum(uint8_t cmd, uint8_t len, const uint8_t * data);

private:
  bool open_serial(const std::string & port);
  void close_serial();
  bool configure_serial();

  // 发送命令帧: STX_REQ | CMD | LEN | DATA... | CHK
  bool send_command(uint8_t cmd, const uint8_t * data, uint8_t len);

  // 发送马达使能命令
  bool send_enable_cmd(uint8_t motor_id);

  // 接收一帧反馈数据
  bool recv_feedback(std::vector<MotorFeedback> & motors);

  // ---- 热插拔 ----
  // 扫描 /dev/ttyACM* 找到 STM32 设备, 验证后返回路径
  std::string find_stm32_port();

  // 尝试重连: 扫描→打开→配置→清错→使能
  bool try_reconnect();

  // 从 motor_map_file (YAML) 加载 关节↔电机ID + 方向符号
  // 失败返回 false (配置错误, 不驱动错误电机)
  bool load_motor_map(const std::string & path);

  // 断开时清理并准备重连
  void handle_disconnect();

  // 初始化连接(首次或重连成功): 清除错误 + 使能马达
  void init_motors();

  // ---- 参数 ----
  std::string serial_port_;      // 端口: "auto" 或具体路径如 /dev/ttyACM0
  int baud_rate_;
  double reduction_ratio_;       // 减速比
  double torque_limit_;          // 力矩限幅
  double current_limit_;         // 电流限幅 [A]
  bool use_mock_{false};         // 仿真模式
  std::array<int8_t, NUM_JOINTS> joint_sign_{};  // 关节方向符号 ±1 (来自 exo_motor_map.yaml)
  std::array<uint8_t, NUM_JOINTS> motor_id_{};   // 关节→电机ID (来自 exo_motor_map.yaml)

  // 节流日志用时钟 (成员避免 RT 线程中临时 Clock 悬垂/堆分配)
  rclcpp::Clock clock_;

  // ---- 热插拔状态 ----
  bool connected_{false};
  rclcpp::Time last_data_time_;
  rclcpp::Time last_reconnect_attempt_;
  double data_timeout_{0.2};        // 断线判定超时 [s]
  double reconnect_interval_{1.0};  // 重连尝试间隔 [s]
  std::string current_port_;        // 实际连接的端口路径

  // ---- 串口 ----
  int fd_{-1};

  // ---- 接收状态机 ----
  RxState rx_state_{RxState::WAIT_STX};
  std::array<uint8_t, 256> rx_buf_{};
  size_t rx_idx_{0};
  uint8_t rx_cmd_{0};
  uint8_t expected_len_{0};

  // ---- 状态接口存储 ----
  std::array<double, NUM_JOINTS> hw_positions_{};   // rad (输出轴单圈编码器解绕+软件归零)
  std::array<double, NUM_JOINTS> hw_velocities_{};  // rad/s (输出轴)
  std::array<double, NUM_JOINTS> hw_efforts_{};     // A (电机 iq 电流反馈)

  // ---- 单圈编码器解绕 + 软件归零 ----
  std::array<int32_t, NUM_JOINTS> enc_raw_prev_{};    // 上一帧原始编码器 (首次为 -1)
  std::array<int64_t, NUM_JOINTS> enc_continuous_{};  // 解绕后的连续计数偏移
  std::array<int64_t, NUM_JOINTS> enc_zero_offset_{}; // 激活后首帧捕获的零位计数
  bool zero_captured_{false};                         // 是否已捕获零位

  // ---- 命令接口存储 ----
  std::array<double, NUM_JOINTS> hw_commands_{};    // 上层力矩指令

  // ---- 关节名称 ----
  std::vector<std::string> joint_names_;
};

}  // namespace exoskeleton_hardware

#endif  // EXOSKELETON_HARDWARE__EXO_HARDWARE_INTERFACE_HPP_

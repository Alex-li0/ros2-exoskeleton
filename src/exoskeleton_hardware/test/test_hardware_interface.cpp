#include <gtest/gtest.h>

#include <cstring>
#include <cmath>

#include "exoskeleton_hardware/exo_hardware_interface.hpp"

// ===================== XOR 校验测试 =====================

TEST(ChecksumTest, EmptyData)
{
  // chk = CMD ^ LEN = 0x15 ^ 0x00 = 0x15
  uint8_t chk = exoskeleton_hardware::ExoHardwareInterface::calc_checksum(
    exoskeleton_hardware::usb_protocol::CMD_MOTOR_RAW_FEEDBACK, 0, nullptr);
  EXPECT_EQ(chk, 0x15);
}

TEST(ChecksumTest, SingleByte)
{
  // chk = CMD ^ LEN ^ data[0] = 0x11 ^ 0x01 ^ 0x01 = 0x11
  uint8_t data[] = {0x01};
  uint8_t chk = exoskeleton_hardware::ExoHardwareInterface::calc_checksum(0x11, 1, data);
  EXPECT_EQ(chk, 0x11 ^ 0x01 ^ 0x01);
}

TEST(ChecksumTest, KnownVector)
{
  // chk = 0x11 ^ 0x06 ^ 0x01 ^ 0x02 ^ 0x00 ^ 0x00 ^ 0x00 ^ 0x41
  // = 0x11 ^ 0x06 ^ 0x01 ^ 0x02 ^ 0x00 ^ 0x00 ^ 0x00 ^ 0x41
  // 0x11 ^ 0x06 = 0x17
  // 0x17 ^ 0x01 = 0x16
  // 0x16 ^ 0x02 = 0x14
  // 0x14 ^ 0x00 ^ 0x00 ^ 0x00 = 0x14
  // 0x14 ^ 0x41 = 0x55
  uint8_t data[] = {0x01, 0x02, 0x00, 0x00, 0x00, 0x41};
  uint8_t chk = exoskeleton_hardware::ExoHardwareInterface::calc_checksum(0x11, 6, data);
  EXPECT_EQ(chk, 0x55);
}

TEST(ChecksumTest, Deterministic)
{
  uint8_t data[] = {0xAA, 0xBB, 0xCC};
  uint8_t chk1 = exoskeleton_hardware::ExoHardwareInterface::calc_checksum(0x15, 3, data);
  uint8_t chk2 = exoskeleton_hardware::ExoHardwareInterface::calc_checksum(0x15, 3, data);
  EXPECT_EQ(chk1, chk2);
}

TEST(ChecksumTest, DifferentInputsDiffer)
{
  uint8_t a[] = {0x01, 0x02, 0x03};
  uint8_t b[] = {0x01, 0x02, 0x04};
  EXPECT_NE(
    exoskeleton_hardware::ExoHardwareInterface::calc_checksum(0x11, 3, a),
    exoskeleton_hardware::ExoHardwareInterface::calc_checksum(0x11, 3, b));
}

TEST(ChecksumTest, DifferentCmdDiffer)
{
  uint8_t data[] = {0x01, 0x02};
  EXPECT_NE(
    exoskeleton_hardware::ExoHardwareInterface::calc_checksum(0x11, 2, data),
    exoskeleton_hardware::ExoHardwareInterface::calc_checksum(0x15, 2, data));
}

// ===================== USB 协议常量测试 =====================

TEST(UsbProtocolTest, StxValues)
{
  EXPECT_EQ(exoskeleton_hardware::usb_protocol::STX_REQ, 0xAA);
  EXPECT_EQ(exoskeleton_hardware::usb_protocol::STX_ACK, 0x55);
}

TEST(UsbProtocolTest, CommandCodes)
{
  EXPECT_EQ(exoskeleton_hardware::usb_protocol::CMD_MOTOR_SET_TARGET, 0x11);
  EXPECT_EQ(exoskeleton_hardware::usb_protocol::CMD_MOTOR_ENABLE, 0x12);
  EXPECT_EQ(exoskeleton_hardware::usb_protocol::CMD_MOTOR_RAW_FEEDBACK, 0x15);
}

TEST(UsbProtocolTest, ControlModes)
{
  EXPECT_EQ(exoskeleton_hardware::usb_protocol::MODE_POSITION, 0);
  EXPECT_EQ(exoskeleton_hardware::usb_protocol::MODE_SPEED, 1);
  EXPECT_EQ(exoskeleton_hardware::usb_protocol::MODE_TORQUE, 2);
}

TEST(UsbProtocolTest, EncoderResolution)
{
  // 16-bit 编码器
  EXPECT_EQ(exoskeleton_hardware::usb_protocol::ENCODER_RESOLUTION, 65536u);
}

TEST(UsbProtocolTest, AnkleEncoderResolution)
{
  EXPECT_EQ(exoskeleton_hardware::usb_protocol::ANKLE_ENCODER_RESOLUTION, 4096u);
}

TEST(UsbProtocolTest, MultiTurnResolution)
{
  // 多圈角度: 0.01 deg/LSB
  EXPECT_DOUBLE_EQ(exoskeleton_hardware::usb_protocol::MULTI_TURN_TO_DEG, 0.01);
}

TEST(UsbProtocolTest, DpsToRads)
{
  EXPECT_DOUBLE_EQ(exoskeleton_hardware::usb_protocol::DPS_TO_RADS, M_PI / 180.0);
}

// ===================== MotorFeedback 结构测试 =====================

TEST(MotorFeedbackTest, Size)
{
  // encoder(2) + speed(2) + iq(2) + multi_turn(4) = 10 bytes
  EXPECT_EQ(sizeof(exoskeleton_hardware::MotorFeedback), 10u);
}

TEST(MotorFeedbackTest, FieldOffsets)
{
  exoskeleton_hardware::MotorFeedback m{};
  auto base = reinterpret_cast<uint8_t *>(&m);
  EXPECT_EQ(reinterpret_cast<uint8_t *>(&m.encoder) - base, 0);
  EXPECT_EQ(reinterpret_cast<uint8_t *>(&m.speed_dps) - base, 2);
  EXPECT_EQ(reinterpret_cast<uint8_t *>(&m.iq) - base, 4);
  EXPECT_EQ(reinterpret_cast<uint8_t *>(&m.multi_turn) - base, 6);
}

TEST(AnkleFeedbackTest, Size)
{
  EXPECT_EQ(sizeof(exoskeleton_hardware::AnkleFeedback), 4u);
}

// ===================== 单位转换测试 =====================

TEST(UnitConversionTest, EncoderToDegrees)
{
  // 16-bit 编码器: angle_deg = enc * 360 / 65536
  EXPECT_NEAR(0.0 * 360.0 / 65536.0, 0.0, 1e-10);
  EXPECT_NEAR(32768.0 * 360.0 / 65536.0, 180.0, 1e-10);
  EXPECT_NEAR(65535.0 * 360.0 / 65536.0, 359.995, 0.01);
}

TEST(UnitConversionTest, MultiTurnToRad)
{
  // multi_turn 原始值 (0.01 deg/LSB) → deg → rad
  // 36000 LSB * 0.01 deg/LSB = 360° = 2π rad
  double multi_raw = 36000.0;
  double deg = multi_raw * exoskeleton_hardware::usb_protocol::MULTI_TURN_TO_DEG;
  double rad = deg * exoskeleton_hardware::usb_protocol::DPS_TO_RADS;
  EXPECT_NEAR(deg, 360.0, 1e-10);
  EXPECT_NEAR(rad, 2.0 * M_PI, 1e-10);
}

TEST(UnitConversionTest, SpeedDpsToOutputRads)
{
  // motor speed_dps / reduction_ratio → output dps → output rad/s
  const double reduction = 36.0;
  double motor_dps = 360.0;  // 1 rev/s at motor
  double output_rads = motor_dps / reduction * M_PI / 180.0;
  EXPECT_NEAR(output_rads, 10.0 * M_PI / 180.0, 1e-10);
}

TEST(UnitConversionTest, ZeroConversion)
{
  EXPECT_DOUBLE_EQ(0.0 * 360.0 / 65536.0, 0.0);
  EXPECT_DOUBLE_EQ(0.0 * 0.01 * M_PI / 180.0, 0.0);
}

TEST(UnitConversionTest, AnkleCircularZeroCorrection)
{
  using exoskeleton_hardware::ExoHardwareInterface;
  EXPECT_EQ(ExoHardwareInterface::ankle_count_from_zero(100, 100), 0);
  EXPECT_EQ(ExoHardwareInterface::ankle_count_from_zero(110, 100), 10);
  EXPECT_EQ(ExoHardwareInterface::ankle_count_from_zero(90, 100), -10);
  EXPECT_EQ(ExoHardwareInterface::ankle_count_from_zero(5, 4090), 11);
  EXPECT_EQ(ExoHardwareInterface::ankle_count_from_zero(4090, 5), -11);
}

// ===================== 帧布局测试 =====================

TEST(FrameLayoutTest, SetTargetDataSize)
{
  // motor_id(1) + mode(1) + value(4) = 6 bytes
  const size_t expected = 1 + 1 + 4;
  EXPECT_EQ(expected, 6u);
}

TEST(FrameLayoutTest, CommandFrameOverhead)
{
  // STX(1) + CMD(1) + LEN(1) + CHK(1) = 4 bytes overhead
  const size_t overhead = 4;
  // 力矩控制帧总长: 4 + 6 = 10 bytes
  EXPECT_EQ(overhead + 6, 10u);
}

TEST(FrameLayoutTest, FeedbackPerMotorSize)
{
  // encoder(u16) + speed(i16) + iq(i16) + multi_turn(i32) = 10 bytes
  EXPECT_EQ(sizeof(uint16_t) + sizeof(int16_t) + sizeof(int16_t) + sizeof(int32_t), 10u);
}

TEST(FrameLayoutTest, ParsesCurrentLen68AnkleExtension)
{
  std::array<uint8_t, 68> data{};
  data[0] = 2;
  data[1] = 2;
  // 42B motor prefix, then 18B diagnostics. Put a recognizable STM32 tick in it.
  data[44] = 0x78;
  data[45] = 0x56;
  data[46] = 0x34;
  data[47] = 0x12;
  // Left ankle at offset 60, right ankle at offset 64.
  data[60] = 1;
  data[61] = 0x01;
  data[62] = 0x34;
  data[63] = 0x02;
  data[64] = 2;
  data[65] = 0x03;
  data[66] = 0xCD;
  data[67] = 0x0A;

  std::vector<exoskeleton_hardware::MotorFeedback> motors;
  exoskeleton_hardware::FeedbackMetadata metadata;
  ASSERT_TRUE(
    exoskeleton_hardware::ExoHardwareInterface::parse_feedback_data(
      data.data(), data.size(), motors, metadata));
  ASSERT_EQ(motors.size(), 4u);
  ASSERT_TRUE(metadata.extension_valid);
  ASSERT_TRUE(metadata.ankle_extension_valid);
  EXPECT_EQ(metadata.stm32_tick_ms, 0x12345678u);
  EXPECT_EQ(metadata.ankles[0].node_id, 1);
  EXPECT_EQ(metadata.ankles[0].flags, 0x01);
  EXPECT_EQ(metadata.ankles[0].raw_count, 0x0234);
  EXPECT_EQ(metadata.ankles[1].node_id, 2);
  EXPECT_EQ(metadata.ankles[1].flags, 0x03);
  EXPECT_EQ(metadata.ankles[1].raw_count, 0x0ACD);
}

TEST(FrameLayoutTest, KeepsLegacyLen42Compatible)
{
  std::array<uint8_t, 42> data{};
  data[0] = 2;
  data[1] = 2;
  std::vector<exoskeleton_hardware::MotorFeedback> motors;
  exoskeleton_hardware::FeedbackMetadata metadata;
  ASSERT_TRUE(
    exoskeleton_hardware::ExoHardwareInterface::parse_feedback_data(
      data.data(), data.size(), motors, metadata));
  EXPECT_EQ(motors.size(), 4u);
  EXPECT_FALSE(metadata.extension_valid);
  EXPECT_FALSE(metadata.ankle_extension_valid);
}

// ===================== NUM_JOINTS 测试 =====================

TEST(ConstantsTest, NumJoints)
{
  EXPECT_EQ(exoskeleton_hardware::NUM_JOINTS, 4u);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

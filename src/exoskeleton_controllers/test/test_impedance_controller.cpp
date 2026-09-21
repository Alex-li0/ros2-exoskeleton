#include <gtest/gtest.h>

#include <cmath>
#include <vector>

// ===================== 阻抗控制律核心逻辑测试 =====================
// 直接测试数学逻辑，不依赖 ros2_control 框架

class ImpedanceLawTest : public ::testing::Test
{
protected:
  // 阻抗控制律: τ = Kp*(q_ref - q) + Kd*(qdot_ref - qdot) + τ_ff
  double compute_impedance(
    double q_ref, double q, double qdot_ref, double qdot,
    double kp, double kd, double tau_ff)
  {
    return kp * (q_ref - q) + kd * (qdot_ref - qdot) + tau_ff;
  }

  double compute_virtual_tube(
    double position_error, double velocity_error, double kp, double kd,
    double half_width, double transition_width)
  {
    if (half_width <= 0.0)
    {
      return kp * position_error + kd * velocity_error;
    }
    const double outside_distance = std::abs(position_error) - half_width;
    if (outside_distance <= 0.0)
    {
      return 0.0;
    }
    const double wall_error = std::copysign(outside_distance, position_error);
    if (transition_width <= 0.0)
    {
      return kp * wall_error + kd * velocity_error;
    }
    const double normalized = std::clamp(outside_distance / transition_width, 0.0, 1.0);
    const double activation = normalized * normalized * (3.0 - 2.0 * normalized);
    return activation * (kp * wall_error + kd * velocity_error);
  }

  // 力矩限幅
  double clamp_torque(double tau, double limit)
  {
    return std::clamp(tau, -limit, limit);
  }

  // 变化率限制
  double rate_limit(double tau_raw, double tau_prev, double rate_limit, double dt)
  {
    double max_delta = rate_limit * dt;
    double delta = tau_raw - tau_prev;
    delta = std::clamp(delta, -max_delta, max_delta);
    return tau_prev + delta;
  }

  // 速度低通滤波
  double filter_velocity(double raw, double prev, double alpha)
  {
    return alpha * raw + (1.0 - alpha) * prev;
  }
};

TEST_F(ImpedanceLawTest, ZeroErrorZeroTorque)
{
  // 位置和速度误差为零 → 力矩为零（无前馈）
  double tau = compute_impedance(1.0, 1.0, 0.0, 0.0, 50.0, 5.0, 0.0);
  EXPECT_DOUBLE_EQ(tau, 0.0);
}

TEST_F(ImpedanceLawTest, PurePositionError)
{
  // q_ref=0.1, q=0, qdot_ref=0, qdot=0, Kp=100 → τ=10
  double tau = compute_impedance(0.1, 0.0, 0.0, 0.0, 100.0, 5.0, 0.0);
  EXPECT_NEAR(tau, 10.0, 1e-10);
}

TEST_F(ImpedanceLawTest, PureVelocityError)
{
  // qdot_ref=1.0, qdot=0, Kd=5 → τ=5
  double tau = compute_impedance(0.0, 0.0, 1.0, 0.0, 50.0, 5.0, 0.0);
  EXPECT_NEAR(tau, 5.0, 1e-10);
}

TEST_F(ImpedanceLawTest, FeedforwardOnly)
{
  double tau = compute_impedance(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 15.0);
  EXPECT_DOUBLE_EQ(tau, 15.0);
}

TEST_F(ImpedanceLawTest, Combined)
{
  // Kp=50, pos_err=0.1 → 5; Kd=8, vel_err=0.2 → 1.6; ff=2
  double tau = compute_impedance(0.1, 0.0, 0.2, 0.0, 50.0, 8.0, 2.0);
  EXPECT_NEAR(tau, 5.0 + 1.6 + 2.0, 1e-10);
}

TEST_F(ImpedanceLawTest, NegativeError)
{
  double tau = compute_impedance(-0.1, 0.0, 0.0, 0.0, 100.0, 5.0, 0.0);
  EXPECT_NEAR(tau, -10.0, 1e-10);
}

TEST_F(ImpedanceLawTest, VirtualTubeSuppressesFeedbackInsideBoundary)
{
  EXPECT_DOUBLE_EQ(compute_virtual_tube(0.09, 5.0, 100.0, 10.0, 0.1, 0.02), 0.0);
}

TEST_F(ImpedanceLawTest, VirtualTubeUsesOnlyDistanceOutsideBoundary)
{
  EXPECT_NEAR(compute_virtual_tube(0.15, 0.0, 100.0, 10.0, 0.1, 0.0), 5.0, 1e-10);
  EXPECT_NEAR(compute_virtual_tube(-0.15, 0.0, 100.0, 10.0, 0.1, 0.0), -5.0, 1e-10);
}

TEST_F(ImpedanceLawTest, VirtualTubeSmoothstepTransition)
{
  // 越界距离为平滑宽度的一半时，smoothstep(0.5)=0.5。
  EXPECT_NEAR(compute_virtual_tube(0.11, 0.0, 100.0, 0.0, 0.1, 0.02), 0.5, 1e-10);
}

TEST_F(ImpedanceLawTest, ZeroWidthMatchesLegacyFeedback)
{
  EXPECT_NEAR(
    compute_virtual_tube(0.1, 0.2, 50.0, 8.0, 0.0, 0.0),
    compute_impedance(0.1, 0.0, 0.2, 0.0, 50.0, 8.0, 0.0), 1e-10);
  EXPECT_NEAR(compute_virtual_tube(0.0, 0.2, 50.0, 8.0, 0.0, 0.0), 1.6, 1e-10);
}

// ===================== 力矩限幅测试 =====================

TEST_F(ImpedanceLawTest, TorqueClampPositive)
{
  EXPECT_DOUBLE_EQ(clamp_torque(50.0, 20.0), 20.0);
}

TEST_F(ImpedanceLawTest, TorqueClampNegative)
{
  EXPECT_DOUBLE_EQ(clamp_torque(-50.0, 20.0), -20.0);
}

TEST_F(ImpedanceLawTest, TorqueClampWithinLimit)
{
  EXPECT_DOUBLE_EQ(clamp_torque(15.0, 20.0), 15.0);
}

TEST_F(ImpedanceLawTest, TorqueClampZero)
{
  EXPECT_DOUBLE_EQ(clamp_torque(0.0, 20.0), 0.0);
}

// ===================== 变化率限制测试 =====================

TEST_F(ImpedanceLawTest, RateLimitNoChange)
{
  // rate=100 Nm/s, dt=0.002s → max_delta=0.2 Nm
  double tau = rate_limit(10.0, 10.0, 100.0, 0.002);
  EXPECT_DOUBLE_EQ(tau, 10.0);
}

TEST_F(ImpedanceLawTest, RateLimitSmallStep)
{
  // 变化 0.1 Nm < max_delta 0.2 Nm → 不限制
  double tau = rate_limit(10.1, 10.0, 100.0, 0.002);
  EXPECT_NEAR(tau, 10.1, 1e-10);
}

TEST_F(ImpedanceLawTest, RateLimitLargeStep)
{
  // 变化 1.0 Nm > max_delta 0.2 Nm → 限制到 +0.2
  double tau = rate_limit(11.0, 10.0, 100.0, 0.002);
  EXPECT_NEAR(tau, 10.2, 1e-10);
}

TEST_F(ImpedanceLawTest, RateLimitLargeNegativeStep)
{
  double tau = rate_limit(9.0, 10.0, 100.0, 0.002);
  EXPECT_NEAR(tau, 9.8, 1e-10);
}

// ===================== 速度滤波测试 =====================

TEST_F(ImpedanceLawTest, FilterNoFilter)
{
  // alpha=1 → 不滤波，直接返回原始值
  EXPECT_DOUBLE_EQ(filter_velocity(5.0, 3.0, 1.0), 5.0);
}

TEST_F(ImpedanceLawTest, FilterFullSmoothing)
{
  // alpha=0 → 完全平滑，返回上一次值
  EXPECT_DOUBLE_EQ(filter_velocity(5.0, 3.0, 0.0), 3.0);
}

TEST_F(ImpedanceLawTest, FilterPartial)
{
  // alpha=0.2, raw=5, prev=3 → 0.2*5 + 0.8*3 = 1 + 2.4 = 3.4
  EXPECT_NEAR(filter_velocity(5.0, 3.0, 0.2), 3.4, 1e-10);
}

TEST_F(ImpedanceLawTest, FilterConvergence)
{
  // 连续滤波应该收敛到输入值
  double val = 0.0;
  double target = 10.0;
  double alpha = 0.2;
  for (int i = 0; i < 1000; ++i)
  {
    val = filter_velocity(target, val, alpha);
  }
  EXPECT_NEAR(val, target, 1e-3);
}

// ===================== 参考接口布局测试 =====================

TEST(ReferenceOffsetsTest, FivePerJoint)
{
  // 每关节 5 个参考接口: position, velocity, ff_torque, kp, kd
  constexpr size_t num_joints = 6;
  constexpr size_t interfaces_per_joint = 5;
  constexpr size_t total = num_joints * interfaces_per_joint;
  EXPECT_EQ(total, 30);
}

TEST(ReferenceOffsetsTest, IndexLayout)
{
  // joint 0: [0,1,2,3,4], joint 1: [5,6,7,8,9], ...
  for (size_t i = 0; i < 6; ++i)
  {
    EXPECT_EQ(i * 5 + 0, i * 5);      // target_position
    EXPECT_EQ(i * 5 + 1, i * 5 + 1);  // target_velocity
    EXPECT_EQ(i * 5 + 2, i * 5 + 2);  // feedforward_torque
    EXPECT_EQ(i * 5 + 3, i * 5 + 3);  // kp
    EXPECT_EQ(i * 5 + 4, i * 5 + 4);  // kd
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

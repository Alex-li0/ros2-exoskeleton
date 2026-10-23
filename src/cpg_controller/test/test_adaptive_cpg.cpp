#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

#include "cpg_controller/adaptive_cpg.hpp"

namespace {
using cpg_controller::AdaptiveCPG;
using cpg_controller::AdaptiveConfig;
using cpg_controller::Coefficients;
using cpg_controller::MeasurementFrame;

std::array<Coefficients, 6> prior()
{
  std::array<Coefficients, 6> result;
  const std::array<double, 6> amplitude{.22, .35, .06, -.23, -.27, -.05};
  for (size_t j = 0; j < result.size(); ++j) {
    result[j].offset.value = -.03 * static_cast<double>(j);
    result[j].x[1].value = amplitude[j];
    result[j].y[1].value = (.3 + .07 * j) * amplitude[j];
    result[j].x[2].value = (.15 - .01 * j) * amplitude[j];
    result[j].y[3].value = (-.08 + .02 * j) * amplitude[j];
  }
  return result;
}

MeasurementFrame observation(double stamp, double phase,
  const std::array<Coefficients, 6> &weights)
{
  MeasurementFrame frame;
  frame.stamp = stamp;
  frame.valid.fill(true);
  for (size_t j = 0; j < weights.size(); ++j) {
    frame.position[j] = weights[j].offset.value;
    for (size_t n = 1; n <= 3; ++n) {
      frame.position[j] += weights[j].x[n].value * std::cos(n * phase) +
        weights[j].y[n].value * std::sin(n * phase);
    }
  }
  return frame;
}

void matched_observations(AdaptiveCPG &core, double begin, double duration)
{
  const auto weights = prior();
  for (int i = 1; i <= static_cast<int>(std::round(duration / .002)); ++i) {
    const double time = begin + .002 * i;
    core.step(time);
    if (i % 5 == 0) {
      const auto frame = observation(time, core.output().phase, weights);
      EXPECT_TRUE(core.observe(frame, time));
    }
  }
}
}  // namespace

TEST(AdaptiveCPG, MatchedSixJointInputHasNoParameterDrift)
{
  AdaptiveConfig config;
  config.events_enabled = false;
  AdaptiveCPG core(prior(), config, 10);
  matched_observations(core, 10, 5);
  EXPECT_NEAR(core.output().omega_base, config.omega_initial, 1e-9);
  EXPECT_NEAR(core.output().residual_rms, 0, 1e-8);
  const auto initial = prior();
  for (size_t j = 0; j < initial.size(); ++j) {
    EXPECT_NEAR(core.weights()[j].offset.value, initial[j].offset.value, 1e-9);
    for (size_t n = 1; n <= 3; ++n) {
      EXPECT_NEAR(core.weights()[j].x[n].value, initial[j].x[n].value, 1e-9);
      EXPECT_NEAR(core.weights()[j].y[n].value, initial[j].y[n].value, 1e-9);
    }
  }
}

TEST(AdaptiveCPG, PhaseFeedbackAcceleratesForLeadingMultijointMeasurement)
{
  AdaptiveConfig config;
  config.shape_enabled = false;
  config.events_enabled = false;
  AdaptiveCPG core(prior(), config, 10);
  matched_observations(core, 10, 3);
  for (int i = 1; i <= 100; ++i) {
    const double time = 13 + i * .002;
    core.step(time);
    if (i % 5 == 0) {
      ASSERT_TRUE(core.observe(observation(time, core.output().phase + .08, prior()), time));
    }
  }
  EXPECT_GT(core.output().phase_error, 0);
  EXPECT_GT(core.output().phase_rate, config.omega_initial);
  EXPECT_LE(core.output().phase_rate, config.omega_max);
}

TEST(AdaptiveCPG, DuplicateFutureExpiredAndNonfiniteMeasurementsCannotRefreshState)
{
  AdaptiveConfig config;
  AdaptiveCPG core(prior(), config, 10);
  core.step(10.02);
  auto frame = observation(10.02, core.output().phase, prior());
  ASSERT_TRUE(core.observe(frame, 10.02));
  EXPECT_FALSE(core.observe(frame, 10.02));
  frame.stamp = 10.03;
  EXPECT_FALSE(core.observe(frame, 10.02));
  frame.stamp = 9;
  EXPECT_FALSE(core.observe(frame, 10.02));
  frame.stamp = 10.021;
  frame.position.fill(std::numeric_limits<double>::quiet_NaN());
  core.step(10.022);
  EXPECT_FALSE(core.observe(frame, 10.022));
  const auto before = core.weights();
  core.step(10.15);
  EXPECT_EQ(static_cast<int>(core.output().state), 3);  // LOST
  EXPECT_FALSE(core.output().shape_active);
  EXPECT_LE(core.output().phase_rate, config.omega_max);
  EXPECT_GT(core.output().phase_rate, 0);
  EXPECT_DOUBLE_EQ(core.weights()[0].offset.value, before[0].offset.value);
}

TEST(AdaptiveCPG, OutputVelocityIsDerivativeOfNormalizedFourierReadout)
{
  AdaptiveConfig config;
  config.shape_enabled = false;
  AdaptiveCPG core(prior(), config, 10);
  core.step(10.1);
  auto a = core.output();
  core.step(10.1005);
  const auto b = core.output();
  core.step(10.101);
  const auto c = core.output();
  for (size_t j = 0; j < 6; ++j)
    EXPECT_NEAR((c.position[j] - a.position[j]) / .001, b.velocity[j], 2e-3);
}

TEST(AdaptiveCPG, EventUsesSameFootPeriodAndRejectsReplay)
{
  AdaptiveConfig config;
  config.shape_enabled = false;
  config.events_enabled = true;
  AdaptiveCPG core(prior(), config, 10);
  for (int i = 1; i <= 1000; ++i) {
    const double time = 10 + i * .002;
    core.step(time);
    if (i % 5 == 0) {
      ASSERT_TRUE(core.observe(observation(time, core.output().phase, prior()), time));
    }
    if (i == 100) {
      ASSERT_TRUE(core.event(time, 0, 1, 1, core.output().phase, time));
      EXPECT_FALSE(core.event(time, 0, 1, 1, core.output().phase, time));
    }
    if (i == 150) {
      ASSERT_TRUE(core.event(time, 1, 1, 1, core.output().phase, time));
    }
    if (i == 600) {
      ASSERT_TRUE(core.event(time, 0, 2, 1, core.output().phase, time));
    }
  }
  // LEFT events are one second apart. RIGHT's intervening event must not be
  // interpreted as a full stride and drive omega toward the upper bound.
  EXPECT_GT(core.output().omega_base, config.omega_initial);
  EXPECT_LT(core.output().omega_base, 7);
  EXPECT_LE(core.output().phase_rate, config.omega_max);
}

TEST(AdaptiveCPG, RejectedGrossFrameCannotInitializeClockOrChangeState)
{
  AdaptiveConfig config;
  const auto weights = prior();
  AdaptiveCPG core(weights, config, 10);
  core.step(10.02);
  const auto before = core.output();
  MeasurementFrame corrupt;
  corrupt.stamp = 10.02;
  corrupt.valid.fill(true);
  for (size_t j = 0; j < weights.size(); ++j) {
    double energy = 0;
    for (size_t n = 1; n <= 3; ++n)
      energy += std::pow(weights[j].x[n].value, 2) + std::pow(weights[j].y[n].value, 2);
    const double scale = std::max(.06, std::sqrt(energy / 2));
    corrupt.position[j] = weights[j].offset.value + 6 * scale;
  }
  EXPECT_FALSE(core.observe(corrupt, 10.02));
  EXPECT_EQ(core.output().state, before.state);
  EXPECT_DOUBLE_EQ(core.output().phase, before.phase);
  EXPECT_DOUBLE_EQ(core.output().omega_base, before.omega_base);
}

TEST(AdaptiveCPG, DelayedMeasurementsAndEventCompareAgainstHistoricalPhase)
{
  AdaptiveConfig config;
  config.shape_enabled = false;
  config.phase_gain = 0;
  config.frequency_gain = 0;
  config.events_enabled = true;
  AdaptiveCPG core(prior(), config, 10);
  ASSERT_TRUE(core.observe(observation(10, 0, prior()), 10));
  for (int i = 1; i <= 250; ++i) {
    const double time = 10 + .002 * i;
    core.step(time);
    if (i % 5 == 0 && i >= 25) {
      const double source = time - .04;
      ASSERT_TRUE(core.observe(observation(source,
        config.omega_initial * (source - 10), prior()), time));
      EXPECT_NEAR(core.output().residual_rms, 0, 1e-10);
    }
    if (i == 150) {
      const double source = time - .05;
      ASSERT_TRUE(core.event(source, 0, 1, 1,
        config.omega_initial * (source - 10), time));
    }
  }
  EXPECT_NEAR(core.output().phase_rate, config.omega_initial, 1e-9);
  EXPECT_NEAR(core.output().phase, config.omega_initial * .5, 1e-9);
}

TEST(AdaptiveCPG, AcquisitionRetainsHistoryForValidNinetyMillisecondDelay)
{
  AdaptiveConfig config;
  config.shape_enabled = false;
  config.events_enabled = false;
  config.measurement_timeout = .12;
  config.phase_gain = config.frequency_gain = 0;  // isolate history acquisition
  AdaptiveCPG core(prior(), config, 10);
  size_t accepted = 0;
  for (int i = 1; i <= 2500; ++i) {
    const double time = 10 + .002 * i;
    core.step(time);
    if (i % 5 == 0 && i >= 50) {
      const double source = time - .09;
      if (core.observe(observation(source,
          config.omega_initial * (source - 10), prior()), time)) ++accepted;
    }
  }
  EXPECT_GT(accepted, 400u);
  EXPECT_EQ(core.output().state, cpg_controller::LearningState::LOCKED);
  EXPECT_NEAR(core.output().residual_rms, 0, 1e-9);
}

TEST(AdaptiveCPG, LearningVelocityIncludesActualCoefficientRate)
{
  AdaptiveConfig config;
  config.events_enabled = false;
  config.phase_gain = config.frequency_gain = 0;
  AdaptiveCPG core(prior(), config, 10);
  ASSERT_TRUE(core.observe(observation(10, 0, prior()), 10));
  auto truth = prior();
  auto a = core.output(), b = a;
  double maximum_error = 0, maximum_rate = 0;
  for (int i = 1; i <= 16000; ++i) {
    const double time = 10 + .0005 * i;
    core.step(time);
    if (i % 20 == 0) {
      if (i > 6000) {
        truth = prior();
        for (auto &joint : truth) {
          joint.offset.value += .01;
          for (size_t n = 1; n <= 3; ++n) {
            joint.x[n].value *= 1.12;
            joint.y[n].value *= 1.12;
          }
        }
      }
      ASSERT_TRUE(core.observe(observation(time,
        config.omega_initial * (time - 10), truth), time));
    }
    const auto c = core.output();
    if (i > 7000) {
      for (size_t j = 0; j < 6; ++j) {
        maximum_error = std::max(maximum_error,
          std::abs((c.position[j] - a.position[j]) / .001 - b.velocity[j]));
        maximum_rate = std::max(maximum_rate, std::abs(core.weights()[j].offset.rate));
      }
    }
    a = b; b = c;
  }
  EXPECT_GT(maximum_rate, 1e-3);
  EXPECT_GT(core.weights()[0].offset.value - prior()[0].offset.value, 1e-3);
  EXPECT_LT(maximum_error, 3e-4);
}

TEST(AdaptiveCPG, SixJointDeformationLearnsBeyondPhaseAndFrequencySynchronization)
{
  AdaptiveConfig full_config;
  full_config.events_enabled = true;
  AdaptiveConfig sync_config = full_config;
  sync_config.shape_enabled = false;
  const auto initial = prior();
  auto deformed = initial;
  for (size_t j = 0; j < 6; ++j) {
    const double scale = .84 + .06 * j;
    const double shift = .03 * (static_cast<double>(j) - 2.5);
    deformed[j].offset.value += j % 2 ? .018 : -.012;
    for (size_t n = 1; n <= 3; ++n) {
      const double angle = n * shift;
      deformed[j].x[n].value = scale * (initial[j].x[n].value * std::cos(angle) +
        initial[j].y[n].value * std::sin(angle));
      deformed[j].y[n].value = scale * (-initial[j].x[n].value * std::sin(angle) +
        initial[j].y[n].value * std::cos(angle));
    }
  }
  AdaptiveCPG full(initial, full_config, 10), sync(initial, sync_config, 10);
  const auto first = observation(10, 0, deformed);
  ASSERT_TRUE(full.observe(first, 10));
  ASSERT_TRUE(sync.observe(first, 10));
  ASSERT_TRUE(full.event(10, 0, 1, 1, 0, 10));
  ASSERT_TRUE(sync.event(10, 0, 1, 1, 0, 10));
  std::array<double, 6> full_squared{}, sync_squared{};
  size_t count = 0;
  int previous_halfcycle = 0;
  for (int i = 1; i <= 20000; ++i) {
    const double time = 10 + i * .002;
    const double truth_phase = full_config.omega_initial * (time - 10);
    full.step(time); sync.step(time);
    const auto truth = observation(time, truth_phase, deformed);
    if (i % 5 == 0) {
      ASSERT_TRUE(full.observe(truth, time));
      ASSERT_TRUE(sync.observe(truth, time));
    }
    const int halfcycle = static_cast<int>(std::floor(truth_phase / std::acos(-1.0)));
    if (halfcycle > previous_halfcycle) {
      const double source = 10 + halfcycle * std::acos(-1.0) / full_config.omega_initial;
      const size_t foot = halfcycle % 2;
      const uint64_t id = halfcycle / 2 + 1;
      const double target = foot ? std::acos(-1.0) : 0;
      ASSERT_TRUE(full.event(source, foot, id, 1, target, time));
      ASSERT_TRUE(sync.event(source, foot, id, 1, target, time));
      previous_halfcycle = halfcycle;
    }
    if (time >= 40) {
      ++count;
      for (size_t j = 0; j < 6; ++j) {
        full_squared[j] += std::pow(full.output().position[j] - truth.position[j], 2);
        sync_squared[j] += std::pow(sync.output().position[j] - truth.position[j], 2);
      }
    }
  }
  ASSERT_GT(count, 1000u);
  double total_full = 0, total_sync = 0;
  for (size_t j = 0; j < 6; ++j) {
    total_full += full_squared[j]; total_sync += sync_squared[j];
    EXPECT_LT(full_squared[j], .7 * sync_squared[j]) << "joint " << j;
    double weight_change = std::abs(full.weights()[j].offset.value - initial[j].offset.value);
    for (size_t n = 1; n <= 3; ++n) {
      weight_change += std::abs(full.weights()[j].x[n].value - initial[j].x[n].value);
      weight_change += std::abs(full.weights()[j].y[n].value - initial[j].y[n].value);
    }
    EXPECT_GT(weight_change, 1e-3) << "joint " << j;
  }
  EXPECT_LT(std::sqrt(total_full / total_sync), .5);
}

TEST(AdaptiveCPG, ReacquisitionRejectsEventFromBeforeNewObservationHistory)
{
  AdaptiveConfig config;
  config.shape_enabled = false;
  AdaptiveCPG core(prior(), config, 10);
  matched_observations(core, 10, 2);
  const double old_phase = core.output().phase;
  for (int i = 1; i <= 100; ++i) core.step(12 + .002 * i);
  ASSERT_EQ(core.output().state, cpg_controller::LearningState::LOST);
  ASSERT_TRUE(core.observe(observation(12.2, core.output().phase + .2, prior()), 12.2));
  EXPECT_FALSE(core.event(12, 0, 1, 1, old_phase, 12.2));
}

TEST(AdaptiveCPG, SimultaneousPolicyLearnsShapeDuringCaptureWithSameGains)
{
  AdaptiveConfig staged_config;
  staged_config.events_enabled = false;
  staged_config.lock_duration = 5;
  auto joint_config = staged_config;
  joint_config.shape_requires_lock = false;
  AdaptiveCPG staged(prior(), staged_config, 10);
  AdaptiveCPG joint(prior(), joint_config, 10);
  auto target = prior();
  for (size_t j = 0; j < 6; ++j) {
    target[j].offset.value += .015;
    target[j].x[1].value *= 1.1;
  }
  bool joint_shape_active = false;
  for (int k = 1; k <= 500; ++k) {
    const double time = 10 + .002 * k;
    staged.step(time); joint.step(time);
    if (k % 5 == 0) {
      const auto sample = observation(time, staged_config.omega_initial * (time - 10), target);
      ASSERT_TRUE(staged.observe(sample, time));
      ASSERT_TRUE(joint.observe(sample, time));
    }
    EXPECT_FALSE(staged.output().shape_active);
    if (joint.output().shape_active) joint_shape_active = true;
  }
  EXPECT_TRUE(joint_shape_active);
  EXPECT_NE(staged.output().state, cpg_controller::LearningState::LOCKED);
  EXPECT_NE(joint.output().state, cpg_controller::LearningState::LOCKED);
  EXPECT_DOUBLE_EQ(staged.weights()[0].offset.value, prior()[0].offset.value);
  EXPECT_GT(std::abs(joint.weights()[0].offset.value - prior()[0].offset.value), 1e-4);
}

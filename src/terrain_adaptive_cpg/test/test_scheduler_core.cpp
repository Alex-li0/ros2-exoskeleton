#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "terrain_adaptive_cpg/scheduler_core.hpp"

using terrain_adaptive_cpg::ContinuousLimits;
using terrain_adaptive_cpg::ContinuousTarget;

TEST(SchedulerCore, RejectsInvalidCovariance)
{
  std::vector<double> covariance(36, 0.0);
  EXPECT_TRUE(terrain_adaptive_cpg::covarianceStructurallyValid(covariance));
  covariance[7] = -1.0;
  EXPECT_FALSE(terrain_adaptive_cpg::covarianceStructurallyValid(covariance));
  covariance[7] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(terrain_adaptive_cpg::covarianceStructurallyValid(covariance));
}

TEST(SchedulerCore, ClampsAndRateLimitsSyntheticTarget)
{
  ContinuousLimits limits;
  limits.omega_scale_min = 0.8;
  limits.omega_scale_max = 1.2;
  limits.amplitude_scale_min = 0.7;
  limits.amplitude_scale_max = 1.3;
  limits.offset_delta_min = -0.1;
  limits.offset_delta_max = 0.1;
  limits.omega_scale_rate = 0.5;
  limits.amplitude_scale_rate = 1.0;
  limits.offset_delta_rate = 0.2;
  ContinuousTarget current{1.0, {1.0}, {0.0}};
  ContinuousTarget requested{2.0, {2.0}, {1.0}};
  const auto output = terrain_adaptive_cpg::applyLimits(current, requested, limits, 0.1);
  EXPECT_DOUBLE_EQ(output.omega_scale, 1.05);
  EXPECT_DOUBLE_EQ(output.amplitude_scale[0], 1.1);
  EXPECT_DOUBLE_EQ(output.offset_delta[0], 0.02);
}

TEST(SchedulerCore, FiltersAfterRateLimit)
{
  ContinuousLimits limits;
  limits.omega_scale_min = 0.0;
  limits.omega_scale_max = 2.0;
  limits.omega_scale_rate = 10.0;
  limits.filter_time_constant = 0.1;
  ContinuousTarget current{1.0, {}, {}};
  ContinuousTarget requested{2.0, {}, {}};
  const auto output = terrain_adaptive_cpg::applyLimits(current, requested, limits, 0.1);
  EXPECT_DOUBLE_EQ(output.omega_scale, 1.5);
}

TEST(SchedulerCore, SwitchWindowIsDoubleStanceOnly)
{
  EXPECT_FALSE(terrain_adaptive_cpg::contactAllowsSwitch(0));
  EXPECT_FALSE(terrain_adaptive_cpg::contactAllowsSwitch(1));
  EXPECT_FALSE(terrain_adaptive_cpg::contactAllowsSwitch(2));
  EXPECT_TRUE(terrain_adaptive_cpg::contactAllowsSwitch(3));
}

TEST(SchedulerCore, DataDrivenModelStaysUnavailable)
{
  EXPECT_FALSE(terrain_adaptive_cpg::lookupTableTarget().has_value());
  EXPECT_FALSE(terrain_adaptive_cpg::interpolatedTarget().has_value());
  EXPECT_FALSE(terrain_adaptive_cpg::regressionTarget().has_value());
  terrain_adaptive_cpg::FirstStepTransitionRequest request;
  EXPECT_FALSE(terrain_adaptive_cpg::firstStepTransitionTarget(request).has_value());
}

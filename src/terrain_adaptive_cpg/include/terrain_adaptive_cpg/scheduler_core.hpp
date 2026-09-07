#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace terrain_adaptive_cpg
{

struct ContinuousLimits
{
  double omega_scale_min{1.0};
  double omega_scale_max{1.0};
  double amplitude_scale_min{1.0};
  double amplitude_scale_max{1.0};
  double offset_delta_min{0.0};
  double offset_delta_max{0.0};
  double omega_scale_rate{0.0};
  double amplitude_scale_rate{0.0};
  double offset_delta_rate{0.0};
  double filter_time_constant{0.0};
};

struct ContinuousTarget
{
  double omega_scale{1.0};
  std::vector<double> amplitude_scale;
  std::vector<double> offset_delta;
};

enum class ContinuousModelType
{
  DISABLED,
  LOOKUP_TABLE,
  LINEAR_INTERPOLATION,
  REGRESSION
};

struct FirstStepTransitionRequest
{
  std::string from_terrain;
  std::string to_terrain;
  double contact_phase{0.0};
};

inline double boundedStep(double current, double requested, double lower, double upper,
                          double rate, double dt, double filter_time_constant)
{
  const double bounded = std::clamp(requested, lower, upper);
  const double max_delta = std::max(0.0, rate) * std::max(0.0, dt);
  const double rate_limited = current + std::clamp(bounded - current, -max_delta, max_delta);
  if (filter_time_constant <= 0.0 || dt <= 0.0) {
    return rate_limited;
  }
  const double alpha = std::clamp(dt / (filter_time_constant + dt), 0.0, 1.0);
  return current + alpha * (rate_limited - current);
}

inline ContinuousTarget applyLimits(
  const ContinuousTarget & current, const ContinuousTarget & requested,
  const ContinuousLimits & limits, double dt)
{
  ContinuousTarget out = current;
  out.omega_scale = boundedStep(
    current.omega_scale, requested.omega_scale, limits.omega_scale_min,
    limits.omega_scale_max, limits.omega_scale_rate, dt, limits.filter_time_constant);
  out.amplitude_scale.resize(requested.amplitude_scale.size(), 1.0);
  out.offset_delta.resize(requested.offset_delta.size(), 0.0);
  for (size_t i = 0; i < requested.amplitude_scale.size(); ++i) {
    const double previous = i < current.amplitude_scale.size() ? current.amplitude_scale[i] : 1.0;
    out.amplitude_scale[i] = boundedStep(
      previous, requested.amplitude_scale[i], limits.amplitude_scale_min,
      limits.amplitude_scale_max, limits.amplitude_scale_rate, dt, limits.filter_time_constant);
  }
  for (size_t i = 0; i < requested.offset_delta.size(); ++i) {
    const double previous = i < current.offset_delta.size() ? current.offset_delta[i] : 0.0;
    out.offset_delta[i] = boundedStep(
      previous, requested.offset_delta[i], limits.offset_delta_min,
      limits.offset_delta_max, limits.offset_delta_rate, dt, limits.filter_time_constant);
  }
  return out;
}

inline bool covarianceStructurallyValid(const std::vector<double> & covariance)
{
  if (covariance.size() != 36) return false;
  for (double value : covariance) {
    if (!std::isfinite(value)) return false;
  }
  for (size_t i = 0; i < 6; ++i) {
    if (covariance[i * 6 + i] < 0.0) return false;
  }
  return true;
}

inline bool contactAllowsSwitch(uint8_t state)
{
  return state == 3;  // 双脚支撑是当前唯一不依赖数值标定的保守切换窗口。
}

inline std::optional<ContinuousTarget> lookupTableTarget()
{
  // TODO(data_required): 加载离散地形量测与已验证 CPG 参数对。
  return std::nullopt;
}

inline std::optional<ContinuousTarget> interpolatedTarget()
{
  // TODO(data_required): 仅在实测查表点覆盖工作域后实现有界插值。
  return std::nullopt;
}

inline std::optional<ContinuousTarget> regressionTarget()
{
  // TODO(data_required): 仅加载离线验证并带版本信息的回归模型；禁止在线学习。
  return std::nullopt;
}

inline std::optional<ContinuousTarget> firstStepTransitionTarget(
  const FirstStepTransitionRequest & /*request*/)
{
  // TODO(data_required): 获得平地到地形边界的首步接触与轨迹数据后实现。
  return std::nullopt;
}

}  // namespace terrain_adaptive_cpg

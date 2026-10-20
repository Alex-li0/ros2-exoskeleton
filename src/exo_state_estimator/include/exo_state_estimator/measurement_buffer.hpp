#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

namespace exo_state_estimator {

struct JointSample {
  double timestamp;
  std::vector<double> positions;
};

// Interpolation is only allowed inside a sufficiently short measured interval.
// No extrapolation, and missing joint values are never replaced with zero.
inline bool interpolateJoints(const std::deque<JointSample>& samples, double stamp,
                              double max_gap, std::vector<double>& positions) {
  positions.clear();
  auto next = std::lower_bound(samples.begin(), samples.end(), stamp,
      [](const JointSample& sample, double t) { return sample.timestamp < t; });
  if (next != samples.end() && std::abs(next->timestamp - stamp) < 1e-9) {
    positions = next->positions;
    return true;
  }
  if (next == samples.begin() || next == samples.end()) return false;
  const auto& previous = *std::prev(next);
  const double dt = next->timestamp - previous.timestamp;
  if (dt <= 0 || dt > max_gap || previous.positions.size() != next->positions.size()) return false;
  const double ratio = (stamp - previous.timestamp) / dt;
  positions.resize(next->positions.size());
  for (size_t i = 0; i < positions.size(); ++i) {
    positions[i] = previous.positions[i] + ratio * (next->positions[i] - previous.positions[i]);
  }
  return true;
}

}  // namespace exo_state_estimator

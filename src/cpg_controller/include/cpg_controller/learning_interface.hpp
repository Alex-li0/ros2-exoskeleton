#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>
#include "cpg_controller/reference_math.hpp"

namespace cpg_controller {
struct MeasurementFrame {
  double stamp{0};
  std::array<double,6> position{};
  std::array<bool,6> valid{};
};
// Cache only: no extrapolation, no reuse of reference/display data as teaching input.
class MeasurementInput {
public:
  bool accept(const std::vector<std::string>& names,const std::vector<double>& q,
      double stamp,double now,double timeout,const std::vector<std::string>& expected) {
    if (expected.size()!=6 || names.size()!=q.size() || !std::isfinite(stamp) ||
        !std::isfinite(now) || stamp<=0 || stamp<=frame_.stamp || stamp>now ||
        now-stamp>timeout) return false;
    MeasurementFrame next; next.stamp=stamp;
    for (size_t i=0;i<names.size();++i) {
      if (std::count(names.begin(),names.end(),names[i])!=1) return false;
      auto it=std::find(expected.begin(),expected.end(),names[i]);
      if (it==expected.end()) continue;
      const size_t j=std::distance(expected.begin(),it);
      if (std::isfinite(q[i])) { next.position[j]=q[i]; next.valid[j]=true; }
    }
    if (std::none_of(next.valid.begin(),next.valid.end(),[](bool v){return v;})) return false;
    frame_=next; return true;
  }
  bool fresh(double now,double timeout) const {
    return frame_.stamp>0 && now>=frame_.stamp && now-frame_.stamp<=timeout;
  }
  const MeasurementFrame& frame() const { return frame_; }
private:
  MeasurementFrame frame_{};
};
// Contract for a later validated learner. Rates are rad/s, rad/s^2 and coefficient/s.
// No ROS command accepts these yet: defaults are exactly neutral.
struct LearningRates {
  double phase_rate{0};
  double frequency_rate{0};
  struct JointRates {
    double offset{0};
    std::array<double,4> x{}, y{};  // index 1..3; index 0 unused
  };
  std::array<JointRates,6> coefficient_rates{};
};
// Events will use exo_state_estimator/GaitEvent at the integration boundary.
// Future implementation must compare history at event.stamp, never receipt time.
}  // namespace cpg_controller

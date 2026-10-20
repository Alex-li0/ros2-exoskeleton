#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace exo_state_estimator {

// Shared landmark only during confirmed stable full-foot support.
class KinematicContactFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3> {
 public:
  KinematicContactFactor(gtsam::Key pose, gtsam::Key landmark,
                         const gtsam::Point3& point,
                         const gtsam::SharedNoiseModel& noise)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3>(noise, pose, landmark),
        point_(point) {}
  gtsam::Vector evaluateError(const gtsam::Pose3& pose, const gtsam::Point3& landmark,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const override {
    gtsam::Matrix36 H;
    const auto world = pose.transformFrom(point_, H);
    if (H1) *H1 = H;
    if (H2) *H2 = -gtsam::Matrix33::Identity();
    return world - landmark;
  }
 private:
  gtsam::Point3 point_;
};

}  // namespace exo_state_estimator

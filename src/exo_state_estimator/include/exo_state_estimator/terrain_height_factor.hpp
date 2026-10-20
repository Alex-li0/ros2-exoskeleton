#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace exo_state_estimator {

// 一元一维高程图路标因子
// 残差: r = n_hat^T * (L_k - p_map)  in R^1
// 雅可比: H_L = n_hat^T  (1x3)
class TerrainHeightLandmarkFactor : public gtsam::NoiseModelFactor1<gtsam::Point3> {
 public:
  static constexpr double kSigmaCompliance = 0.01;
  static constexpr double kSigmaComplianceSq = kSigmaCompliance * kSigmaCompliance;

  TerrainHeightLandmarkFactor(gtsam::Key l_key, const gtsam::Point3& p_map,
                              const gtsam::Vector3& n_hat, double sigma_sensor)
      : gtsam::NoiseModelFactor1<gtsam::Point3>(
            gtsam::noiseModel::Isotropic::Sigma(
                1, std::sqrt(sigma_sensor * sigma_sensor + kSigmaComplianceSq)),
            l_key),
        n_hat_(n_hat.normalized()),
        p_map_(p_map) {}

  gtsam::Vector evaluateError(
      const gtsam::Point3& L_k,
      boost::optional<gtsam::Matrix&> H = boost::none) const override {
    double error = n_hat_.dot(L_k - p_map_);
    if (H) {
      *H = gtsam::Matrix::Zero(1, 3);
      H->row(0) = n_hat_.transpose();
    }
    return (gtsam::Vector(1) << error).finished();
  }

 private:
  gtsam::Vector3 n_hat_;
  gtsam::Vector3 p_map_;
};

}  // namespace exo_state_estimator

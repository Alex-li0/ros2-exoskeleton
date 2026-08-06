#pragma once

#include <Eigen/Dense>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace exo_state_estimator {

// 二元本体运动学 Max-Mixture 因子
// 残差: r_raw = X_i.transformFrom(B_p_foot) - L_k
// 双模态: Stick(踩实) vs Slip(打滑)，能量比较自动选择
class KinematicMaxMixtureFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3> {
 public:
  KinematicMaxMixtureFactor(gtsam::Key x_key, gtsam::Key l_key,
                            const gtsam::Vector3& B_p_foot,
                            const Eigen::Matrix3d& Cov_valid,
                            double w_valid = 0.999)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3>(
            gtsam::noiseModel::Unit::Create(3), x_key, l_key),
        B_p_foot_(B_p_foot),
        w_valid_(std::max(1e-6, w_valid)) {
    w_slip_ = std::max(1e-6, 1.0 - w_valid_);
    Eigen::Matrix3d Cov_slip = 1e6 * Eigen::Matrix3d::Identity();
    Eigen::LLT<Eigen::Matrix3d> lltValid(Cov_valid);
    Inv_Cov_valid_half_ =
        lltValid.matrixL().solve(Eigen::Matrix3d::Identity());
    Eigen::LLT<Eigen::Matrix3d> lltSlip(Cov_slip);
    Inv_Cov_slip_half_ = lltSlip.matrixL().solve(Eigen::Matrix3d::Identity());
  }

  gtsam::Vector evaluateError(
      const gtsam::Pose3& X_i, const gtsam::Point3& L_k,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const override {
    gtsam::Vector3 W_p_foot = X_i.transformFrom(B_p_foot_);
    gtsam::Vector3 r_raw = W_p_foot - L_k;

    gtsam::Vector3 r_white_valid = Inv_Cov_valid_half_ * r_raw;
    double E_valid =
        0.5 * r_white_valid.squaredNorm() - std::log(w_valid_);

    gtsam::Vector3 r_white_slip = Inv_Cov_slip_half_ * r_raw;
    double E_slip =
        0.5 * r_white_slip.squaredNorm() - std::log(w_slip_);

    gtsam::Matrix33 R = X_i.rotation().matrix();
    gtsam::Matrix33 p_skew = gtsam::skewSymmetric(B_p_foot_);
    gtsam::Matrix H_X_raw(3, 6);
    H_X_raw.block<3, 3>(0, 0) = -R * p_skew;
    H_X_raw.block<3, 3>(0, 3) = R;
    gtsam::Matrix H_L_raw = -Eigen::Matrix3d::Identity();

    if (E_valid <= E_slip) {
      if (H1) *H1 = gtsam::Matrix(Inv_Cov_valid_half_) * H_X_raw;
      if (H2) *H2 = gtsam::Matrix(Inv_Cov_valid_half_) * H_L_raw;
      return r_white_valid;
    } else {
      if (H1) *H1 = gtsam::Matrix(Inv_Cov_slip_half_) * H_X_raw;
      if (H2) *H2 = gtsam::Matrix(Inv_Cov_slip_half_) * H_L_raw;
      return r_white_slip;
    }
  }

  bool isStick(const gtsam::Pose3& X_i, const gtsam::Point3& L_k) const {
    gtsam::Vector3 W_p_foot = X_i.transformFrom(B_p_foot_);
    gtsam::Vector3 r_raw = W_p_foot - L_k;
    gtsam::Vector3 r_white_valid = Inv_Cov_valid_half_ * r_raw;
    gtsam::Vector3 r_white_slip = Inv_Cov_slip_half_ * r_raw;
    double E_valid =
        0.5 * r_white_valid.squaredNorm() - std::log(w_valid_);
    double E_slip =
        0.5 * r_white_slip.squaredNorm() - std::log(w_slip_);
    return E_valid <= E_slip;
  }

 private:
  gtsam::Vector3 B_p_foot_;
  Eigen::Matrix3d Inv_Cov_valid_half_;
  Eigen::Matrix3d Inv_Cov_slip_half_;
  double w_valid_;
  double w_slip_;
};

}  // namespace exo_state_estimator

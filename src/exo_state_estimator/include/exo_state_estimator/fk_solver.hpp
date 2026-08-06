#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>

namespace exo_state_estimator {

struct FKResult {
  Eigen::Vector3d p_foot;    // 足端在盆腔系的位置
  Eigen::Matrix3d J_pos;     // 位置雅可比 (3 x n_joints)
  bool valid;
};

// 正运动学求解器：从 URDF 加载运动学链，计算足端位置和雅可比
class FKSolver {
 public:
  bool init(const std::string& urdf_string);
  bool initFromFile(const std::string& urdf_path);

  // leg: "left" 或 "right"
  // joint_values: 3个主动关节角度 [hip3, knee, ankle] (rad)
  FKResult compute(const std::string& leg,
                   const std::vector<double>& joint_values) const;

  const std::vector<std::string>& getJointNames(const std::string& leg) const;

 private:
  urdf::Model urdf_model_;
  KDL::Chain left_chain_;
  KDL::Chain right_chain_;
  mutable std::unique_ptr<KDL::ChainFkSolverPos> left_fk_solver_;
  mutable std::unique_ptr<KDL::ChainFkSolverPos> right_fk_solver_;

  std::vector<std::string> left_joint_names_{"left_hip3", "left_knee", "left_ankle"};
  std::vector<std::string> right_joint_names_{"right_hip3", "right_knee", "right_ankle"};

  static constexpr int kNumJoints = 3;

  Eigen::Matrix3d rotationToEigen(const KDL::Rotation& r) const;
  Eigen::Vector3d vectorToEigen(const KDL::Vector& v) const;
};

}  // namespace exo_state_estimator

#include "exo_state_estimator/fk_solver.hpp"

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace exo_state_estimator {

bool FKSolver::init(const std::string& urdf_string) {
  if (!urdf_model_.initString(urdf_string)) {
    std::cerr << "[FKSolver] Failed to parse URDF string" << std::endl;
    return false;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(urdf_model_, tree)) {
    std::cerr << "[FKSolver] Failed to build KDL tree from URDF" << std::endl;
    return false;
  }

  // 左腿链: base_link -> left_foot
  // 包含 left_hip1, left_hip2, left_hip3, left_knee, left_ankle
  // 主动关节: left_hip3, left_knee, left_ankle
  // left_hip1, left_hip2 为固定关节(或可设为0)
  if (!tree.getChain("base_link", "left_foot", left_chain_)) {
    std::cerr << "[FKSolver] Failed to get left leg chain: base_link -> left_foot"
              << std::endl;
    return false;
  }

  // 右腿链: base_link -> right_foot_link
  if (!tree.getChain("base_link", "right_foot_link", right_chain_)) {
    std::cerr << "[FKSolver] Failed to get right leg chain: base_link -> right_foot_link"
              << std::endl;
    return false;
  }

  left_fk_solver_ =
      std::make_unique<KDL::ChainFkSolverPos_recursive>(left_chain_);
  right_fk_solver_ =
      std::make_unique<KDL::ChainFkSolverPos_recursive>(right_chain_);

  std::cout << "[FKSolver] Left chain: " << left_chain_.getNrOfJoints()
            << " joints, " << left_chain_.getNrOfSegments() << " segments"
            << std::endl;
  std::cout << "[FKSolver] Right chain: " << right_chain_.getNrOfJoints()
            << " joints, " << right_chain_.getNrOfSegments() << " segments"
            << std::endl;

  return true;
}

bool FKSolver::initFromFile(const std::string& urdf_path) {
  std::ifstream file(urdf_path);
  if (!file.is_open()) {
    std::cerr << "[FKSolver] Cannot open URDF file: " << urdf_path << std::endl;
    return false;
  }
  std::stringstream ss;
  ss << file.rdbuf();
  return init(ss.str());
}

FKResult FKSolver::compute(const std::string& leg,
                           const std::vector<double>& joint_values,
                           const Eigen::Vector3d& contact_offset) const {
  FKResult result;
  result.valid = false;

  if (joint_values.size() != kNumJoints) {
    std::cerr << "[FKSolver] Expected " << kNumJoints << " joint values, got "
              << joint_values.size() << std::endl;
    return result;
  }

  const KDL::Chain* chain = nullptr;
  KDL::ChainFkSolverPos* solver = nullptr;

  if (leg == "left") {
    chain = &left_chain_;
    solver = left_fk_solver_.get();
  } else if (leg == "right") {
    chain = &right_chain_;
    solver = right_fk_solver_.get();
  } else {
    std::cerr << "[FKSolver] Unknown leg: " << leg << std::endl;
    return result;
  }

  if (!solver || !contact_offset.allFinite()) return result;

  KDL::JntArray q(chain->getNrOfJoints());
  q.data.setZero();  // Unmeasured hip1/hip2 must be mechanically locked at model zero.
  std::array<int, kNumJoints> indices{{-1, -1, -1}};
  const auto& names = getJointNames(leg);
  int index = 0;
  for (unsigned int i = 0; i < chain->getNrOfSegments(); ++i) {
    const auto& joint = chain->getSegment(i).getJoint();
    if (joint.getType() == KDL::Joint::None) continue;
    for (int k = 0; k < kNumJoints; ++k) {
      if (joint.getName() == names[k]) {
        if (!std::isfinite(joint_values[k])) return result;
        indices[k] = index;
        q(index) = joint_values[k];
      }
    }
    ++index;
  }
  for (int index : indices) if (index < 0) return result;
  const KDL::Vector offset(contact_offset.x(), contact_offset.y(), contact_offset.z());

  KDL::Frame frame_out;
  int ret = solver->JntToCart(q, frame_out);
  if (ret < 0) {
    std::cerr << "[FKSolver] FK solver failed with code " << ret << std::endl;
    return result;
  }

  result.p_foot = vectorToEigen(frame_out * offset);
  result.valid = true;

  // 计算位置雅可比 (数值微分)
  // J_pos(:, j) = d(p_foot)/d(q_j)
  const double dq = 1e-6;
  Eigen::Matrix<double, 3, Eigen::Dynamic> J(3, kNumJoints);
  for (int j = 0; j < kNumJoints; ++j) {
    KDL::JntArray q_plus = q;
    q_plus(indices[j]) += dq;
    KDL::Frame frame_plus;
    solver->JntToCart(q_plus, frame_plus);
    Eigen::Vector3d p_plus = vectorToEigen(frame_plus * offset);
    J.col(j) = (p_plus - result.p_foot) / dq;
  }
  result.J_pos = J;

  return result;
}

const std::vector<std::string>& FKSolver::getJointNames(
    const std::string& leg) const {
  if (leg == "left") return left_joint_names_;
  return right_joint_names_;
}

Eigen::Vector3d FKSolver::vectorToEigen(const KDL::Vector& v) const {
  return Eigen::Vector3d(v.x(), v.y(), v.z());
}

}  // namespace exo_state_estimator

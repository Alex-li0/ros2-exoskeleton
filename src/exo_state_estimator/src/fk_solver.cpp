#include "exo_state_estimator/fk_solver.hpp"

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
  // 包含 left_hip1, left_hip2, (left_hip3, left_camera), left_knee, left_ankle
  // 主动关节: left_hip3, left_knee, left_ankle
  // left_hip1, left_hip2 为固定关节(或可设为0)
  // left_camera 为连续关节,默认0
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
                           const std::vector<double>& joint_values) const {
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

  // 构建 KDL joint 数组
  // KDL chain 包含所有关节(active + fixed), 但只有 active joints 需要赋值
  KDL::JntArray q(chain->getNrOfJoints());
  int active_idx = 0;
  for (unsigned int i = 0; i < chain->getNrOfSegments(); ++i) {
    const KDL::Segment& seg = chain->getSegment(i);
    if (seg.getJoint().getType() != KDL::Joint::None) {
      if (active_idx < kNumJoints) {
        q(active_idx) = joint_values[active_idx];
      }
      active_idx++;
    }
  }

  // 如果 chain 的 active joint 数不等于 kNumJoints, 需要映射
  // URDF 的 chain 可能包含 hip1, hip2 等额外关节
  // 我们假设 hip1=0, hip2=0, camera=0, 只设置 hip3, knee, ankle
  if (static_cast<int>(chain->getNrOfJoints()) > kNumJoints) {
    // 找到 hip3, knee, ankle 在 chain 中的位置
    int j = 0;
    for (unsigned int i = 0; i < chain->getNrOfSegments(); ++i) {
      const KDL::Segment& seg = chain->getSegment(i);
      if (seg.getJoint().getType() != KDL::Joint::None) {
        std::string jname = seg.getJoint().getName();
        if (jname.find("hip3") != std::string::npos ||
            jname.find("knee") != std::string::npos ||
            jname.find("ankle") != std::string::npos) {
          // 找到对应值
          for (int k = 0; k < kNumJoints; ++k) {
            if (jname.find(left_joint_names_[k]) != std::string::npos ||
                jname.find(right_joint_names_[k]) != std::string::npos) {
              q(j) = joint_values[k];
              break;
            }
          }
        }
        // 其他 active joints 默认为 0
        j++;
      }
    }
  }

  KDL::Frame frame_out;
  int ret = solver->JntToCart(q, frame_out);
  if (ret < 0) {
    std::cerr << "[FKSolver] FK solver failed with code " << ret << std::endl;
    return result;
  }

  result.p_foot = vectorToEigen(frame_out.p);
  result.J_pos = rotationToEigen(frame_out.M);
  result.valid = true;

  // 计算位置雅可比 (数值微分)
  // J_pos(:, j) = d(p_foot)/d(q_j)
  const double dq = 1e-6;
  Eigen::Matrix<double, 3, Eigen::Dynamic> J(3, kNumJoints);
  for (int j = 0; j < kNumJoints; ++j) {
    KDL::JntArray q_plus = q;
    q_plus(j) += dq;
    KDL::Frame frame_plus;
    solver->JntToCart(q_plus, frame_plus);
    Eigen::Vector3d p_plus = vectorToEigen(frame_plus.p);
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

Eigen::Matrix3d FKSolver::rotationToEigen(const KDL::Rotation& r) const {
  Eigen::Matrix3d m;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) m(i, j) = r(i, j);
  return m;
}

Eigen::Vector3d FKSolver::vectorToEigen(const KDL::Vector& v) const {
  return Eigen::Vector3d(v.x(), v.y(), v.z());
}

}  // namespace exo_state_estimator

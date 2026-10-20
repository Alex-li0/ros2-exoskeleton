#include <gtest/gtest.h>
#include <kdl_parser/kdl_parser.hpp>
#include <gtsam/base/numericalDerivative.h>

#include "exo_state_estimator/contact_detector.hpp"
#include "exo_state_estimator/factor_graph_optimizer.hpp"
#include "exo_state_estimator/kinematic_contact_factor.hpp"
#include "exo_state_estimator/measurement_buffer.hpp"
#include "exo_state_estimator/terrain_height_factor.hpp"

using namespace exo_state_estimator;

TEST(GaitContact, NoMeasurementAndStateOnlyNeverCreateStableSupport) {
  GaitContactTracker tracker;
  EXPECT_EQ(tracker.at(1).state(), ContactState::UNKNOWN);
  ASSERT_TRUE(tracker.addObservation(1.01, 1, true, true, true, .99));
  ASSERT_TRUE(tracker.addObservation(1.01, 2, false, false, true, .99));
  auto state = tracker.at(1.02);
  EXPECT_EQ(state.state(), ContactState::LEFT_STANCE);
  EXPECT_FALSE(state.left.stable);
}

TEST(GaitContact, FullFootEventRequiresConfirmationAndFreshness) {
  GaitContactTracker tracker(.15);
  ASSERT_TRUE(tracker.addEvent(1, 1, GaitEventType::FULL_FOOT_CONTACT, 1, true, .99));
  EXPECT_FALSE(tracker.at(1).left.stable);
  ASSERT_TRUE(tracker.addObservation(1.01, 1, true, false, true, .99));
  EXPECT_FALSE(tracker.at(1.02).left.stable);
  ASSERT_TRUE(tracker.addObservation(1.03, 1, true, true, true, .99));
  const auto first = tracker.at(1.04).left;
  EXPECT_TRUE(first.stable);
  ASSERT_TRUE(tracker.addObservation(1.05, 1, true, true, true, .99));
  EXPECT_EQ(tracker.at(1.06).left.session, first.session);
  EXPECT_FALSE(tracker.at(1.3).left.valid);
  ASSERT_TRUE(tracker.addObservation(1.31, 1, true, true, true, .99));
  EXPECT_FALSE(tracker.at(1.32).left.stable);
}

TEST(GaitContact, ToeUnloadingIsNotWholeFootOffAndReentryChangesSession) {
  GaitContactTracker tracker;
  tracker.addEvent(1, 1, GaitEventType::FULL_FOOT_CONTACT, 1, true, 1);
  tracker.addObservation(1.001, 1, true, true, true, 1);
  const auto first = tracker.at(1.002).left;
  ASSERT_TRUE(first.stable);
  tracker.addEvent(1.01, 1, GaitEventType::TOE_OFF, 2, true, 1);
  auto toe = tracker.at(1.02).left;
  EXPECT_TRUE(toe.contact);
  EXPECT_FALSE(toe.stable);
  tracker.addEvent(1.03, 1, GaitEventType::FULL_FOOT_CONTACT, 3, true, 1);
  tracker.addObservation(1.031, 1, true, true, true, 1);
  EXPECT_GT(tracker.at(1.04).left.session, first.session);
  tracker.addEvent(1.05, 1, GaitEventType::FINAL_CONTACT_OFF, 4, true, 1);
  EXPECT_FALSE(tracker.at(1.06).left.contact);
}

TEST(GaitContact, ExitAndReentryBetweenKeyframesCannotReuseSession) {
  GaitContactTracker tracker;
  tracker.addEvent(1, 1, GaitEventType::FULL_FOOT_CONTACT, 1, true, 1);
  tracker.addObservation(1.001, 1, true, true, true, 1);
  const auto first = tracker.at(1.002).left;
  tracker.addEvent(1.005, 1, GaitEventType::HEEL_OFF, 2, true, 1);
  tracker.addEvent(1.006, 1, GaitEventType::FULL_FOOT_CONTACT, 3, true, 1);
  tracker.addObservation(1.007, 1, true, true, true, 1);
  const auto next = tracker.at(1.01).left;
  EXPECT_TRUE(next.stable);
  EXPECT_NE(first.session, next.session);
}

TEST(GaitContact, DuplicatesIgnoredButLateNewExitInvalidatesSupport) {
  GaitContactTracker tracker;
  tracker.addEvent(1, 1, GaitEventType::FULL_FOOT_CONTACT, 1, true, 1);
  tracker.addObservation(1.001, 1, true, true, true, 1);
  ASSERT_TRUE(tracker.at(1.01).left.stable);
  EXPECT_FALSE(tracker.addEvent(1, 1, GaitEventType::FULL_FOOT_CONTACT, 1, true, 1));
  EXPECT_FALSE(tracker.lastRejectionWasLate());
  EXPECT_TRUE(tracker.at(1.02).left.stable);
  EXPECT_FALSE(tracker.addEvent(1.015, 1, GaitEventType::HEEL_OFF, 2, true, 1));
  EXPECT_TRUE(tracker.lastRejectionWasLate());
  EXPECT_FALSE(tracker.at(1.03).left.stable);
}

TEST(GaitContact, InvalidAndLowConfidenceCannotEnableSupport) {
  GaitContactTracker tracker;
  tracker.addEvent(1, 1, GaitEventType::FULL_FOOT_CONTACT, 1, true, .3);
  tracker.addObservation(1.001, 1, true, true, true, 1);
  EXPECT_FALSE(tracker.at(1.01).left.stable);
  tracker.addEvent(1.02, 1, GaitEventType::FULL_FOOT_CONTACT, 2, true, 1);
  tracker.addObservation(1.021, 1, false, true, true, 1);
  EXPECT_FALSE(tracker.at(1.03).left.valid);
  tracker.reset();
  EXPECT_EQ(tracker.at(0).state(), ContactState::UNKNOWN);
}

TEST(TimeAlignment, InterpolateWithoutExtrapolationOrLargeGaps) {
  std::deque<JointSample> samples{{1., {0., 2.}}, {1.01, {1., 4.}}};
  std::vector<double> result;
  ASSERT_TRUE(interpolateJoints(samples, 1.005, .03, result));
  EXPECT_NEAR(result[0], .5, 1e-10);
  EXPECT_NEAR(result[1], 3., 1e-10);
  EXPECT_FALSE(interpolateJoints(samples, .99, .03, result));
  EXPECT_FALSE(interpolateJoints(samples, 1.02, .03, result));
  EXPECT_FALSE(interpolateJoints(samples, 1.005, .001, result));
}

SensorData frame(double stamp, double x = 0) {
  SensorData data;
  data.timestamp = stamp;
  data.vio_valid = true;
  data.vio_pose = gtsam::Pose3(gtsam::Rot3(), {x, 0, .95});
  return data;
}

TEST(Optimizer, FirstAndSecondFramesAndDuplicateProtection) {
  EstimatorConfig config;
  config.use_imu_factor = false;
  FactorGraphOptimizer optimizer;
  ASSERT_TRUE(optimizer.init(config));
  auto data = frame(1);
  ASSERT_TRUE(optimizer.processSensorData(data));
  data = frame(1.04, .04);
  ASSERT_TRUE(optimizer.processSensorData(data));
  EstimatorOutput before, after;
  ASSERT_TRUE(optimizer.getLatestOutput(before));
  EXPECT_NEAR(before.pose.x(), .04, 1e-8);
  EXPECT_NEAR(before.velocity.x(), 1., 1e-8);
  EXPECT_DOUBLE_EQ(before.timestamp, data.timestamp);
  EXPECT_FALSE(optimizer.processSensorData(data));
  EXPECT_FALSE(optimizer.processSensorData(frame(.99)));
  optimizer.getLatestOutput(after);
  EXPECT_EQ(after.num_factors, before.num_factors);
  EXPECT_DOUBLE_EQ(after.timestamp, before.timestamp);
}

TEST(Optimizer, FixedLagActuallyBoundsStates) {
  EstimatorConfig config;
  config.use_imu_factor = false;
  config.lag_seconds = .1;
  FactorGraphOptimizer optimizer;
  ASSERT_TRUE(optimizer.init(config));
  for (int i = 0; i < 200; ++i) ASSERT_TRUE(optimizer.processSensorData(frame(1 + i * .01, i * .001)));
  EstimatorOutput output;
  ASSERT_TRUE(optimizer.getLatestOutput(output));
  EXPECT_LE(output.num_states, 12);
  EXPECT_LT(output.num_factors, 20);
}

TEST(Optimizer, ImuSamplesAreClippedToKeyframeInterval) {
  EstimatorConfig config;
  config.use_imu_factor = true;
  config.vio_sigmas.setConstant(100.0);  // IMU determines the short-interval acceleration.
  FactorGraphOptimizer optimizer;
  ASSERT_TRUE(optimizer.init(config));
  ASSERT_TRUE(optimizer.processSensorData(frame(1)));
  for (int i = 1; i <= 20; ++i) {
    const double begin = 1 + (i - 1) * .04;
    const double end = 1 + i * .04;
    auto data = frame(end, .25 * std::pow(end - 1, 2));
    // Extra samples on both ends must not add extra integration time.
    for (int j = -1; j <= 9; ++j) data.imu_samples.push_back({begin + j * .005, {.5, 0, 9.81}, {0, 0, 0}});
    ASSERT_TRUE(optimizer.processSensorData(data));
  }
  EstimatorOutput output;
  optimizer.getLatestOutput(output);
  EXPECT_NEAR(output.velocity.x(), .4, .003);
  EXPECT_NEAR(output.pose.x(), .16, .003);
  EXPECT_NEAR(output.pose.z(), .95, .003);
}

TEST(Optimizer, MissingImuIntervalDoesNotCreateFactorAndCanRecover) {
  EstimatorConfig config;
  FactorGraphOptimizer optimizer;
  ASSERT_TRUE(optimizer.init(config));
  ASSERT_TRUE(optimizer.processSensorData(frame(1)));
  auto data = frame(1.04);
  data.imu_samples = {{1, {0, 0, 9.81}, {0, 0, 0}}, {1.04, {0, 0, 9.81}, {0, 0, 0}}};
  EXPECT_FALSE(optimizer.processSensorData(data));
  data.imu_samples.clear();
  for (int j = 0; j <= 8; ++j) data.imu_samples.push_back({1 + j * .005, {0, 0, 9.81}, {0, 0, 0}});
  ASSERT_TRUE(optimizer.processSensorData(data));
  EstimatorOutput output;
  optimizer.getLatestOutput(output);
  EXPECT_NEAR(output.velocity.norm(), 0, 1e-6);
}

TEST(FK, NamedJointMappingSoleOffsetAndJacobianMatchIndependentKDL) {
  FKSolver solver;
  ASSERT_TRUE(solver.initFromFile(TEST_URDF_PATH));
  KDL::Tree tree;
  ASSERT_TRUE(kdl_parser::treeFromFile(TEST_URDF_PATH, tree));
  KDL::Chain chain;
  ASSERT_TRUE(tree.getChain("base_link", "left_foot", chain));
  KDL::ChainFkSolverPos_recursive reference(chain);
  std::vector<double> values{.2, .4, .1};
  Eigen::Vector3d offset(.03, 0, -.06);
  KDL::JntArray q(chain.getNrOfJoints()); q.data.setZero();
  q(2) = .2; q(3) = .4; q(4) = .1;
  KDL::Frame pose;
  ASSERT_EQ(reference.JntToCart(q, pose), 0);
  const auto p = pose * KDL::Vector(offset.x(), offset.y(), offset.z());
  const Eigen::Vector3d expected(p.x(), p.y(), p.z());
  const auto actual = solver.compute("left", values, offset);
  ASSERT_TRUE(actual.valid);
  EXPECT_LT((actual.p_foot - expected).norm(), 1e-10);
  for (int j = 0; j < 3; ++j) {
    auto changed = values; changed[j] += 1e-5;
    const Eigen::Vector3d derivative = (solver.compute("left", changed, offset).p_foot - actual.p_foot) / 1e-5;
    EXPECT_LT((derivative - actual.J_pos.col(j)).norm(), 1e-5);
  }
  EXPECT_GT(actual.J_pos.col(2).norm(), .01);
}

TEST(Factors, SlopedPlaneUsesTheActualQueryPoint) {
  TerrainHeightLandmarkFactor factor(gtsam::symbol_shorthand::L(0), {1, 0, 1}, {-1, 0, 1}, .01);
  EXPECT_NEAR(factor.evaluateError({1, 0, 1})(0), 0, 1e-12);
  EXPECT_NEAR(factor.evaluateError({2, 0, 2})(0), 0, 1e-12);
  EXPECT_GT(std::abs(factor.evaluateError({1, 0, 0})(0)), .5);
}

TEST(Factors, ContactJacobiansMatchPoseRetraction) {
  KinematicContactFactor factor(gtsam::symbol_shorthand::X(0), gtsam::symbol_shorthand::L(0),
      {.2, .1, -.9}, gtsam::noiseModel::Isotropic::Sigma(3, .05));
  gtsam::Pose3 pose(gtsam::Rot3::RzRyRx(.2, -.1, .3), {1., .2, .9});
  gtsam::Point3 landmark(1, .2, 0);
  gtsam::Matrix H1, H2;
  factor.evaluateError(pose, landmark, H1, H2);
  std::function<gtsam::Vector(const gtsam::Pose3&)> fn = [&](const gtsam::Pose3& p) {
    return factor.evaluateError(p, landmark);
  };
  const auto numerical = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(fn, pose);
  EXPECT_LT((H1 - numerical).norm(), 1e-6);
  EXPECT_LT((H2 + Eigen::Matrix3d::Identity()).norm(), 1e-12);
}

TEST(Optimizer, LandmarksSharedOnlyWithinStableSessionsAndSurviveLongStance) {
  EstimatorConfig config;
  config.use_imu_factor = false;
  config.urdf_path = TEST_URDF_PATH;
  config.contact_geometry_calibrated = true;
  config.left_contact_offset = {.03, 0, -.06};
  config.right_contact_offset = {.03, 0, -.06};
  config.lag_seconds = .1;
  FactorGraphOptimizer optimizer;
  ASSERT_TRUE(optimizer.init(config));
  auto data = frame(1);
  data.joint_positions = {.1, .2, .1, .1, .2, .1};
  data.support.left = {true, true, true, 1};
  data.support.right = {true, false, false, 0};
  ASSERT_TRUE(optimizer.processSensorData(data));
  EstimatorOutput output;
  optimizer.getLatestOutput(output);
  const int first = output.left_landmark;
  ASSERT_GE(first, 0);
  for (int i = 1; i < 200; ++i) {
    data.timestamp = 1 + i * .01;
    ASSERT_TRUE(optimizer.processSensorData(data));
    optimizer.getLatestOutput(output);
    EXPECT_EQ(output.left_landmark, first);
  }
  EXPECT_LE(output.num_states, 14);
  data.timestamp += .01;
  data.support.left.stable = false;
  ASSERT_TRUE(optimizer.processSensorData(data));
  optimizer.getLatestOutput(output);
  EXPECT_EQ(output.left_landmark, -1);
  data.timestamp += .01;
  data.support.left.stable = true;
  ++data.support.left.session;
  ASSERT_TRUE(optimizer.processSensorData(data));
  optimizer.getLatestOutput(output);
  EXPECT_NE(output.left_landmark, first);
  EXPECT_GE(output.left_landmark, 0);
}

TEST(Optimizer, UncalibratedGeometryCannotEnableKinematics) {
  EstimatorConfig config;
  config.use_imu_factor = false;
  config.urdf_path = TEST_URDF_PATH;
  FactorGraphOptimizer optimizer;
  ASSERT_TRUE(optimizer.init(config));
  auto data = frame(1);
  data.joint_positions = {0, 0, 0, 0, 0, 0};
  data.support.left = {true, true, true, 1};
  ASSERT_TRUE(optimizer.processSensorData(data));
  EstimatorOutput output;
  optimizer.getLatestOutput(output);
  EXPECT_EQ(output.left_landmark, -1);
}

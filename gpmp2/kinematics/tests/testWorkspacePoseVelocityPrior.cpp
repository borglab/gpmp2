/**
 *  @file    testWorkspacePoseVelocityPrior.cpp
 *  @author  Matthew King-Smith
 **/

#include <CppUnitLite/TestHarness.h>
#include <gpmp2/kinematics/WorkspacePoseVelocityPrior.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpmp2;

/* ************************************************************************** */
TEST(WorkspacePoseVelocityPrior, error) {
  Vector2 a(1, 1), alpha(M_PI / 2, 0), d(0, 0);
  Arm arm = Arm(2, a, alpha, d);
  Vector2 q;
  Vector2 qdot;
  Pose3 des_pose;
  // Vector6 des_vel;
  // WorkspacePoseVelocityPrior factor;
  Vector actual, expect;
  Matrix H_pose_exp, H_pose_act, H_vel_exp, H_vel_act;
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(12, 1.0);

  q = Vector2(M_PI / 4.0, -M_PI / 2);
  qdot = (Vector2() << 0.1, 0.1).finished();
  des_pose = Pose3();
  Vector6 des_vel = (Vector6() << 0, 0, 0, 0, 0, 0).finished();
  WorkspacePoseVelocityPrior factor(0, 0, cost_model, arm, des_pose, des_vel);
  actual = factor.evaluateError(q, qdot, &H_pose_act, &H_vel_act);
  expect = (Vector(12) << 00.613943126, 1.48218982, -0.613943126, 1.1609828,
            0.706727485, -0.547039678, 0, -0.141421356237309, 0,
            -0.070710678118655, 0.070710678118655, -0.1000)
               .finished();
  H_pose_exp = numericalDerivative11(
      std::function<Vector(const Vector2&)>(
          std::bind(&WorkspacePoseVelocityPrior::evaluateError, factor,
                    std::placeholders::_1, qdot, nullptr, nullptr)),
      q, 1e-6);

  H_vel_exp = numericalDerivative11(
      std::function<Vector(const Vector2&)>(
          std::bind(&WorkspacePoseVelocityPrior::evaluateError, factor, q,
                    std::placeholders::_1, nullptr, nullptr)),
      qdot, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(H_pose_exp, H_pose_act, 1e-6));
  EXPECT(assert_equal(H_vel_exp, H_vel_act, 1e-6));
}

/* ************************************************************************** */
TEST(WorkspacePoseVelocityPrior, optimization) {
  noiseModel::Gaussian::shared_ptr cost_model =
      noiseModel::Isotropic::Sigma(12, 0.1);

  Vector a = (Vector(2) << 1, 1).finished();
  Vector alpha = (Vector(2) << 0, 0).finished();
  Vector d = (Vector(2) << 0, 0).finished();
  Arm arm = Arm(2, a, alpha, d);
  Pose3 des_pose = Pose3(Rot3(), Point3(2, 0, 0));
  Vector6 des_vel = Vector6(0, 0, 0, 0, 0, 0);
  Key qkey = Symbol('x', 0);
  Key qdotkey = Symbol('v', 0);
  Vector q = (Vector(2) << 0, 0).finished();
  Vector qdot = (Vector(2) << 0, 0).finished();
  Vector qinit = (Vector(2) << M_PI / 2, M_PI / 2).finished();
  Vector qdotinit = (Vector(2) << 0.1, 0.1).finished();

  NonlinearFactorGraph graph;
  graph.add(WorkspacePoseVelocityPrior(qkey, qdotkey, cost_model, arm, des_pose,
                                       des_vel));
  Values init_values;
  init_values.insert(qkey, qinit);
  init_values.insert(qdotkey, qdotinit);

  LevenbergMarquardtParams parameters;
  parameters.setVerbosity("ERROR");
  parameters.setAbsoluteErrorTol(1e-12);
  LevenbergMarquardtOptimizer optimizer(graph, init_values, parameters);
  optimizer.optimize();
  Values results = optimizer.values();

  EXPECT_DOUBLES_EQUAL(0, graph.error(results), 1e-3);
  EXPECT(assert_equal(q, results.at<Vector>(qkey), 1e-3));
  EXPECT(assert_equal(qdot, results.at<Vector>(qdotkey), 1e-3));
}

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

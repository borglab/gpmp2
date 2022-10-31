/**
 *  @file testGaussianProcessPriorLieLTIPose2.cpp
 *  @author Matt King-Smith
 **/

#include <CppUnitLite/TestHarness.h>
#include <gpmp2/gp/GaussianProcessPriorLieLTIPose2.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpmp2;

/* ************************************************************************** */
TEST(GaussianProcessPriorLieLTIPose2, Factor) {
  const double delta_t = 0.5;
  Matrix Qc = 0.01 * Matrix::Identity(3, 3);
  noiseModel::Gaussian::shared_ptr Qc_model =
      noiseModel::Gaussian::Covariance(Qc);
  Key key_pose1 = Symbol('x', 1), key_control = Symbol('u',1), key_pose2 = Symbol('x', 2);
  Key key_vel1 = Symbol('v', 1), key_vel2 = Symbol('v', 2);
  GaussianProcessPriorLieLTIPose2 factor(key_pose1, key_vel1, key_pose2, key_vel2, key_control,
                                   delta_t, Qc_model);
  Pose2 p1, p2;
  Vector3 v1, v2, c1;
  Matrix actualH1, actualH2, actualH3, actualH4, actualH5;
  Matrix expectH1, expectH2, expectH3, expectH4, expectH5;
  Vector actual, expect;

  // some random stuff just for testing jacobian (error is not zero)
  p1 = Pose2(-0.1, 1.2, 0.3);
  //p2 = Pose2(-0.0999998780487805, 1.20000036585366, 0.300008771929825);
  v1 = (Vector3() << 0, 0, 0).finished();
  //v2 = (Vector3() << 2.4390243902439e-05, 7.31707317073171e-05, 0.00175438596491228).finished();
  p2 = Pose2(-0.0996951219512195, 1.20091463414634, 0.321929824561404);
  v2 = (Vector3() << 0.00121951219512195, 0.00365853658536585, 0.087719298245614)
           .finished();
  c1 = (Vector3() << 1, 3, 5).finished();
  expect = (Vector(6) << 0, 0, 0, 0, 0, 0).finished();
  actual = factor.evaluateError(p1, v1, p2, v2, c1, actualH1, actualH2, actualH3,
                                actualH4, actualH5);
  expectH1 = numericalDerivative11(
      std::function<Vector(const Pose2&)>(
          std::bind(&GaussianProcessPriorLieLTIPose2::evaluateError, factor,
                    std::placeholders::_1, v1, p2, v2, c1, boost::none, boost::none, boost::none,
                    boost::none, boost::none)),
      p1, 1e-6);
  expectH2 = numericalDerivative11(
      std::function<Vector(const Vector3&)>(
          std::bind(&GaussianProcessPriorLieLTIPose2::evaluateError, factor, p1,
                    std::placeholders::_1, p2, v2, c1, boost::none, boost::none, boost::none,
                    boost::none, boost::none)),
      v1, 1e-6);
  expectH3 = numericalDerivative11(
      std::function<Vector(const Pose2&)>(
          std::bind(&GaussianProcessPriorLieLTIPose2::evaluateError, factor, p1, v1,
                    std::placeholders::_1, v2, c1, boost::none, boost::none, boost::none,
                    boost::none, boost::none)),
      p2, 1e-6);
  expectH4 = numericalDerivative11(
      std::function<Vector(const Vector3&)>(
          std::bind(&GaussianProcessPriorLieLTIPose2::evaluateError, factor, p1, v1,
                    p2, std::placeholders::_1, c1, boost::none, boost::none, boost::none,
                    boost::none, boost::none)),
      v2, 1e-6);
  expectH5 = numericalDerivative11(
      std::function<Vector(const Vector3&)>(
          std::bind(&GaussianProcessPriorLieLTIPose2::evaluateError, factor, p1, v1,
                    p2, v2, std::placeholders::_1, boost::none, boost::none, boost::none,
                    boost::none, boost::none)),
      c1, 1e-6);
  EXPECT(assert_equal(expect, actual, 1e-6));
  EXPECT(assert_equal(expectH1, actualH1, 1e-6));
  EXPECT(assert_equal(expectH2, actualH2, 1e-6));
  EXPECT(assert_equal(expectH3, actualH3, 1e-6));
  EXPECT(assert_equal(expectH4, actualH4, 1e-6));
  EXPECT(assert_equal(expectH5, actualH5, 1e-6));
}

/* ************************************************************************** */
// TEST(GaussianProcessPriorPose2, Optimization) {
//   /**
//    * A simple graph:
//    *
//    * p1   p2
//    * |    |
//    * x1   x2
//    *  \  /
//    *   gp
//    *  /  \
//    * v1  v2
//    *
//    * p1 and p2 are pose prior factor to fix the poses, gp is the GP factor
//    * that get correct velocity of v2
//    */

//   noiseModel::Isotropic::shared_ptr model_prior =
//       noiseModel::Isotropic::Sigma(3, 0.001);
//   double delta_t = 1;
//   Matrix Qc = 0.01 * Matrix::Identity(3, 3);
//   noiseModel::Gaussian::shared_ptr Qc_model =
//       noiseModel::Gaussian::Covariance(Qc);

//   Pose2 pose1(0, 0, 0), pose2(1, 0, 0);
//   Vector v1 = (Vector(3) << 1, 0, 0).finished();
//   Vector v2 = (Vector(3) << 2.0, -0.5, 0.6).finished();  // rnd value

//   NonlinearFactorGraph graph;
//   graph.add(PriorFactor<Pose2>(Symbol('x', 1), pose1, model_prior));
//   graph.add(PriorFactor<Pose2>(Symbol('x', 2), pose2, model_prior));
//   // graph.add(PriorFactor<Vector6>(Symbol('v', 1), v1, model_prior));
//   graph.add(GaussianProcessPriorPose2(Symbol('x', 1), Symbol('v', 1),
//                                       Symbol('x', 2), Symbol('v', 2), delta_t,
//                                       Qc_model));

//   Values init_values;
//   init_values.insert(Symbol('x', 1), pose1);
//   init_values.insert(Symbol('v', 1), v1);
//   init_values.insert(Symbol('x', 2), pose2);
//   init_values.insert(Symbol('v', 2), v2);

//   GaussNewtonParams parameters;
//   GaussNewtonOptimizer optimizer(graph, init_values, parameters);
//   optimizer.optimize();
//   Values values = optimizer.values();

//   EXPECT_DOUBLES_EQUAL(0, graph.error(values), 1e-6);
//   EXPECT(assert_equal(pose1, values.at<Pose2>(Symbol('x', 1)), 1e-6));
//   EXPECT(assert_equal(pose2, values.at<Pose2>(Symbol('x', 2)), 1e-6));
//   EXPECT(assert_equal(v1, values.at<Vector>(Symbol('v', 1)), 1e-6));
//   EXPECT(assert_equal(v1, values.at<Vector>(Symbol('v', 2)), 1e-6));
// }

/* ************************************************************************** */
/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

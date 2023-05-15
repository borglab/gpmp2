/**
 *  @file testArm.cpp
 *  @author Mustafa Mukadam, Jing Dong
 *  @date Nov 11, 2015
 **/

#include <CppUnitLite/TestHarness.h>
#include <gpmp2/kinematics/Arm.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace gpmp2;

// fk wrapper
Pose3 fkpose(const Arm& arm, const Vector& jp, const Vector& jv, size_t i) {
  vector<Pose3> pos;
  vector<Vector6> vel;
  arm.forwardKinematics(jp, jv, pos, &vel);
  return pos.at(i);
}

Vector6 fkvelocity(const Arm& arm, const Vector& jp, const Vector& jv,
                   size_t i) {
  vector<Pose3> pos;
  vector<Vector6> vel;
  arm.forwardKinematics(jp, jv, pos, &vel);
  return vel.at(i);
}

// /* ************************************************************************** */
// TEST_DISABLED(Arm, 2linkPlanarExamples) {
//   // 2 link simple example, with none zero base poses
//   Vector2 a(1, 1), alpha(0, 0), d(0, 0);
//   Pose3 base_pose(Rot3::Ypr(M_PI / 4.0, 0, 0), Point3(2.0, 1.0, -1.0));
//   short int dof = 2;
//   Arm arm(dof, a, alpha, d, base_pose);
//   Vector2 q, qdot;
//   Vector qdymc;  // dynamic size version qdot
//   vector<Pose3> pvec_exp, pvec_act;
//   vector<Vector3> vvec_exp, vvec_act;
//   vector<Matrix> vJp_exp, vJp_act, vJv_exp, vJv_act, pJp_exp, pJp_act;

//   // origin with zero vel
//   q = Vector2(0.0, 0.0);
//   qdot = Vector2(0.0, 0.0);

//   // forward kinematics
//   arm.forwardKinematics(q, qdot, pvec_act, &vvec_act, &pJp_act, &vJp_act,
//                         &vJv_act);

//   // expected joint positions
//   pvec_exp.push_back(Pose3(Rot3::Ypr(M_PI / 4.0, 0, 0),
//                            Point3(2.707106781186548, 1.707106781186548, -1.0)));
//   pvec_exp.push_back(Pose3(Rot3::Ypr(M_PI / 4.0, 0, 0),
//                            Point3(3.414213562373095, 2.414213562373095, -1.0)));

//   // expected joint velocities
//   vvec_exp.push_back(Vector3(0, 0, 0));
//   vvec_exp.push_back(Vector3(0, 0, 0));

//   for (short int kk = 0; kk < dof; kk++) {
//     pJp_exp.push_back(numericalDerivative11(
//         std::function<Pose3(const Vector2&)>(
//             std::bind(&fkpose, arm, std::placeholders::_1, qdot, size_t(kk))),
//         q, 1e-6));
//     vJp_exp.push_back(numericalDerivative11(
//         std::function<Vector3(const Vector2&)>(std::bind(
//             &fkvelocity, arm, std::placeholders::_1, qdot, size_t(kk))),
//         q, 1e-6));
//     vJv_exp.push_back(numericalDerivative11(
//         std::function<Vector3(const Vector2&)>(
//             std::bind(&fkvelocity, arm, q, std::placeholders::_1, size_t(kk))),
//         qdot, 1e-6));
//     EXPECT(assert_equal(pvec_exp[kk], pvec_act[kk], 1e-9));
//     EXPECT(assert_equal(vvec_exp[kk], vvec_act[kk], 1e-9));
//     EXPECT(assert_equal(pJp_exp[kk], pJp_act[kk], 1e-6));
//     EXPECT(assert_equal(vJp_exp[kk], vJp_act[kk], 1e-6));
//     EXPECT(assert_equal(vJv_exp[kk], vJv_act[kk], 1e-6));
//   }

//   // origin with none zero vel of j0
//   q = Vector2(0.0, 0.0);
//   qdot = Vector2(1.0, 0.0);

//   // forward kinematics
//   arm.forwardKinematics(q, qdot, pvec_act, &vvec_act, &pJp_act, &vJp_act,
//                         &vJv_act);

//   // expected joint positions
//   pvec_exp.clear();
//   pvec_exp.push_back(Pose3(Rot3::Ypr(M_PI / 4.0, 0, 0),
//                            Point3(2.707106781186548, 1.707106781186548, -1.0)));
//   pvec_exp.push_back(Pose3(Rot3::Ypr(M_PI / 4.0, 0, 0),
//                            Point3(3.414213562373095, 2.414213562373095, -1.0)));

//   // expected joint velocities
//   vvec_exp.clear();
//   vvec_exp.push_back(Vector3(-0.707106781186548, 0.707106781186548, 0));
//   vvec_exp.push_back(Vector3(-1.414213562373095, 1.414213562373095, 0));

//   pJp_exp.clear();
//   vJp_exp.clear();
//   vJv_exp.clear();
//   for (short int kk = 0; kk < dof; kk++) {
//     pJp_exp.push_back(numericalDerivative11(
//         std::function<Pose3(const Vector2&)>(
//             std::bind(&fkpose, arm, std::placeholders::_1, qdot, size_t(kk))),
//         q, 1e-6));
//     vJp_exp.push_back(numericalDerivative11(
//         std::function<Vector3(const Vector2&)>(std::bind(
//             &fkvelocity, arm, std::placeholders::_1, qdot, size_t(kk))),
//         q, 1e-6));
//     vJv_exp.push_back(numericalDerivative11(
//         std::function<Vector3(const Vector2&)>(
//             std::bind(&fkvelocity, arm, q, std::placeholders::_1, size_t(kk))),
//         qdot, 1e-6));
//     EXPECT(assert_equal(pvec_exp[kk], pvec_act[kk], 1e-9));
//     EXPECT(assert_equal(vvec_exp[kk], vvec_act[kk], 1e-9));
//     EXPECT(assert_equal(pJp_exp[kk], pJp_act[kk], 1e-6));
//     EXPECT(assert_equal(vJp_exp[kk], vJp_act[kk], 1e-6));
//     EXPECT(assert_equal(vJv_exp[kk], vJv_act[kk], 1e-6));
//   }

//   // origin with none zero vel of j1
//   q = Vector2(0.0, 0.0);
//   qdot = Vector2(0.0, 1.0);

//   // forward kinematics
//   arm.forwardKinematics(q, qdot, pvec_act, &vvec_act, &pJp_act, &vJp_act,
//                         &vJv_act);

//   // expected joint positions
//   pvec_exp.clear();
//   pvec_exp.push_back(Pose3(Rot3::Ypr(M_PI / 4.0, 0, 0),
//                            Point3(2.707106781186548, 1.707106781186548, -1.0)));
//   pvec_exp.push_back(Pose3(Rot3::Ypr(M_PI / 4.0, 0, 0),
//                            Point3(3.414213562373095, 2.414213562373095, -1.0)));

//   // expected joint velocities
//   vvec_exp.clear();
//   vvec_exp.push_back(Vector3(0, 0, 0));
//   vvec_exp.push_back(Vector3(-0.707106781186548, 0.707106781186548, 0));

//   pJp_exp.clear();
//   vJp_exp.clear();
//   vJv_exp.clear();
//   for (short int kk = 0; kk < dof; kk++) {
//     pJp_exp.push_back(numericalDerivative11(
//         std::function<Pose3(const Vector2&)>(
//             std::bind(&fkpose, arm, std::placeholders::_1, qdot, size_t(kk))),
//         q, 1e-6));
//     vJp_exp.push_back(numericalDerivative11(
//         std::function<Vector3(const Vector2&)>(std::bind(
//             &fkvelocity, arm, std::placeholders::_1, qdot, size_t(kk))),
//         q, 1e-6));
//     vJv_exp.push_back(numericalDerivative11(
//         std::function<Vector3(const Vector2&)>(
//             std::bind(&fkvelocity, arm, q, std::placeholders::_1, size_t(kk))),
//         qdot, 1e-6));
//     EXPECT(assert_equal(pvec_exp[kk], pvec_act[kk], 1e-9));
//     EXPECT(assert_equal(vvec_exp[kk], vvec_act[kk], 1e-9));
//     EXPECT(assert_equal(pJp_exp[kk], pJp_act[kk], 1e-6));
//     EXPECT(assert_equal(vJp_exp[kk], vJp_act[kk], 1e-6));
//     EXPECT(assert_equal(vJv_exp[kk], vJv_act[kk], 1e-6));
//   }

//   // origin with none zero pos/vel
//   q = Vector2(M_PI / 4.0, M_PI / 4.0);
//   qdot = Vector2(0.1, 0.1);

//   // forward kinematics
//   arm.forwardKinematics(q, qdot, pvec_act, &vvec_act, &pJp_act, &vJp_act,
//                         &vJv_act);

//   // expected joint positions
//   pvec_exp.clear();
//   pvec_exp.push_back(
//       Pose3(Rot3::Ypr(M_PI / 2.0, 0, 0), Point3(2.0, 2.0, -1.0)));
//   pvec_exp.push_back(
//       Pose3(Rot3::Ypr(M_PI * 0.75, 0, 0),
//             Point3(1.2928932188134528, 2.707106781186548, -1.0)));

//   // expected joint velocities
//   vvec_exp.clear();
//   vvec_exp.push_back(Vector3(-0.1, 0, 0));
//   vvec_exp.push_back(Vector3(-0.241421356237309, -0.141421356237309, 0));

//   pJp_exp.clear();
//   vJp_exp.clear();
//   vJv_exp.clear();
//   for (short int kk = 0; kk < dof; kk++) {
//     pJp_exp.push_back(numericalDerivative11(
//         std::function<Pose3(const Vector2&)>(
//             std::bind(&fkpose, arm, std::placeholders::_1, qdot, size_t(kk))),
//         q, 1e-6));
//     vJp_exp.push_back(numericalDerivative11(
//         std::function<Vector3(const Vector2&)>(std::bind(
//             &fkvelocity, arm, std::placeholders::_1, qdot, size_t(kk))),
//         q, 1e-6));
//     vJv_exp.push_back(numericalDerivative11(
//         std::function<Vector3(const Vector2&)>(
//             std::bind(&fkvelocity, arm, q, std::placeholders::_1, size_t(kk))),
//         qdot, 1e-6));
//     EXPECT(assert_equal(pvec_exp[kk], pvec_act[kk], 1e-9));
//     EXPECT(assert_equal(vvec_exp[kk], vvec_act[kk], 1e-9));
//     EXPECT(assert_equal(pJp_exp[kk], pJp_act[kk], 1e-6));
//     EXPECT(assert_equal(vJp_exp[kk], vJp_act[kk], 1e-6));
//     EXPECT(assert_equal(vJv_exp[kk], vJv_act[kk], 1e-6));
//   }
// }

// /* ************************************************************************** */
// TEST_DISABLED(Arm, 3link3Dexample) {
//   // random 3 link arm
//   // no link rotation ground truth

//   Vector3 a(3.6, 9.8, -10.2), alpha(-0.4, 0.7, 0.9), d(-12.3, -3.4, 5.0);
//   short int dof = 3;
//   Arm arm(dof, a, alpha, d);
//   Vector3 q, qdot;
//   vector<Point3> pvec_exp;
//   vector<Pose3> pvec_act;
//   vector<Vector3> vvec_exp, vvec_act;
//   vector<Matrix> vJp_exp, vJp_act, vJv_exp, vJv_act;
//   vector<Matrix> pJp_exp, pJp_act;

//   // random example
//   q = Vector3(-1.1, 6.3, 2.4);
//   qdot = Vector3(10.78, 3, -6.3);

//   // forward kinematics
//   arm.forwardKinematics(q, qdot, pvec_act, &vvec_act, &pJp_act, &vJp_act,
//                         &vJv_act);

//   // expected joint positions
//   pvec_exp.push_back(Point3(1.6329, -3.2083, -12.3000));
//   pvec_exp.push_back(Point3(5.0328, -12.4727, -15.4958));
//   pvec_exp.push_back(Point3(1.4308, -22.9046, -12.8049));

//   // expected joint velocities
//   vvec_exp.push_back(Vector3(34.5860, 17.6032, 0));
//   vvec_exp.push_back(Vector3(158.3610, 66.9758, -11.4473));
//   vvec_exp.push_back(Vector3(240.7202, 32.6894, -34.1207));

//   for (short int kk = 0; kk < dof; kk++) {
//     pJp_exp.push_back(numericalDerivative11(
//         std::function<Pose3(const Vector3&)>(
//             std::bind(&fkpose, arm, std::placeholders::_1, qdot, size_t(kk))),
//         q, 1e-6));
//     vJp_exp.push_back(numericalDerivative11(
//         std::function<Vector3(const Vector3&)>(std::bind(
//             &fkvelocity, arm, std::placeholders::_1, qdot, size_t(kk))),
//         q, 1e-6));
//     vJv_exp.push_back(numericalDerivative11(
//         std::function<Vector3(const Vector3&)>(
//             std::bind(&fkvelocity, arm, q, std::placeholders::_1, size_t(kk))),
//         qdot, 1e-6));
//     EXPECT(assert_equal(pvec_exp[kk], pvec_act[kk].translation(), 1e-3));
//     EXPECT(assert_equal(vvec_exp[kk], vvec_act[kk], 1e-3));
//     EXPECT(assert_equal(pJp_exp[kk], pJp_act[kk], 1e-6));
//     EXPECT(assert_equal(vJp_exp[kk], vJp_act[kk], 1e-6));
//     EXPECT(assert_equal(vJv_exp[kk], vJv_act[kk], 1e-6));
//   }
// }

// /* ************************************************************************** */
// TEST_DISABLED(Arm, WAMexample) {
//   // WAM arm example, only joint position, no rotation
//   Vector7 a = (Vector7() << 0.0, 0.0, 0.045, -0.045, 0.0, 0.0, 0.0).finished();
//   Vector7 alpha = (Vector7() << -M_PI / 2.0, M_PI / 2.0, -M_PI / 2.0,
//                    M_PI / 2.0, -M_PI / 2.0, M_PI / 2.0, 0.0)
//                       .finished();
//   Vector7 d = (Vector7() << 0.0, 0.0, 0.55, 0.0, 0.3, 0.0, 0.06).finished();
//   short int dof = 7;
//   Arm arm(dof, a, alpha, d);
//   Vector7 q, qdot;
//   vector<Point3> pvec_exp;
//   vector<Pose3> pvec_act1, pvec_act2;
//   vector<Vector3> vvec_exp, vvec_act;
//   vector<Matrix> pJp_exp, pJp_act1, pJp_act2, vJp_exp, vJp_act, vJv_exp,
//       vJv_act;

//   // example
//   q = (Vector7() << 1.58, 1.1, 0, 1.7, 0, -1.24, 1.57).finished();
//   qdot = (Vector7() << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished();

//   pvec_exp.push_back(Point3(0, 0, 0));
//   pvec_exp.push_back(Point3(0, 0, 0));
//   pvec_exp.push_back(Point3(-0.0047, 0.5106, 0.2094));
//   pvec_exp.push_back(Point3(-0.0051, 0.5530, 0.2244));
//   pvec_exp.push_back(Point3(-0.0060, 0.6534, -0.0582));
//   pvec_exp.push_back(Point3(-0.0060, 0.6534, -0.0582));
//   pvec_exp.push_back(Point3(-0.0066, 0.7134, -0.0576));

//   vvec_exp.push_back(Vector3(0, 0, 0));
//   vvec_exp.push_back(Vector3(0, 0, 0));
//   vvec_exp.push_back(Vector3(-0.0557, 0.0204, -0.0511));
//   vvec_exp.push_back(Vector3(-0.0606, 0.0234, -0.0595));
//   vvec_exp.push_back(Vector3(-0.0999, -0.0335, -0.0796));
//   vvec_exp.push_back(Vector3(-0.0999, -0.0335, -0.0796));
//   vvec_exp.push_back(Vector3(-0.1029, -0.0333, -0.0976));

//   // full fk with velocity
//   arm.forwardKinematics(q, qdot, pvec_act1, &vvec_act, &pJp_act1, &vJp_act,
//                         &vJv_act);
//   // fk no velocity
//   arm.forwardKinematics(q, {}, pvec_act2, nullptr, &pJp_act2);

//   for (short int kk = 0; kk < dof; kk++) {
//     pJp_exp.push_back(numericalDerivative11(
//         std::function<Pose3(const Vector7&)>(
//             std::bind(&fkpose, arm, std::placeholders::_1, qdot, size_t(kk))),
//         q, 1e-6));
//     vJp_exp.push_back(numericalDerivative11(
//         std::function<Vector3(const Vector7&)>(std::bind(
//             &fkvelocity, arm, std::placeholders::_1, qdot, size_t(kk))),
//         q, 1e-6));
//     vJv_exp.push_back(numericalDerivative11(
//         std::function<Vector3(const Vector7&)>(
//             std::bind(&fkvelocity, arm, q, std::placeholders::_1, size_t(kk))),
//         qdot, 1e-6));

//     EXPECT(assert_equal(pvec_exp[kk], pvec_act1[kk].translation(), 1e-3));
//     EXPECT(assert_equal(pvec_exp[kk], pvec_act2[kk].translation(), 1e-3));
//     EXPECT(assert_equal(vvec_exp[kk], vvec_act[kk], 1e-3));
//     EXPECT(assert_equal(pJp_exp[kk], pJp_act1[kk], 1e-6));
//     EXPECT(assert_equal(pJp_exp[kk], pJp_act2[kk], 1e-6));
//     EXPECT(assert_equal(vJp_exp[kk], vJp_act[kk], 1e-6));
//     EXPECT(assert_equal(vJv_exp[kk], vJv_act[kk], 1e-6));
//   }
// }

/* ************************************************************************** */
TEST(Arm, KinovaGen3) {
  // Kinova Gen3 7DOF arm modified DH example
  // Verify inertial joint positions, linear velocities, and end-effector pose

  Vector7 a = (Vector7() << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
  Vector7 alpha = (Vector7() << M_PI, M_PI / 2.0, -M_PI / 2.0, M_PI / 2.0,
                   -M_PI / 2.0, M_PI / 2.0, -M_PI / 2.0)
                      .finished();
  Vector7 d =
      (Vector7() << -0.2848, -0.0118, -0.4208, -0.0128, -0.3143, 0.0, -0.1674)
          .finished();

  short int dof = 7;
  bool modDF = true;
  Arm arm(dof, a, alpha, d, modDF);

  Vector7 q, qdot;

  // inertial joint positions
  vector<Point3> pi_exp;
  // computed joint poses and expect end-effector pose
  vector<Pose3> pi_act_wo_fwdVel, pi_act, pEE_exp;
  // Jacobian w.r.t. joint angles and velocities
  vector<Matrix> pJp_exp, pJp_act_wo_fwdVel, pJp_act, vJp_exp, vJp_act, vJv_exp,
      vJv_act;

  // joint inertial linear velocities
  vector<Vector6> vi_exp, vi_act;

  // nontrivial configuration
  q = (Vector7() << M_PI, M_PI / 4, M_PI / 6, -M_PI / 6, -M_PI / 4, -M_PI / 2,
       -M_PI)
          .finished();
  qdot = (Vector7() << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished();

  // expected joint positions
  pi_exp.push_back(Point3(0, 0, 0.2848));              // J1
  pi_exp.push_back(Point3(0, 0.0118, 0.2848));         // J2
  pi_exp.push_back(Point3(-0.2976, 0.0118, 0.5824));   // J3
  pi_exp.push_back(Point3(-0.2930, 0.0229, 0.5869));   // J4
  pi_exp.push_back(Point3(-0.3893, -0.0557, 0.8756));  // J5
  pi_exp.push_back(Point3(-0.3893, -0.0557, 0.8756));  // J6
  pi_exp.push_back(Point3(-0.2428, -0.0044, 0.9384));  // J7

  // expected end-effector pose
  pEE_exp.push_back(
      Pose3(Rot3(0.3062, 0.3750, -0.8750, 0.2500, -0.9186, -0.3062, -0.9186,
                 -0.1250, -0.3750),
            Point3(-0.2428, -0.0044, 0.9384)));  // J7 pose (end-effector)

  // expected joint velocities
  vi_exp.push_back(Vector6(0, 0, 0, 0, 0, 0.1));                          // J1
  vi_exp.push_back(Vector6(0, 0, 0, 0, 0.0, 0));                       // J2
  vi_exp.push_back(Vector6(-0.0298, 0, -0.0298, 0.0707, -0.1, 0.070));  // J3
  vi_exp.push_back(
      Vector6(-0.0294, -0.0006, -0.0285, 0.0354, -0.1866, -0.1061));  // J4
  vi_exp.push_back(
      Vector6(-0.0916, -0.0006, -0.0493, 0.066, -0.1616, -0.1979));  // J5
  vi_exp.push_back(
      Vector6(-0.0916, -0.0006, -0.0493, 0.1035, -0.2535, -0.2104));  // J6
  vi_exp.push_back(Vector6(-0.0968, -0.0380, -0.0068, 0.1035, -0.2535,
                           -0.2104));  // J7

  // fk no velocity
  arm.forwardKinematics(q, {}, pi_act_wo_fwdVel, nullptr, &pJp_act_wo_fwdVel);

  // full fk with velocity
  arm.forwardKinematics(q, qdot, pi_act, &vi_act, &pJp_act, &vJp_act, &vJv_act);

  for (int kk = 0; kk < 3; kk++) {
    pJp_exp.push_back(numericalDerivative11(
        std::function<Pose3(const Vector7&)>(
            std::bind(&fkpose, arm, std::placeholders::_1, qdot, size_t(kk))),
        q, 1e-6));

    vJp_exp.push_back(numericalDerivative11(
        std::function<Vector6(const Vector7&)>(std::bind(
            &fkvelocity, arm, std::placeholders::_1, qdot, size_t(kk))),
        q, 1e-6));

    vJv_exp.push_back(numericalDerivative11(
        std::function<Vector6(const Vector7&)>(
            std::bind(&fkvelocity, arm, q, std::placeholders::_1, size_t(kk))),
        qdot, 1e-6));

    // check end-effector pose
    if (kk == (dof - 1)) {
      EXPECT(assert_equal(pEE_exp[0], pi_act_wo_fwdVel[kk], 1e-3));
      EXPECT(assert_equal(pEE_exp[0], pi_act[kk], 1e-3));
    }

    EXPECT(assert_equal(pJp_exp[kk], pJp_act_wo_fwdVel[kk], 1e-6));
    EXPECT(assert_equal(pi_exp[kk], pi_act_wo_fwdVel[kk].translation(), 1e-3));
    EXPECT(assert_equal(pi_exp[kk], pi_act[kk].translation(), 1e-3));
    EXPECT(assert_equal(vi_exp[kk], vi_act[kk], 1e-3));
    // EXPECT(assert_equal(pJp_exp[kk], pJp_act[kk], 1e-6));
    // EXPECT(assert_equal(vJp_exp[kk], vJp_act[kk], 1e-6));
    // EXPECT(assert_equal(vJv_exp[kk], vJv_act[kk], 1e-6));
  }
}
/* ************************************************************************** */

/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

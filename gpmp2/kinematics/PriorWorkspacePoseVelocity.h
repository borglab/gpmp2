/**
 *  @file   PriorWorkspacePoseVelocity.h
 *  @brief  Prior defined on the workspace pose and velocity of final joint
 *(end-effector) of a robot given its state in configuration space
 *  @author Matthew King-Smith
 *  @date   May 16, 2023
 **/

#pragma once

#include <gpmp2/kinematics/Arm.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <iostream>
#include <vector>

namespace gpmp2 {

/**
 * binary factor Gaussian prior defined on the workspace pose and velocities
 */
class PriorWorkspacePoseVelocity
    : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Vector> {
 private:
  // typedefs
  typedef PriorWorkspacePoseVelocity This;
  typedef gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Vector> Base;

  // arm
  Arm arm_;

  // desired workspace pose
  gtsam::Pose3 des_pose_;

  // desired velocity (linear and angular)
  gtsam::Vector6 des_vel_;

 public:
  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;

  /// Default constructor
  PriorWorkspacePoseVelocity() {}

  /**
   * Constructor
   * @param cost_model cost function covariance
   * @param des_vel desired velocity (linear and angular)
   */
  PriorWorkspacePoseVelocity(gtsam::Key poseKey, gtsam::Key velKey,
                             const gtsam::SharedNoiseModel& cost_model,
                             const Arm& arm, const gtsam::Pose3& des_pose,
                             const gtsam::Vector6& des_vel)
      : Base(cost_model, poseKey, velKey),
        arm_(arm),
        des_pose_(des_pose),
        des_vel_(des_vel) {}

  ~PriorWorkspacePoseVelocity() {}

  /// error function
  gtsam::Vector evaluateError(
      const gtsam::Vector& joint_conf, const gtsam::Vector& joint_rates,
      gtsam::OptionalMatrixType H1 = nullptr,
      gtsam::OptionalMatrixType H2 = nullptr) const {
    using namespace gtsam;

    // fk
    std::vector<Pose3> joint_pose;
    std::vector<Vector6> joint_vel;
    std::vector<Matrix> J_jpx_jp, J_jvx_jp, J_jvx_jv;
    arm_.forwardKinematics(joint_conf, joint_rates, joint_pose, &joint_vel,
                           &J_jpx_jp, &J_jvx_jp, &J_jvx_jv);

    Matrix66 H_ep;
    Vector pose_error = des_pose_.logmap(joint_pose[arm_.dof() - 1], {}, H_ep);
    Vector vel_error = des_vel_ - joint_vel[arm_.dof() - 1];
    if (H1) {
      // Jacobian for the joint positions
      *H1 = (Matrix(6, 2 * arm_.dof()) << H_ep * J_jpx_jp[arm_.dof() - 1],
             J_jvx_jp[arm_.dof() - 1])
                .finished();  // H_ep * J_jpx_jp[arm_.dof() - 1];
    }
    if (H2) {
      // Jacobian for the joint rates
      *H2 = J_jvx_jv[arm_.dof() - 1];
    }
    return pose_error + vel_error;
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const {
    std::cout << s << "PriorWorkspacePoseVelocity :" << std::endl;
    Base::print("", keyFormatter);
    std::cout << "desired end-effector pose : ";
    des_pose_.print();  // << std::endl;
    std::cout << "desired end-effector velocity : " << des_vel_ << std::endl;
  }

 private:
#ifdef GPMP2_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int version) {
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor1", boost::serialization::base_object<Base>(*this));
  }
#endif
};

}  // namespace gpmp2
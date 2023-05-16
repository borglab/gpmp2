/**
 *  @file   PriorWorkspaceVelocity.h
 *  @brief  Prior defined on the workspace velocity of final joint of a robot
 *          given its state in configuration space
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
 * binary factor Gaussian prior defined on the workspace vector
 */
class PriorWorkspaceVelocity : public gtsam::NoiseModelFactorN<gtsam::Vector> {
 private:
  // typedefs
  typedef PriorWorkspaceVelocity This;
  typedef gtsam::NoiseModelFactorN<gtsam::Vector> Base;

  // arm
  Arm arm_;

  // desired velocity (linear and angular)
  gtsam::Vector6 dest_vel_;

 public:
  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;

  /**
   * Constructor
   * @param cost_model cost function covariance
   * @param dest_vel desired velocity (linear and angular)
   */
  PriorWorkspaceVelocity(gtsam::Key poseKey, gtsam::Key velKey,
                         const gtsam::SharedNoiseModel& cost_model,
                         const Arm& arm, const gtsam::Vector6& dest_vel)
      : Base(cost_model, poseKey, velKey), arm_(arm), dest_vel_(dest_vel) {}

  virtual ~PriorWorkspaceVelocity() {}

  /// error function
  gtsam::Vector evaluateError(
      const gtsam::Vector& conf, gtsam::OptionalMatrixType H1 = nullptr,
      gtsam::OptionalMatrixType H2 = nullptr) const override {
    using namespace gtsam;

    // fk
    std::vector<Pose3> joint_pos;
    std::vector<Vector6> joint_vel;
    std::vector<Matrix> J_jpx_jp, J_jvx_jv;
    arm_.forwardKinematics(conf, )

    if (H1) *H1 = Matrix::Zero(conf.size(), conf.size());
    Vector err(conf.size());
    for (size_t i = 0; i < (size_t)conf.size(); i++) {
      if (H1) {
        double Hp;
        err(i) = hingeLossJointLimitCost(conf(i), -vel_limit_(i), vel_limit_(i),
                                         limit_thresh_(i), &Hp);
        (*H1)(i, i) = Hp;
      } else {
        err(i) = hingeLossJointLimitCost(conf(i), -vel_limit_(i), vel_limit_(i),
                                         limit_thresh_(i));
      }
    }
    return err;
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
    std::cout << s << "VelocityLimitFactorVector :" << std::endl;
    Base::print("", keyFormatter);
    std::cout << "Limit cost threshold : " << limit_thresh_ << std::endl;
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

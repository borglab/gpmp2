/**
 *  @file  GoalFactorArm.h
 *  @brief generate error for guild an Arm to reach a 3D point destination
 *  @author Jing Dong
 *  @date  Nov 23, 2015
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
 * unary factor connect to the last pose in arm configuration space
 */
class GoalFactorArm : public gtsam::NoiseModelFactorN<gtsam::Vector> {
 private:
  // typedefs
  typedef GoalFactorArm This;
  typedef gtsam::NoiseModelFactorN<gtsam::Vector> Base;

  // arm
  Arm arm_;

  // destination point
  gtsam::Point3 dest_point_;

 public:
  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;

  /// Default constructor
  GoalFactorArm() {}

  /**
   * Constructor
   * @param cost_model cost function covariance
   */
  GoalFactorArm(gtsam::Key poseKey, const gtsam::SharedNoiseModel& cost_model,
                const Arm& arm, const gtsam::Point3& dest_point)
      : Base(cost_model, poseKey), arm_(arm), dest_point_(dest_point) {}

  ~GoalFactorArm() {}

  /// error function
  gtsam::Vector evaluateError(
      const gtsam::Vector& conf,
      gtsam::OptionalMatrixType H1 = nullptr) const override {
    using namespace gtsam;

    // fk
    std::vector<Pose3> joint_pos;
    std::vector<Matrix> J_jpx_jp;
    arm_.forwardKinematics(conf, {}, joint_pos, {}, &J_jpx_jp);

    if (H1) {
      Matrix36 Hpp;
      Point3 end_point = joint_pos[arm_.dof() - 1].translation(Hpp);
      *H1 = Hpp * J_jpx_jp[arm_.dof() - 1];
      return end_point - dest_point_;

    } else {
      return joint_pos[arm_.dof() - 1].translation() - dest_point_;
    }
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const {
    std::cout << s << "GoalFactorArm :" << std::endl;
    Base::print("", keyFormatter);
    std::cout << "dest : " << dest_point_.transpose() << std::endl;
  }

 private:
#ifdef GPMP2_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int version) {
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor4", boost::serialization::base_object<Base>(*this));
  }
#endif
};

}  // namespace gpmp2

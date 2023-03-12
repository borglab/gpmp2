/**
 *  @file   GaussianPriorWorkspacePosition.h
 *  @brief  Gaussian prior defined on the workspace position of any joint of a
 *robot given its state in configuration space
 *  @author Mustafa Mukadam
 *  @date   Jan 8, 2018
 **/

#pragma once

#include <gpmp2/kinematics/RobotModel.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gpmp2 {

/**
 * Gaussian prior defined on the workspace position
 */
template <class ROBOT>
class GaussianPriorWorkspacePosition
    : public gtsam::NoiseModelFactorN<typename ROBOT::Pose> {
 public:
  // typedefs
  typedef ROBOT Robot;
  typedef typename Robot::Pose Pose;

 private:
  // typedefs
  typedef GaussianPriorWorkspacePosition This;
  typedef gtsam::NoiseModelFactorN<Pose> Base;

  const Robot& robot_;          // Robot
  int joint_;                   // joint on the robot to be constrained
  gtsam::Point3 des_position_;  // desired workspace position for joint

 public:
  /* Default constructor */
  GaussianPriorWorkspacePosition() {}

  /// Constructor
  GaussianPriorWorkspacePosition(gtsam::Key poseKey, const Robot& robot,
                                 int joint, const gtsam::Point3& des_position,
                                 const gtsam::SharedNoiseModel& cost_model)
      : Base(cost_model, poseKey),
        robot_(robot),
        joint_(joint),
        des_position_(des_position) {}

  ~GaussianPriorWorkspacePosition() {}

  /// factor error function
  gtsam::Vector evaluateError(
      const Pose& pose, gtsam::OptionalMatrixType H1 = nullptr) const override {
    using namespace gtsam;

    std::vector<Pose3> joint_pos;
    std::vector<Matrix> J_jpx_jp;
    robot_.fk_model().forwardKinematics(pose, {}, joint_pos, {}, &J_jpx_jp);

    if (H1) {
      Matrix36 Hpp;
      Point3 curr_position = joint_pos[joint_].translation(Hpp);
      *H1 = Hpp * J_jpx_jp[joint_];
      return curr_position - des_position_;
    } else {
      return joint_pos[joint_].translation() - des_position_;
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
    std::cout << s << "GaussianPriorWorkspacePosition :" << std::endl;
    Base::print("", keyFormatter);
    std::cout << "desired position : " << des_position_.transpose()
              << std::endl;
    ;
  }

 private:
#ifdef GPMP2_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int version) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
#endif
};

}  // namespace gpmp2

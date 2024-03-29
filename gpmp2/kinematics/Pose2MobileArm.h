/**
 *  @file  Pose2MobileArm.h
 *  @brief Abstract plannar mobile manipulator, Pose2 + Arm
 *  @author Jing Dong
 *  @date  Oct 4, 2016
 **/

#pragma once

#include <gpmp2/config.h>
#include <gpmp2/geometry/Pose2Vector.h>
#include <gpmp2/kinematics/Arm.h>
#include <gpmp2/kinematics/ForwardKinematics.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>

#include <vector>

namespace gpmp2 {

/**
 * Abstract plannar mobile manipulator, without any physical model
 * representation Inherited from ForwardKinematics
 */
class GPMP2_EXPORT Pose2MobileArm
    : public ForwardKinematics<Pose2Vector, gtsam::Vector> {
 private:
  // typedefs
  typedef ForwardKinematics<Pose2Vector, gtsam::Vector> Base;

  // base to arm pose
  gtsam::Pose3 base_T_arm_;
  // arm class
  Arm arm_;

 public:
  /// default contructor
  Pose2MobileArm() {}

  /// constructor from Arm
  explicit Pose2MobileArm(const Arm& arm,
                          const gtsam::Pose3& base_T_arm = gtsam::Pose3());

  /// Default destructor
  virtual ~Pose2MobileArm() {}

  /**
   *  Forward kinematics: joint configuration to poses in workspace
   *  Velocity kinematics: optional joint velocities to velocities in workspace
   *
   *  @param p position in config space
   *  @param v velocity in config space
   *  @param px link pose in work space
   *  @param vx link velocity in work space
   *  @param J_px_p et al. optional Jacobians
   **/
  void forwardKinematics(const Pose2Vector& p,
                         std::optional<const gtsam::Vector> v,
                         std::vector<gtsam::Pose3>& px,
                         std::vector<gtsam::Vector6>* vx = nullptr,
                         gtsam::OptionalMatrixVecType J_px_p = nullptr,
                         gtsam::OptionalMatrixVecType J_vx_p = nullptr,
                         gtsam::OptionalMatrixVecType J_vx_v = nullptr) const;

  /// accesses
  const gtsam::Pose3& base_T_arm() const { return base_T_arm_; }
  const Arm& arm() const { return arm_; }
};

}  // namespace gpmp2

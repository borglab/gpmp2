/**
 *  @file  Pose2MobileVetLinArm.h
 *  @brief Abstract plannar mobile manipulator, Arm on a vetical linear actuator
 *  @author Jing Dong
 *  @date  Aug 18, 2017
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
 * Abstract plannar mobile manipulator
 * an Arm on Pose2 mobile base with a vetical linear actuator
 * Linear actuator on 1st dim of gtsam::Vector, remaining are Arm's
 */
class GPMP2_EXPORT Pose2MobileVetLinArm
    : public ForwardKinematics<Pose2Vector, gtsam::Vector> {
 private:
  // typedefs
  typedef ForwardKinematics<Pose2Vector, gtsam::Vector> Base;

  // base to arm pose, when linear actuator is on zero
  gtsam::Pose3 base_T_torso_, torso_T_arm_;
  // if reverse_linact_ == true, positive value on lin act means move down
  bool reverse_linact_;
  // arm class
  Arm arm_;

 public:
  /// default contructor
  Pose2MobileVetLinArm() {}

  /// constructor from Arm
  /// if reverse_linact == true, positive value on lin act means move down
  explicit Pose2MobileVetLinArm(
      const Arm& arm, const gtsam::Pose3& base_T_torso = gtsam::Pose3(),
      const gtsam::Pose3& torso_T_arm = gtsam::Pose3(),
      bool reverse_linact = false);

  /// Default destructor
  virtual ~Pose2MobileVetLinArm() {}

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
  const gtsam::Pose3& base_T_torso() const { return base_T_torso_; }
  const gtsam::Pose3& torso_T_arm() const { return torso_T_arm_; }
  const Arm& arm() const { return arm_; }
  bool reverse_linact() const { return reverse_linact_; }
};

}  // namespace gpmp2

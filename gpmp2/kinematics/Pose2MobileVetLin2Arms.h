/**
 *  @file  Pose2MobileVetLin2Arms.h
 *  @brief Plannar mobile manipulator with two arms on vertical linear actuator
 *  @author Jing Dong
 *  @date  Aug 22, 2016
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
 * Abstract plannar mobile manipulator with 2 arm on vertical linear actuator
 * pose class is Pose2Vector, linear actuator use first, arm1 uses next
 * arm1.dof, then arm2 uses last arm2.dof vector DOF = 1 + arm1.dof + arm2.dof
 * total DOF = 3 + 1 + arm1.dof + arm2.dof
 */
class GPMP2_EXPORT Pose2MobileVetLin2Arms
    : public ForwardKinematics<Pose2Vector, gtsam::Vector> {
 private:
  // typedefs
  typedef ForwardKinematics<Pose2Vector, gtsam::Vector> Base;

  // base to arm pose, when linear actuator is on zero
  gtsam::Pose3 base_T_torso_, torso_T_arm1_, torso_T_arm2_;
  // if reverse_linact_ == true, positive value on lin act means move down
  bool reverse_linact_;
  // arm class
  Arm arm1_, arm2_;

 public:
  /// default contructor
  Pose2MobileVetLin2Arms() {}

  /// constructor from Arm
  /// if reverse_linact == true, positive value on lin act means move down
  Pose2MobileVetLin2Arms(const Arm& arm1, const Arm& arm2,
                         const gtsam::Pose3& base_T_torso = gtsam::Pose3(),
                         const gtsam::Pose3& torso_T_arm1 = gtsam::Pose3(),
                         const gtsam::Pose3& torso_T_arm2 = gtsam::Pose3(),
                         bool reverse_linact = false);

  /// Default destructor
  virtual ~Pose2MobileVetLin2Arms() {}

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
  const Arm& arm1() const { return arm1_; }
  const Arm& arm2() const { return arm2_; }
  const gtsam::Pose3& base_T_torso() const { return base_T_torso_; }
  const gtsam::Pose3& torso_T_arm1() const { return torso_T_arm1_; }
  const gtsam::Pose3& torso_T_arm2() const { return torso_T_arm2_; }
  bool reverse_linact() const { return reverse_linact_; }
};

}  // namespace gpmp2

/**
 *  @file  ForwardKinematics.h
 *  @brief Abstract forward kinematics model
 *  @author Jing Dong
 *  @date  May 28, 2015
 **/

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <vector>

namespace gpmp2 {

/// Enum for specifying Denavit-Hartenberg parameterization
enum Parameterization { DH, MODIFIED_DH };

/**
 * Abstract forward kinematics model, without actual model and physical
 * representation template parameters are system pose and velocity state types
 */
template <class POSE, class VELOCITY>
class ForwardKinematics {
 private:
  size_t dof_;       // system degree of freedom
  size_t nr_links_;  // number of links (piece of robot part)

 public:
  /// type defs
  typedef POSE Pose;
  typedef VELOCITY Velocity;

  /// default contructor, for serialization
  ForwardKinematics() {}

  /// Contructor take system DOF and number of links
  /// and the base pose (default zero pose)
  ForwardKinematics(size_t dof, size_t nr_links)
      : dof_(dof), nr_links_(nr_links) {}

  /// Default destructor
  virtual ~ForwardKinematics() {}

  /**
   *  Forward kinematics: poses from configuration space to 3D workspace
   *  Velocity kinematics: optional velocities from configuration space to 3D
   *  workspace, no angular rate pure virtual method, need implementation in
   *  derived class
   *
   *  @param jp robot pose in config space
   *  @param jv robot velocity in config space
   *  @param jpx link poses in 3D work space
   *  @param jvx link velocities in 3D work space, no angular rate
   *  @param J_jpx_jp et al. optional Jacobians
   **/
  virtual void forwardKinematics(
      const Pose& jp, std::optional<const Velocity> jv,
      std::vector<gtsam::Pose3>& jpx,
      std::vector<gtsam::Vector3>* jvx = nullptr,
      gtsam::OptionalMatrixVecType J_jpx_jp = nullptr,
      gtsam::OptionalMatrixVecType J_jvx_jp = nullptr,
      gtsam::OptionalMatrixVecType J_jvx_jv = nullptr) const = 0;

  /**
   * Matrix wrapper for forwardKinematics, mainly used by matlab
   * each column is a single point / velocity of the joint, size 6xN, 3xN, 3xN
   * No Jacobians provided by this version
   */
  gtsam::Matrix forwardKinematicsPose(const Pose& jp) const;
  gtsam::Matrix forwardKinematicsPosition(const Pose& jp) const;
  gtsam::Matrix forwardKinematicsVel(const Pose& jp, const Velocity& jv) const;

  /// accesses
  size_t dof() const { return dof_; }
  size_t nr_links() const { return nr_links_; }
};

}  // namespace gpmp2

#include <gpmp2/kinematics/ForwardKinematics-inl.h>

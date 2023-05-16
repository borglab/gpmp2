/**
 *  @file   PointRobot.h
 *  @brief  Abstract Point Robot
 *  @author Mustafa Mukadam
 *  @date   July 20, 2016
 **/

#pragma once

#include <gpmp2/config.h>
#include <gpmp2/kinematics/ForwardKinematics.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

#include <cmath>
#include <vector>

namespace gpmp2 {

/**
 * Abstract PointRobot class without any physical model representation
 * Inherited from ForwardKinematics
 */
class GPMP2_EXPORT PointRobot
    : public ForwardKinematics<gtsam::Vector, gtsam::Vector> {
 private:
  // typedefs
  typedef ForwardKinematics<gtsam::Vector, gtsam::Vector> Base;

 public:
  /// default constructor
  PointRobot() {}

  /// Constructor takes in DOF
  PointRobot(size_t dof, size_t nr_links) : Base(dof, nr_links) {}

  /// Default destructor
  virtual ~PointRobot() {}

  /**
   *  Forward kinematics: robot configuration to poses in workspace
   *  Velocity kinematics: optional robot velocities to linear velocities in
   *workspace, no angular rate
   *
   *  @param jp   robot pose in config space
   *  @param jv   robot velocity in config space
   *  @param jpx  robot pose in work space
   *  @param jvx  robot velocity in work space
   *  @param J_jpx_jp et al. optional Jacobians
   **/
  void forwardKinematics(const gtsam::Vector& jp,
                         std::optional<const gtsam::Vector> jv,
                         std::vector<gtsam::Pose3>& jpx,
                         std::vector<gtsam::Vector6>* jvx = nullptr,
                         gtsam::OptionalMatrixVecType J_jpx_jp = nullptr,
                         gtsam::OptionalMatrixVecType J_jvx_jp = nullptr,
                         gtsam::OptionalMatrixVecType J_jvx_jv = nullptr) const;

};  // PointRobot

}  // namespace gpmp2

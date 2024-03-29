/**
 *  @file   Pose2MobileBase.h
 *  @brief  Abstract SE(2) mobile base
 *  @author Mustafa Mukadam
 *  @date   Jan 22, 2018
 **/

#pragma once

#include <gpmp2/config.h>
#include <gpmp2/kinematics/ForwardKinematics.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

#include <vector>

namespace gpmp2 {

/**
 * Abstract SE(2) mobile base inherited from ForwardKinematics
 */
class GPMP2_EXPORT Pose2MobileBase
    : public ForwardKinematics<gtsam::Pose2, gtsam::Vector> {
 private:
  // typedefs
  typedef ForwardKinematics<gtsam::Pose2, gtsam::Vector> Base;

 public:
  /// default constructor
  explicit Pose2MobileBase() : Base(3, 1) {}

  /// Default destructor
  virtual ~Pose2MobileBase() {}

  /**
   *  Forward kinematics: joint configuration to poses in workspace
   *  Velocity kinematics: optional joint velocities to velocities workspace
   *
   *  @param p position in config space
   *  @param v velocity in config space
   *  @param px link pose in work space
   *  @param vx link velocity in work space
   *  @param J_px_p et al. optional Jacobians
   **/
  void forwardKinematics(const gtsam::Pose2& p,
                         std::optional<const gtsam::Vector> v,
                         std::vector<gtsam::Pose3>& px,
                         std::vector<gtsam::Vector6>* vx = nullptr,
                         gtsam::OptionalMatrixVecType J_px_p = nullptr,
                         gtsam::OptionalMatrixVecType J_vx_p = nullptr,
                         gtsam::OptionalMatrixVecType J_vx_v = nullptr) const;
};

}  // namespace gpmp2

/**
 *  @file   VehicleDynamicsFactorPose2Vector.h
 *  @brief  simple 2D vehicle dynamics factor for mobile arm base in Lie group
 *SE(2)
 *  @author Jing Dong, Mustafa Mukadam
 *  @date   Oct 14, 2016
 **/

#pragma once

#include <gpmp2/dynamics/VehicleDynamics.h>
#include <gpmp2/geometry/Pose2Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <iostream>
#include <vector>

namespace gpmp2 {

/**
 * unary factor for vehicle dynamics
 */
class VehicleDynamicsFactorPose2Vector
    : public gtsam::NoiseModelFactorN<Pose2Vector, gtsam::Vector> {
 private:
  // typedefs
  typedef VehicleDynamicsFactorPose2Vector This;
  typedef gtsam::NoiseModelFactorN<Pose2Vector, gtsam::Vector> Base;

 public:
  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;

  /* Default constructor */
  VehicleDynamicsFactorPose2Vector() {}

  /**
   * Constructor
   * @param cost_sigma cost function covariance, should to identity model
   */
  VehicleDynamicsFactorPose2Vector(gtsam::Key poseKey, gtsam::Key velKey,
                                   double cost_sigma)
      : Base(gtsam::noiseModel::Isotropic::Sigma(1, cost_sigma), poseKey,
             velKey) {}

  virtual ~VehicleDynamicsFactorPose2Vector() {}

  /// error function
  /// numerical/analytic Jacobians from cost function
  gtsam::Vector evaluateError(
      const Pose2Vector& conf, const gtsam::Vector& vel,
      gtsam::OptionalMatrixType H1 = nullptr,
      gtsam::OptionalMatrixType H2 = nullptr) const override {
    using namespace gtsam;

    if (H1 || H2) {
      Matrix13 Hp, Hv;
      const double err =
          simple2DVehicleDynamicsPose2(conf.pose(), vel.head<3>(), Hp, Hv);
      if (H1) {
        *H1 = Matrix::Zero(1, conf.dim());
        H1->block<1, 3>(0, 0) = Hp;
      }
      if (H2) {
        *H2 = Matrix::Zero(1, conf.dim());
        H2->block<1, 3>(0, 0) = Hv;
      }
      return (Vector(1) << err).finished();

    } else {
      return (Vector(1) << simple2DVehicleDynamicsPose2(conf.pose(),
                                                        vel.head<3>()))
          .finished();
    }
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
    std::cout << s << "VehicleDynamicsFactorPose2Vector :" << std::endl;
    Base::print("", keyFormatter);
  }

  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int version) {
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor2", boost::serialization::base_object<Base>(*this));
  }
};

}  // namespace gpmp2

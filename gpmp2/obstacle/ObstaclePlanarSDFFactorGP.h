/**
 *  @file  ObstaclePlanarSDFFactorGP.h
 *  @brief Obstacle avoidance cost factor, using signed distance field and GP
 *interpolation
 *  @author Jing Dong
 *  @date  Nov 22, 2015
 **/

#pragma once

#include <gpmp2/obstacle/PlanarSDF.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <iostream>

namespace gpmp2 {

/**
 * binary factor for obstacle avoidance use GP interpolation, planar version
 * template robot model and GP interpolator version
 */
template <class ROBOT, class GPINTER>
class ObstaclePlanarSDFFactorGP
    : public gtsam::NoiseModelFactorN<
          typename ROBOT::Pose, typename ROBOT::Velocity, typename ROBOT::Pose,
          typename ROBOT::Velocity> {
 public:
  // typedefs
  typedef ROBOT Robot;
  typedef typename Robot::Pose Pose;
  typedef typename Robot::Velocity Velocity;

 private:
  // typedefs
  typedef ObstaclePlanarSDFFactorGP This;
  typedef gtsam::NoiseModelFactorN<Pose, Velocity, Pose, Velocity> Base;
  typedef GPINTER GPBase;

  // GP interpolator
  GPBase GPbase_;

  // obstacle settings
  double epsilon_;  // distance from object that start non-zero cost

  // arm: planar one, all alpha = 0
  const Robot& robot_;

  // signed distance field from matlab
  const PlanarSDF& sdf_;

 public:
  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;

  /* Default constructor */
  ObstaclePlanarSDFFactorGP() {}

  /**
   * Constructor
   * @param cost_model cost function covariance, should to identity model
   * @param Qc_model   dim is equal to DOF
   * @param field      signed distance field
   * @param nn_index   nearest neighbour index of signed distance field
   * @param check_inter  how many points needed to be interpolated. 0 means no
   * GP interpolation
   */
  ObstaclePlanarSDFFactorGP(gtsam::Key pose1Key, gtsam::Key vel1Key,
                            gtsam::Key pose2Key, gtsam::Key vel2Key,
                            const Robot& robot, const PlanarSDF& sdf,
                            double cost_sigma, double epsilon,
                            const gtsam::SharedNoiseModel& Qc_model,
                            double delta_t, double tau)
      :

        Base(gtsam::noiseModel::Isotropic::Sigma(robot.nr_body_spheres(),
                                                 cost_sigma),
             pose1Key, vel1Key, pose2Key, vel2Key),
        GPbase_(Qc_model, delta_t, tau),
        epsilon_(epsilon),
        robot_(robot),
        sdf_(sdf) {
    // TODO: check arm is plannar
  }

  ~ObstaclePlanarSDFFactorGP() {}

  /// error function
  /// numerical jacobians / analytic jacobians from cost function
  gtsam::Vector evaluateError(
      const Pose& conf1, const Velocity& vel1, const Pose& conf2,
      const Velocity& vel2, gtsam::OptionalMatrixType H1 = nullptr,
      gtsam::OptionalMatrixType H2 = nullptr,
      gtsam::OptionalMatrixType H3 = nullptr,
      gtsam::OptionalMatrixType H4 = nullptr) const override;

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const {
    std::cout << s << "ObstaclePlanarSDFFactorGP :" << std::endl;
    Base::print("", keyFormatter);
  }

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

#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGP-inl.h>

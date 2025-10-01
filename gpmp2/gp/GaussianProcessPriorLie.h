/**
 *  @file  GaussianProcessPriorLie.h
 *  @brief GP prior works on any Lie group
 *  @author Jing Dong
 *  @date Oct 3, 2016
 **/

#pragma once

#include <gpmp2/gp/GPutils.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/concepts.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <ostream>

namespace gpmp2 {

/**
 * 4-way factor for Gaussian Process prior factor on any Lie group
 */
template <typename T>
class GaussianProcessPriorLie
    : public gtsam::NoiseModelFactorN<T, gtsam::Vector, T, gtsam::Vector> {
 private:
  GTSAM_CONCEPT_ASSERT(gtsam::IsLieGroup<T>);
  typedef GaussianProcessPriorLie<T> This;
  typedef gtsam::NoiseModelFactorN<T, gtsam::Vector, T, gtsam::Vector> Base;

  size_t dof_;
  double delta_t_;

 public:
  GaussianProcessPriorLie() {} /* Default constructor only for serialization */

  /// Constructor
  /// @param delta_t is the time between the two states
  GaussianProcessPriorLie(gtsam::Key poseKey1, gtsam::Key velKey1,
                          gtsam::Key poseKey2, gtsam::Key velKey2,
                          double delta_t,
                          const gtsam::SharedNoiseModel& Qc_model)
      : Base(gtsam::noiseModel::Gaussian::Covariance(
                 calcQ(getQc(Qc_model), delta_t)),
             poseKey1, velKey1, poseKey2, velKey2),
        dof_(Qc_model->dim()),
        delta_t_(delta_t) {}

  ~GaussianProcessPriorLie() {}

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// factor error function
  gtsam::Vector evaluateError(
      const T& pose1, const gtsam::Vector& vel1, const T& pose2,
      const gtsam::Vector& vel2, gtsam::OptionalMatrixType H1 = nullptr,
      gtsam::OptionalMatrixType H2 = nullptr,
      gtsam::OptionalMatrixType H3 = nullptr,
      gtsam::OptionalMatrixType H4 = nullptr) const override {
    using namespace gtsam;

    Matrix Hinv, Hcomp1, Hcomp2, Hlogmap;
    Vector r;
    if (H1 || H2 || H3 || H4)
      r = traits<T>::Logmap(traits<T>::Compose(traits<T>::Inverse(pose1, Hinv),
                                               pose2, Hcomp1, Hcomp2),
                            Hlogmap);
    else
      r = traits<T>::Logmap(
          traits<T>::Compose(traits<T>::Inverse(pose1, Hinv), pose2));

    // jacobians
    if (H1)
      *H1 = (Matrix(2 * dof_, dof_) << Hlogmap * Hcomp1 * Hinv,
             Matrix::Zero(dof_, dof_))
                .finished();
    if (H2)
      *H2 = (Matrix(2 * dof_, dof_) << -delta_t_ * Matrix::Identity(dof_, dof_),
             -Matrix::Identity(dof_, dof_))
                .finished();
    if (H3)
      *H3 =
          (Matrix(2 * dof_, dof_) << Hlogmap * Hcomp2, Matrix::Zero(dof_, dof_))
              .finished();
    if (H4)
      *H4 = (Matrix(2 * dof_, dof_) << Matrix::Zero(dof_, dof_),
             Matrix::Identity(dof_, dof_))
                .finished();

    return (Vector(2 * dof_) << (r - vel1 * delta_t_), (vel2 - vel1))
        .finished();
  }

  /** number of variables attached to this factor */
  size_t size() const { return 4; }

  /** equals specialized to this factor */
  bool equals(const gtsam::NonlinearFactor& expected,
              double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&expected);
    return e != NULL && Base::equals(*e, tol) &&
           fabs(this->delta_t_ - e->delta_t_) < tol;
  }

  /** print contents */
  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const {
    std::cout << s << "4-way Gaussian Process Facto on Lie<" << dof_ << ">"
              << std::endl;
    Base::print("", keyFormatter);
  }

 private:
#ifdef GPMP2_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int version) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(dof_);
    ar& BOOST_SERIALIZATION_NVP(delta_t_);
  }
#endif

};  // GaussianProcessPriorLie

}  // namespace gpmp2

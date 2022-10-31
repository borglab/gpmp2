/**
 *  @file  GaussianProcessPriorLieLTI.h
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

#include <boost/lexical_cast.hpp>
#include <boost/serialization/export.hpp>
#include <ostream>

namespace gpmp2 {

/**
 * 5-way factor for Gaussian Process prior factor on any Lie group with control
 */
template <typename T>
class GaussianProcessPriorLieLTI
    : public gtsam::NoiseModelFactor5<T, gtsam::Vector, T, gtsam::Vector,
                                      gtsam::Vector> {
private:
  BOOST_CONCEPT_ASSERT((gtsam::IsLieGroup<T>));
  typedef GaussianProcessPriorLieLTI<T> This;
  typedef gtsam::NoiseModelFactor5<T, gtsam::Vector, T, gtsam::Vector,
                                   gtsam::Vector>
      Base;

  size_t dof_;
  double delta_t_;

public:
  GaussianProcessPriorLieLTI() {
  } /* Default constructor only for serialization */

  /// Constructor
  /// @param delta_t is the time between the two states
  GaussianProcessPriorLieLTI(gtsam::Key poseKey1, gtsam::Key velKey1,
                             gtsam::Key poseKey2, gtsam::Key velKey2,
                             gtsam::Key controlKey, double delta_t,
                             const gtsam::SharedNoiseModel &Qc_model)
      : Base(gtsam::noiseModel::Gaussian::Covariance(
                 calcQ(getQc(Qc_model), delta_t)),
             poseKey1, velKey1, poseKey2, velKey2, controlKey),
        dof_(Qc_model->dim()), delta_t_(delta_t) {}

  virtual ~GaussianProcessPriorLieLTI() {}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// factor error function
  gtsam::Vector
  evaluateError(const T &pose1, const gtsam::Vector &vel1, const T &pose2,
                const gtsam::Vector &vel2, const gtsam::Vector &control,
                boost::optional<gtsam::Matrix &> H1 = boost::none,
                boost::optional<gtsam::Matrix &> H2 = boost::none,
                boost::optional<gtsam::Matrix &> H3 = boost::none,
                boost::optional<gtsam::Matrix &> H4 = boost::none,
                boost::optional<gtsam::Matrix &> H5 = boost::none) const {
    using namespace gtsam;

    // Control matrix
    gtsam::Matrix B =
        (gtsam::Matrix(2 * dof_, dof_) << gtsam::Matrix::Zero(dof_, dof_),
         gtsam::Matrix::Identity(dof_, dof_))
            .finished();

    // ASTROS mass and inertia about the z-axis
    float m = 410.0, Izz = 28.5;

    // Matrix gain values for ASTROS (need to pass in variable in the future)
    B(3, 0) = 1.0 / m;
    B(4, 1) = 1.0 / m;
    B(5, 2) = 1.0 / Izz;

    // Matrix A
    gtsam::Matrix A =
        (gtsam::Matrix(2 * dof_, 2 * dof_) << gtsam::Matrix::Zero(dof_, dof_),
         gtsam::Matrix::Identity(dof_, dof_), gtsam::Matrix::Zero(dof_, dof_),
         gtsam::Matrix::Zero(dof_, dof_))
            .finished();

    gtsam::Matrix J =
        (gtsam::Matrix(2 * dof_, dof_) << gtsam::Matrix::Identity(dof_, dof_),
         gtsam::Matrix::Identity(dof_, dof_))
            .finished();
    J(0, 0) = (delta_t_*delta_t_) / (2.0 * m);
    J(1, 1) = (delta_t_*delta_t_) / (2.0 * m);
    J(2, 2) = (delta_t_*delta_t_) / (2.0 * Izz);
    J(3, 0) = delta_t_ / m;
    J(4, 1) = delta_t_ / m;
    J(5, 2) = delta_t_ / Izz;

    Matrix Hinv, Hcomp1, Hcomp2, Hlogmap;
    Vector r;
    if (H1 || H2 || H3 || H4 || H5)
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
    if (H5)
      *H5 = -J;

    gtsam::Vector x2_m_x1 =
        (gtsam::Vector(2 * dof_) << (r - vel1 * delta_t_), (vel2 - vel1))
            .finished();


    return x2_m_x1 - calcPhi(dof_, delta_t_) *
                         (gtsam::Matrix::Identity(2 * dof_, 2 * dof_) -
                          0.5 * delta_t_ * A) *
                         B *delta_t_ *control;
  }

  /** number of variables attached to this factor */
  size_t size() const { return 5; }

  /** equals specialized to this factor */
  virtual bool equals(const gtsam::NonlinearFactor &expected,
                      double tol = 1e-9) const {
    const This *e = dynamic_cast<const This *>(&expected);
    return e != NULL && Base::equals(*e, tol) &&
           fabs(this->delta_t_ - e->delta_t_) < tol;
  }

  /** print contents */
  void print(const std::string &s = "",
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const {
    std::cout << s << "5-way Gaussian Process Factor on Lie with control<" << dof_ << ">"
              << std::endl;
    Base::print("", keyFormatter);
  }

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int version) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(dof_);
    ar &BOOST_SERIALIZATION_NVP(delta_t_);
  }

}; // GaussianProcessPriorLie

} // namespace gpmp2

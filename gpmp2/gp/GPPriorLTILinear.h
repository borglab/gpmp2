/**
 *  @file  GPPriorLTILinear.h
 *  @brief Linear Time Invariant GP prior with Control
 *  @author Matthew King-Smith
 **/

#pragma once

#include <boost/lexical_cast.hpp>
#include <gpmp2/gp/GPutils.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/concepts.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gpmp2 {

/*
 * 5-way factor for Gaussian Process prior factor, linear version
 */
class GPPriorLTILinear
    : public gtsam::NoiseModelFactor5<gtsam::Vector, gtsam::Vector, gtsam::Vector,
                                      gtsam::Vector, gtsam::Vector> {
private:
  size_t dof_;
  double delta_t_;

  typedef GPPriorLTILinear This;
  typedef gtsam::NoiseModelFactor5<gtsam::Vector, gtsam::Vector, gtsam::Vector,
                                   gtsam::Vector, gtsam::Vector>
      Base;

public:
  GPPriorLTILinear() {} /* Default constructor only for serialization */

  /// Constructor
  /// @param delta_t is the time between the two states
  GPPriorLTILinear(gtsam::Key poseKey1, gtsam::Key velKey1, gtsam::Key poseKey2,
             gtsam::Key velKey2, gtsam::Key conKey, double delta_t,
             const gtsam::SharedNoiseModel Qc_model)
      : Base(gtsam::noiseModel::Gaussian::Covariance(
                 calcQ(getQc(Qc_model), delta_t)),
             poseKey1, velKey1, poseKey2, velKey2, conKey),
        dof_(Qc_model->dim()), delta_t_(delta_t) {}

  virtual ~GPPriorLTILinear() {}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// factor error function
  gtsam::Vector
  evaluateError(const gtsam::Vector &pose1, const gtsam::Vector &vel1,
                const gtsam::Vector &pose2, const gtsam::Vector &vel2,
                const gtsam::Vector &con,
                boost::optional<gtsam::Matrix &> H1 = boost::none,
                boost::optional<gtsam::Matrix &> H2 = boost::none,
                boost::optional<gtsam::Matrix &> H3 = boost::none,
                boost::optional<gtsam::Matrix &> H4 = boost::none,
                boost::optional<gtsam::Matrix &> H5 = boost::none) const {

    // state vector
    gtsam::Vector x1 = (gtsam::Vector(2 * dof_) << pose1, vel1).finished();
    gtsam::Vector x2 = (gtsam::Vector(2 * dof_) << pose2, vel2).finished();

    // Control matrix
    gtsam::Matrix B =
        (gtsam::Matrix(2 * dof_, dof_) << gtsam::Matrix::Zero(dof_, dof_), //
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
         gtsam::Matrix::Identity(dof_, dof_), //
         gtsam::Matrix::Zero(dof_, dof_), gtsam::Matrix::Zero(dof_, dof_))
            .finished();

    // Control Jacobian
    gtsam::Matrix J = (gtsam::Matrix(2 * dof_, dof_)
                           << gtsam::Matrix::Identity(dof_, dof_), //
                       gtsam::Matrix::Identity(dof_, dof_))
                          .finished();
    J(0, 0) = (delta_t_ * delta_t_) / (2.0 * m);
    J(1, 1) = (delta_t_ * delta_t_) / (2.0 * m);
    J(2, 2) = (delta_t_ * delta_t_) / (2.0 * Izz);
    J(3, 0) = delta_t_ / m;
    J(4, 1) = delta_t_ / m;
    J(5, 2) = delta_t_ / Izz;

    // Jacobians
    if (H1)
      *H1 =
          (gtsam::Matrix(2 * dof_, dof_) << gtsam::Matrix::Identity(dof_, dof_),
           gtsam::Matrix::Zero(dof_, dof_))
              .finished();
    if (H2)
      *H2 = (gtsam::Matrix(2 * dof_, dof_)
                 << delta_t_ * gtsam::Matrix::Identity(dof_, dof_),
             gtsam::Matrix::Identity(dof_, dof_))
                .finished();
    if (H3)
      *H3 = (gtsam::Matrix(2 * dof_, dof_)
                 << -1.0 * gtsam::Matrix::Identity(dof_, dof_),
             gtsam::Matrix::Zero(dof_, dof_))
                .finished();
    if (H4)
      *H4 = (gtsam::Matrix(2 * dof_, dof_) << gtsam::Matrix::Zero(dof_, dof_),
             -1.0 * gtsam::Matrix::Identity(dof_, dof_))
                .finished();

    if (H5)
      *H5 = J;
    // transition matrix & error
    return calcPhi(dof_, delta_t_) * x1 - x2 +
           calcPhi(dof_, delta_t_) *
               (gtsam::Matrix::Identity(2 * dof_, 2 * dof_) -
                0.5 * delta_t_ * A) *
               B * delta_t_ * con;
  }

  /** dimensions */
  size_t dim() const { return dof_; }

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
    std::cout << s << "5-way Gaussian Process Factor Linear(" << dof_ << ")"
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
};

} // namespace gpmp2

/// traits
namespace gtsam {
template <>
struct traits<gpmp2::GPPriorLTILinear> : public Testable<gpmp2::GPPriorLTILinear> {};
} // namespace gtsam

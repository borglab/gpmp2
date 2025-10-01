/**
 * @file DynamicLieTraits.h
 * @date Oct 4, 2016
 * @author Frank Dellaert
 * @author Mike Bosse
 * @author Jing Dong
 * @brief gtsam Lie group traits for dynamic size types
 */

#pragma once

#include <gtsam/base/Lie.h>

namespace gtsam {
namespace internal {

/// A helper class that implements the traits interface for GTSAM lie groups.
/// To use this for your gtsam type, define:
/// template<> struct traits<Class> : public internal::LieGroupTraits<Class> {};
/// Assumes existence of: identity, dimension, localCoordinates, retract,
/// and additionally Logmap, Expmap, compose, between, and inverse
template <class Class>
struct DynamicLieGroupTraits {
  typedef lie_group_tag structure_category;

  /// @name Group
  /// @{
  typedef multiplicative_group_tag group_flavor;
  static Class Identity() { return Class::Identity(); }
  /// @}

  /// @name Manifold
  /// @{
  typedef Class ManifoldType;
  enum { dimension = Class::dimension };
  typedef Eigen::Matrix<double, dimension, 1> TangentVector;
  typedef OptionalJacobian<dimension, dimension> ChartJacobian;

  static int GetDimension(const Class& m) { return m.dim(); }

  static TangentVector Local(const Class& origin, const Class& other,
                             ChartJacobian Horigin = {},
                             ChartJacobian Hother = {}) {
    return origin.localCoordinates(other, Horigin, Hother);
  }

  static Class Retract(const Class& origin, const TangentVector& v,
                       ChartJacobian Horigin = {}, ChartJacobian Hv = {}) {
    return origin.retract(v, Horigin, Hv);
  }
  /// @}

  /// @name Lie Group
  /// @{
  static TangentVector Logmap(const Class& m, ChartJacobian Hm = {}) {
    return Class::Logmap(m, Hm);
  }

  static Class Expmap(const TangentVector& v, ChartJacobian Hv = {}) {
    return Class::Expmap(v, Hv);
  }

  static Class Compose(const Class& m1, const Class& m2,  //
                       ChartJacobian H1 = {}, ChartJacobian H2 = {}) {
    return m1.compose(m2, H1, H2);
  }

  static Class Between(const Class& m1, const Class& m2,  //
                       ChartJacobian H1 = {}, ChartJacobian H2 = {}) {
    return m1.between(m2, H1, H2);
  }

  static Class Inverse(const Class& m,  //
                       ChartJacobian H = {}) {
    return m.inverse(H);
  }

  static Eigen::MatrixXd AdjointMap(const Class& m) { return m.AdjointMap(); }
  /// @}
};

/// Both LieGroupTraits and Testable
template <class Class>
struct DynamicLieGroup : DynamicLieGroupTraits<Class>, Testable<Class> {};

}  // namespace internal
}  // namespace gtsam

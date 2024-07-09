#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/query_results/deformable_contact.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"

namespace drake {
namespace multibody {
namespace internal {

/* This contains the geometric contact information coming out of SceneGraph, as
consumed by MultibodyPlant. Depending on MbP's contact model and the proximity
properties in the scene graph, one or the other vector might be guaranteed to be
empty; e.g., even in kPoint only mode we still have a field named `surfaces` but
it's always empty.

When T != double, the class is specialized to omit member data that is
incompatible with such scalars.

@tparam_default_scalar */
template <typename T>
struct GeometryContactData {
  std::vector<geometry::PenetrationAsPointPair<T>> point_pairs;
  std::vector<geometry::ContactSurface<T>> surfaces;
  geometry::internal::DeformableContact<T> deformable;
};

/* Full specialization of HydroelasticContactInfo for T = AutoDiffXd.
This omits DeformableContact data. */
template <>
struct GeometryContactData<AutoDiffXd> {
  using T = AutoDiffXd;
  std::vector<geometry::PenetrationAsPointPair<T>> point_pairs;
  std::vector<geometry::ContactSurface<T>> surfaces;
};

/* Full specialization of HydroelasticContactInfo for T = Expression.
This omits ContactSurface and DeformableContact data. */
template <>
struct GeometryContactData<symbolic::Expression> {
  using T = symbolic::Expression;
  std::vector<geometry::PenetrationAsPointPair<T>> point_pairs;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    struct ::drake::multibody::internal::GeometryContactData);

#include "drake/geometry/optimization/convex_set.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperrectangle.h"
#include "drake/geometry/optimization/point.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace {

using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using test::LimitMalloc;

// N.B. See also convex_set_solving_test for additional unit test cases.

GTEST_TEST(ConvexSetsTest, BasicTest) {
  ConvexSets sets;

  const ConvexSet& a = *sets.emplace_back(Point(Vector2d{1., 2.}));
  const Vector3d b_point{3., 4., 5.};
  std::unique_ptr<Point> b_original = std::make_unique<Point>(b_point);
  Point* b_pointer = b_original.get();
  const ConvexSet& b = *sets.emplace_back(std::move(b_original));

  EXPECT_EQ(a.ambient_dimension(), 2);
  EXPECT_EQ(b.ambient_dimension(), 3);

  EXPECT_EQ(sets.size(), 2);
  EXPECT_EQ(sets[0]->ambient_dimension(), 2);
  EXPECT_EQ(sets[1]->ambient_dimension(), 3);

  // Confirm that a const reference to the container provides only const access
  // to the set.
  const ConvexSets& const_sets = sets;
  static_assert(std::is_same_v<const ConvexSet&, decltype(*const_sets[0])>);

  // Confirm that I can move sets without copying the underlying data.
  // Note: jwnimmer-tri argued that this should not be a strong requirement.
  // Derived ConvexSets with substantial memory footprint could implement
  // Clone() using a shared_ptr on their data.  It may be fine to remove this if
  // a different pattern requires it.
  ConvexSets moved = std::move(sets);
  EXPECT_EQ(moved.size(), 2);
  EXPECT_EQ(moved[0]->ambient_dimension(), 2);
  EXPECT_EQ(moved[1]->ambient_dimension(), 3);
  EXPECT_TRUE(moved[1]->PointInSet(b_point));
  const Vector3d new_point{6., 7., 8.};
  EXPECT_FALSE(moved[1]->PointInSet(new_point));
  b_pointer->set_x(new_point);
  EXPECT_TRUE(moved[1]->PointInSet(new_point));
}

GTEST_TEST(MakeConvexSetsTest, Basic) {
  HPolyhedron box = HPolyhedron::MakeUnitBox(2);
  ConvexSets sets =
      MakeConvexSets(box, box.Clone(), Point(Vector3d(1.0, 2.0, 3.0)));

  EXPECT_EQ(sets.size(), 3);
  EXPECT_EQ(sets[0]->ambient_dimension(), 2);
  EXPECT_EQ(sets[1]->ambient_dimension(), 2);
  EXPECT_EQ(sets[2]->ambient_dimension(), 3);
}

// A mutable lvalue reference is copied, not moved.
GTEST_TEST(MakeConvexSetsTest, MutableLvalueReference) {
  const HPolyhedron box = HPolyhedron::MakeUnitBox(2);
  std::unique_ptr<ConvexSet> box_clone = box.Clone();
  ConvexSets sets = MakeConvexSets(box_clone);
  EXPECT_EQ(sets.size(), 1);
  EXPECT_NE(box_clone.get(), nullptr);
}

// The amount of copying is as small as possible.
GTEST_TEST(MakeConvexSetsTest, NoExtraCopying) {
  const HPolyhedron box = HPolyhedron::MakeUnitBox(2);

  // A `unique_ptr<ConvexSet>` is moved into place, no copies.
  // The only allocation is the std::vector storage itself.
  {
    std::unique_ptr<ConvexSet> box1{box.Clone()};
    std::unique_ptr<ConvexSet> box2{box.Clone()};
    LimitMalloc guard({.max_num_allocations = 1});
    MakeConvexSets(std::move(box1), std::move(box2));
  }

  // A `copyable_unique_ptr<ConvexSet>` is moved into place, no copies.
  {
    copyable_unique_ptr<ConvexSet> box1{box.Clone()};
    copyable_unique_ptr<ConvexSet> box2{box.Clone()};
    LimitMalloc guard({.max_num_allocations = 1});
    MakeConvexSets(std::move(box1), std::move(box2));
  }

  // A `const ConvexSet&` is copied just once.
  {
    const int box_clone_num_allocs = 3;  // HPolyhedron, A_ , b_.
    const int num = 1 + box_clone_num_allocs;
    LimitMalloc guard({.max_num_allocations = num, .min_num_allocations = num});
    MakeConvexSets(box);
  }
}

// Minimum implementation of a ConvexSet.
class DummyVolumeSet : public ConvexSet {
 public:
  explicit DummyVolumeSet(bool can_calc_volume)
      : ConvexSet(0, can_calc_volume) {}

 protected:
  std::unique_ptr<ConvexSet> DoClone() const override { return nullptr; }
  bool DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>&,
                    double) const override {
    return false;
  }
  std::pair<VectorX<symbolic::Variable>,
            std::vector<solvers::Binding<solvers::Constraint>>>
  DoAddPointInSetConstraints(
      solvers::MathematicalProgram*,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>&)
      const override {
    return {{}, {}};
  }
  std::vector<solvers::Binding<solvers::Constraint>>
  DoAddPointInNonnegativeScalingConstraints(
      solvers::MathematicalProgram*,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>&,
      const symbolic::Variable&) const override {
    return {};
  }
  std::vector<solvers::Binding<solvers::Constraint>>
  DoAddPointInNonnegativeScalingConstraints(
      solvers::MathematicalProgram*, const Eigen::Ref<const Eigen::MatrixXd>&,
      const Eigen::Ref<const Eigen::VectorXd>&,
      const Eigen::Ref<const Eigen::VectorXd>&, double,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>&,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>&)
      const override {
    return {};
  }
  std::pair<std::unique_ptr<Shape>, math::RigidTransformd> DoToShapeWithPose()
      const override {
    return {nullptr, {}};
  }
};

// A convex set that doesn't implement DoCalcVolume but can erroneously report
// that it can compute an exact volume.
class NoImplSet final : public DummyVolumeSet {
 public:
  explicit NoImplSet(bool can_calc_volume) : DummyVolumeSet(can_calc_volume) {}
};

// A convex set that has implemented DoCalcVolume(), but can arbitrarily
// indicate whether it has an exact volume. The value returned by DoCalcVolume()
// depends on whether the constructor's `can_calc_volume` is true or false.
// If true, `DoCalcVolume()` returns a positive value, if `false`, a negative
// value. We should never get a negative value because CalcVolume() should throw
// base on `has_exact_value()`.
class HasImplSet final : public DummyVolumeSet {
 public:
  explicit HasImplSet(bool can_calc_volume)
      : DummyVolumeSet(can_calc_volume), can_calc_volume_(can_calc_volume) {}

 private:
  double DoCalcVolume() const final { return can_calc_volume_ ? 1.5 : -1; }
  bool can_calc_volume_{};
};

// Confirms that CalcVolume() respects has_exact_volume() and that errors in
// derived classes are detected and reported.
GTEST_TEST(ConvexSetTest, CalcVolume) {
  // CalcVolume() correctly avoids calling DoCalcVolume().
  DRAKE_EXPECT_THROWS_MESSAGE(
      NoImplSet(false).CalcVolume(),
      ".*NoImplSet reports that it cannot report an exact volume.*");

  // CalcVolume() calls DoCalcVolume(), revealing the class has lied.
  DRAKE_EXPECT_THROWS_MESSAGE(
      NoImplSet(true).CalcVolume(),
      ".*NoImplSet has a defect -- has_exact_volume.. is reporting true.*");

  // CalcVolume() correctly avoids calling the implemented DoCalcVolume().
  DRAKE_EXPECT_THROWS_MESSAGE(
      HasImplSet(false).CalcVolume(),
      ".*HasImplSet reports that it cannot report an exact volume.*");

  // CalcVolume() called DoCalcVolume() correctly, and it returned a positive
  // value.
  EXPECT_GT(HasImplSet(true).CalcVolume(), 0);
}

// Tests the computation of the minimal axis-aligned bounding box of a
// polyhedron.
GTEST_TEST(ConvexSetTest, AxisAlignedBoundingBox) {
  // Case: Unbounded.
  {
    Matrix<double, 4, 2> A;
    Matrix<double, 4, 1> b;
    // clang-format off
    A <<  1,  1,  // x + y ≤ 1
         -1,  0,  // x ≥ -2
         -1, -1;  // x+y ≥ -1
    b << 1, 2, 1;
    // clang-format on
    HPolyhedron H(A, b);
    EXPECT_FALSE(H.MaybeCalcAxisAlignedBoundingBox().has_value());
  }

  // Case: Bounded parallelogram.
  {
    Matrix<double, 4, 2> A;
    Matrix<double, 4, 1> b;
    // clang-format off
    A <<  1,  0,  // x ≤ 1
          1,  1,  // x + y ≤ 1
         -1,  0,  // x ≥ -2
         -1, -1;  // x+y ≥ -1
    b << 1, 1, 2, 1;
    // clang-format on
    HPolyhedron H(A, b);
    std::optional<Hyperrectangle> aabb_opt =
        H.MaybeCalcAxisAlignedBoundingBox();
    EXPECT_TRUE(aabb_opt.has_value());
    const auto& aabb = aabb_opt.value();
    EXPECT_NEAR(aabb.lb()(0), -2, 1e-6);
    EXPECT_NEAR(aabb.ub()(0), 1, 1e-6);
    EXPECT_NEAR(aabb.lb()(1), -2, 1e-6);
    EXPECT_NEAR(aabb.ub()(1), 3, 1e-6);
  }
}

GTEST_TEST(ConvexSetTest, CalcVolumeViaSampling) {
  Matrix<double, 4, 2> A;
  Matrix<double, 4, 1> b;
  // clang-format off
  A <<  1,  0,  // x ≤ 1
        1,  1,  // x + y ≤ 1
       -1,  0,  // x ≥ -2
       -1, -1;  // x+y ≥ -1
  b << 1, 1, 2, 1;
  // clang-format on
  HPolyhedron H(A, b);

  // Compute the estimated volume of the polytope.
  RandomGenerator generator(1234);
  const auto estimated_volume = H.CalcVolumeViaSampling(&generator, 1e-2);
  // H is a simple parallelogram with volume 6.0.
  EXPECT_NEAR(estimated_volume, 6.0, 1e-1);
};

}  // namespace
}  // namespace optimization
}  // namespace geometry
}  // namespace drake

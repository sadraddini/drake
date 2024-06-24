#include "drake/geometry/optimization/convex_hull.h"

#include <gtest/gtest.h>

#include "drake/common/is_approx_equal_abstol.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperrectangle.h"
#include "drake/geometry/optimization/point.h"
#include "drake/geometry/optimization/test_utilities.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

GTEST_TEST(ConvexHullTest, BasicTests) {
  const Point point(Eigen::Vector2d(1.0, 2.0));
  const Hyperrectangle rectangle(Eigen::Vector2d(-1.0, 1.0),
                                 Eigen::Vector2d(1.0, 1.0));
  ConvexHull hull(MakeConvexSets(point, rectangle));
  EXPECT_EQ(hull.sets().size(), 2);
  // It is not empty
  EXPECT_FALSE(hull.IsEmpty());
  // It is bounded
  EXPECT_TRUE(hull.IsBounded());
  // A point in the convex hull exists
  EXPECT_TRUE(hull.MaybeGetPoint().has_value());
  // Make a HPolyhedron that is empty
  Eigen::MatrixXd A(2, 2);
  A << 1, 0, -1, 0;
  Eigen::VectorXd b(2);
  b << 1, -2;
  HPolyhedron empty_hpolyhedron(A, b);
  ConvexHull empty_hull(MakeConvexSets(point, rectangle, empty_hpolyhedron));
  // It is empty
  EXPECT_TRUE(empty_hull.IsEmpty());
  // It is not bounded
  EXPECT_FALSE(empty_hull.IsBounded());
  // Do not have a point in the convex hull
  EXPECT_FALSE(empty_hull.MaybeGetPoint().has_value());
  // Inppropriate dimensions
  Point point_3d = Point(Eigen::Vector3d(1.0, 2.0, 3.0));
  EXPECT_THROW(ConvexHull(MakeConvexSets(point, point_3d)), std::runtime_error);
}

GTEST_TEST(ConvexHullTest, PointInSet) {
  Point point(Eigen::Vector2d(0.0, 0.0));
  Hyperrectangle rectangle(Eigen::Vector2d(-1.0, 1.0),
                           Eigen::Vector2d(1.0, 1.0));
  ConvexHull hull(MakeConvexSets(point, rectangle));
  EXPECT_TRUE(hull.PointInSet(Eigen::Vector2d(0.0, 0.0), 1e-6));
  EXPECT_TRUE(hull.PointInSet(Eigen::Vector2d(1.0, 1.0), 1e-6));
  EXPECT_TRUE(hull.PointInSet(Eigen::Vector2d(0.4, 0.5), 1e-6));
  EXPECT_TRUE(hull.PointInSet(Eigen::Vector2d(0.5, 0.5), 1e-6));
  EXPECT_FALSE(hull.PointInSet(Eigen::Vector2d(0.6, 0.5), 1e-6));
  // Test tolerances
  EXPECT_FALSE(hull.PointInSet(Eigen::Vector2d(-0.501, 0.5), 1e-4));
  EXPECT_TRUE(hull.PointInSet(Eigen::Vector2d(-0.501, 0.5), 1e-2));
}

GTEST_TEST(ConvexHullTest, AddPointInSetConstraints1) {
  Point point(Eigen::Vector2d(0.0, 0.0));
  Hyperrectangle rectangle(Eigen::Vector2d(-1.0, 1.0),
                           Eigen::Vector2d(1.0, 1.0));
  ConvexHull hull(MakeConvexSets(point, rectangle));
  EXPECT_TRUE(
      internal::CheckAddPointInSetConstraints(hull, Eigen::Vector2d(0.4, 0.4)));
  EXPECT_FALSE(
      internal::CheckAddPointInSetConstraints(hull, Eigen::Vector2d(0.6, 0.4)));
}

GTEST_TEST(ConvexHullTest, AddPointInSetConstraints2) {
  Point point(Eigen::Vector2d(0.0, 0.0));
  Hyperrectangle rectangle(Eigen::Vector2d(-1.0, 1.0),
                           Eigen::Vector2d(1.0, 1.0));
  ConvexHull hull(MakeConvexSets(point, rectangle));
  // solve a mathematical program that finds a point in the convex hull with
  // least L2 distance to (0.8,0) The result should be (0.4, 0.4), easy to see
  // from the geometry
  solvers::MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(2, "x");
  auto [new_vars, new_constraints] = hull.AddPointInSetConstraints(&prog, x);
  prog.AddQuadraticCost((x - Eigen::Vector2d(0.8, 0.0)).squaredNorm());
  const auto result = Solve(prog);
  EXPECT_TRUE(result.is_success());
  const Eigen::VectorXd x_sol = result.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(x_sol, Eigen::Vector2d(0.4, 0.4), 1e-6));
}

GTEST_TEST(ConvexHullTest, AddPointInSetConstraints3) {
  // Make convex hull from a point and another convex hull. Calls
  // AddPointInNonnegativeScalingConstraints for the second convex hull.
  Point point1(Eigen::Vector2d(0.0, 0.0));
  Point point2(Eigen::Vector2d(0.5, 0.0));
  Hyperrectangle rectangle(Eigen::Vector2d(-1.0, 1.0),
                           Eigen::Vector2d(1.0, 1.0));
  ConvexHull hull1(MakeConvexSets(point1, rectangle));
  ConvexHull hull2(MakeConvexSets(hull1, point2));
  // We know that (0.5,0) to (1.0, 1.0) becomes a face
  EXPECT_TRUE(internal::CheckAddPointInSetConstraints(
      hull2, Eigen::Vector2d(0.3, 0.0)));
  EXPECT_TRUE(internal::CheckAddPointInSetConstraints(
      hull2, Eigen::Vector2d(0.6, 0.2)));
  EXPECT_FALSE(internal::CheckAddPointInSetConstraints(
      hull2, Eigen::Vector2d(0.6, 0.1)));
}

GTEST_TEST(ConvexHullTest, AddPointInNonnegativeScalingConstraints1) {
  Point point(Eigen::Vector2d(0.0, 0.0));
  Hyperrectangle rectangle(Eigen::Vector2d(-1.0, 1.0),
                           Eigen::Vector2d(1.0, 1.0));
  ConvexHull hull(MakeConvexSets(point, rectangle));
  solvers::MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(2, "x");
  auto t = prog.NewContinuousVariables(1, "t");
  auto new_constraints =
      hull.AddPointInNonnegativeScalingConstraints(&prog, x, t(0));
  EXPECT_GT(new_constraints.size(), 0);
  // Solve the closest point to (2.0, 1.0). The closest point is (1.5, 1.5) when
  // t = 1.5
  prog.AddQuadraticCost((x - Eigen::Vector2d(2.0, 1.0)).squaredNorm());
  const auto result = Solve(prog);
  EXPECT_TRUE(result.is_success());
  const Eigen::VectorXd x_sol = result.GetSolution(x);
  const Eigen::VectorXd t_sol = result.GetSolution(t);
  EXPECT_TRUE(CompareMatrices(x_sol, Eigen::Vector2d(1.5, 1.5), 1e-4));
  EXPECT_GE(t_sol(0), 1.5);
  // Adding negative constraints on t leads to infeasibility
  prog.AddLinearConstraint(t(0) <= -1.0);
  const auto result2 = Solve(prog);
  EXPECT_FALSE(result2.is_success());
}

GTEST_TEST(ConvexHullTest, AddPointInNonnegativeScalingConstraints2) {
  // Verify by solving a 2D problem and verify the solution
  Point point1(Eigen::Vector2d(0.0, 0.0));
  Point point2(Eigen::Vector2d(0.0, 0.0));
  Hyperrectangle rectangle(Eigen::Vector2d(-1.0, 1.0),
                           Eigen::Vector2d(1.0, 2.0));
  ConvexHull hull(MakeConvexSets(point1, point2, rectangle));
  solvers::MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(2, "x");
  auto t = prog.NewContinuousVariables(3, "t");
  // CCW 90 degree rotation matrix + shift y by 2.0
  Eigen::MatrixXd A(2, 2);
  A << 0, -1, 1, 0;
  Eigen::Vector2d b(0.0, 2.0);
  // Select a 3d vector c = [1, 2, -1] and d = 5.0, just to make the problem
  // more interesting
  Eigen::Vector3d c(1.0, 1.0, 1.0);
  const double d = 0.4;
  auto new_constraints =
      hull.AddPointInNonnegativeScalingConstraints(&prog, A, b, c, d, x, t);
  EXPECT_GT(new_constraints.size(), 0);
  // Pick a point: (-1.7 + a, -0.6). Ax+b will be (0.6, 0.3 + a). It would not
  // be in the convex hull for a = 0
  auto a = prog.NewContinuousVariables(1, "a");
  prog.AddLinearEqualityConstraint(x(0) == -1.7 + a(0));
  prog.AddLinearEqualityConstraint(x(1) == -0.6);
  // It would be in the convex hull for a = 0.3. The smallest (c't + d) that
  // would allow this is 0.6
  prog.AddLinearCost(a(0));
  prog.AddL2NormCost(Eigen::MatrixXd::Identity(3, 3), Eigen::Vector3d::Zero(),
                     t);
  // Solve the problem
  const auto result = Solve(prog);
  EXPECT_TRUE(result.is_success());
  const Eigen::VectorXd t_sol = result.GetSolution(t);
  EXPECT_NEAR(result.GetSolution(a)(0), 0.3, 1e-6);
  EXPECT_NEAR(c.transpose() * t_sol + d, 0.6, 1e-6);
  // We know the t solution, all elements will be 0.2/3
  EXPECT_TRUE(CompareMatrices(t_sol, 0.2 / 3 * Eigen::Vector3d::Ones(), 1e-6));
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake

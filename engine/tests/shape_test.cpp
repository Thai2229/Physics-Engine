#include <gtest/gtest.h>

#include <vector>

#include <physics/shape.hpp>

using namespace physics;

namespace {
constexpr real kTol = 1e-9;
}

TEST(Shape, CircleMassProperties) {
  Shape s = Shape::circle(0.5);
  ASSERT_TRUE(s.isValid());
  MassData md = s.computeMass(2.0);
  real expectedMass = 2.0 * kPi * 0.25;
  EXPECT_NEAR(md.mass, expectedMass, kTol);
  EXPECT_NEAR(md.center.x, 0.0, kTol);
  EXPECT_NEAR(md.center.y, 0.0, kTol);
  EXPECT_NEAR(md.inertia, 0.5 * expectedMass * 0.25, kTol);
  EXPECT_NEAR(s.area(), kPi * 0.25, kTol);
}

TEST(Shape, OffsetCircleUsesParallelAxisTheorem) {
  Shape s = Shape::circle(1.0, Vec2(2, 0));
  MassData md = s.computeMass(1.0);
  EXPECT_NEAR(md.center.x, 2.0, kTol);
  // inertia about the origin = m(r^2/2 + d^2)
  EXPECT_NEAR(md.inertia, md.mass * (0.5 + 4.0), kTol);
}

TEST(Shape, BoxMatchesAnalyticValues) {
  const real w = 2.0;
  const real h = 1.0;
  const real density = 3.0;
  Shape s = Shape::box(w, h);
  ASSERT_TRUE(s.isValid());
  EXPECT_EQ(s.vertexCount(), 4);
  MassData md = s.computeMass(density);
  real m = density * w * h;
  EXPECT_NEAR(s.area(), w * h, kTol);
  EXPECT_NEAR(md.mass, m, kTol);
  EXPECT_NEAR(md.center.x, 0.0, kTol);
  EXPECT_NEAR(md.center.y, 0.0, kTol);
  EXPECT_NEAR(md.inertia, m * (w * w + h * h) / 12.0, kTol);
}

TEST(Shape, OffsetRotatedBox) {
  const real w = 1.0;
  const real h = 3.0;
  Shape s = Shape::box(w, h, Vec2(1, 2), kPi / 2);
  MassData md = s.computeMass(1.0);
  EXPECT_NEAR(md.mass, 3.0, 1e-9);
  EXPECT_NEAR(md.center.x, 1.0, 1e-9);
  EXPECT_NEAR(md.center.y, 2.0, 1e-9);
  // Inertia about own centre is symmetric in w,h; origin inertia adds m d^2.
  real iCenter = md.mass * (w * w + h * h) / 12.0;
  EXPECT_NEAR(md.inertia, iCenter + md.mass * 5.0, 1e-9);
}

TEST(Shape, TriangleAreaCentroidInertia) {
  std::vector<Vec2> v = {Vec2(0, 0), Vec2(3, 0), Vec2(0, 3)};
  Shape s = Shape::polygon(v);
  ASSERT_TRUE(s.isValid());
  EXPECT_NEAR(s.area(), 4.5, kTol);
  MassData md = s.computeMass(1.0);
  EXPECT_NEAR(md.center.x, 1.0, kTol);
  EXPECT_NEAR(md.center.y, 1.0, kTol);
  // right triangle legs a=b=3 about the right-angle vertex: m (a^2 + b^2) / 6
  EXPECT_NEAR(md.inertia, 4.5 * 18.0 / 6.0, kTol);
}

TEST(Shape, ClockwiseVerticesAreReorderedCounterClockwise) {
  std::vector<Vec2> cw = {Vec2(-1, -1), Vec2(-1, 1), Vec2(1, 1), Vec2(1, -1)};
  Shape s = Shape::polygon(cw);
  ASSERT_TRUE(s.isValid());
  EXPECT_NEAR(s.computeMass(1.0).mass, 4.0, kTol);
  // outward normals: every normal points away from the centroid
  for (int i = 0; i < s.vertexCount(); ++i) {
    EXPECT_GT(dot(s.normal(i), s.vertex(i)), 0.0);
    EXPECT_NEAR(s.normal(i).length(), 1.0, kTol);
  }
}

TEST(Shape, InvalidShapesAreRejected) {
  EXPECT_FALSE(Shape::circle(0.0).isValid());
  EXPECT_FALSE(Shape::circle(-1.0).isValid());
  EXPECT_FALSE(Shape::box(0.0, 1.0).isValid());
  EXPECT_FALSE(Shape::polygon(std::vector<Vec2>{}).isValid());
  EXPECT_FALSE(Shape::polygon(std::vector<Vec2>{Vec2(0, 0), Vec2(1, 0)}).isValid());
  EXPECT_FALSE(Shape::polygon(std::vector<Vec2>{Vec2(0, 0), Vec2(1, 0), Vec2(2, 0)}).isValid());
  std::vector<Vec2> nine;
  for (int i = 0; i < 9; ++i) {
    real a = 2.0 * kPi * i / 9.0;
    nine.push_back(Vec2(std::cos(a), std::sin(a)));
  }
  EXPECT_FALSE(Shape::polygon(nine).isValid());
  std::vector<Vec2> eight(nine.begin(), nine.begin() + 8);
  EXPECT_TRUE(Shape::polygon(eight).isValid());
}

TEST(Shape, InvalidShapeHasZeroMass) {
  MassData md = Shape::circle(-1.0).computeMass(1.0);
  EXPECT_EQ(md.mass, 0.0);
  EXPECT_EQ(md.inertia, 0.0);
}

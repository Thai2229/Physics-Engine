#include <gtest/gtest.h>

#include <cmath>

#include <physics/math.hpp>

using namespace physics;

namespace {
constexpr real kTol = 1e-9;
}

TEST(Vec2, Algebra) {
  Vec2 a(1, 2);
  Vec2 b(-3, 4);
  EXPECT_EQ(a + b, Vec2(-2, 6));
  EXPECT_EQ(a - b, Vec2(4, -2));
  EXPECT_EQ(-a, Vec2(-1, -2));
  EXPECT_EQ(a * 2.0, Vec2(2, 4));
  EXPECT_EQ(2.0 * a, Vec2(2, 4));
  Vec2 c = a;
  c += b;
  EXPECT_EQ(c, Vec2(-2, 6));
  c -= b;
  EXPECT_EQ(c, a);
  c *= 3.0;
  EXPECT_EQ(c, Vec2(3, 6));
}

TEST(Vec2, DotAndCrossIdentities) {
  Vec2 a(1.5, -2);
  Vec2 b(0.25, 7);
  EXPECT_NEAR(dot(a, b), dot(b, a), kTol);
  EXPECT_NEAR(cross(a, b), -cross(b, a), kTol);
  EXPECT_NEAR(cross(a, a), 0.0, kTol);
  EXPECT_NEAR(dot(cross(2.0, a), a), 0.0, kTol);
  EXPECT_EQ(cross(a, 2.0), -cross(2.0, a));
  EXPECT_EQ(cross(1.0, Vec2(1, 0)), Vec2(0, 1));
  // Lagrange identity: |a|^2 |b|^2 = dot^2 + cross^2
  EXPECT_NEAR(a.lengthSquared() * b.lengthSquared(), dot(a, b) * dot(a, b) + cross(a, b) * cross(a, b), 1e-9);
}

TEST(Vec2, LengthAndNormalize) {
  Vec2 a(3, 4);
  EXPECT_NEAR(a.length(), 5.0, kTol);
  EXPECT_NEAR(a.lengthSquared(), 25.0, kTol);
  EXPECT_NEAR(distance(Vec2(1, 1), Vec2(4, 5)), 5.0, kTol);
  Vec2 n = a;
  real len = n.normalize();
  EXPECT_NEAR(len, 5.0, kTol);
  EXPECT_NEAR(n.length(), 1.0, kTol);
  EXPECT_NEAR(normalized(a).x, 0.6, kTol);
}

TEST(Vec2, NormalizeGuardsAgainstZeroLength) {
  Vec2 z(0, 0);
  EXPECT_EQ(z.normalize(), 0.0);
  EXPECT_EQ(z, Vec2(0, 0));
  Vec2 tiny(1e-20, 0);
  EXPECT_EQ(normalized(tiny), Vec2(0, 0));
  EXPECT_FALSE(std::isnan(normalized(z).x));
}

TEST(Rot, RotatesVectors) {
  Rot r(kPi / 2);
  Vec2 v = mul(r, Vec2(1, 0));
  EXPECT_NEAR(v.x, 0.0, kTol);
  EXPECT_NEAR(v.y, 1.0, kTol);
  EXPECT_NEAR(r.angle(), kPi / 2, kTol);
  EXPECT_NEAR(Rot(0.3).angle(), 0.3, kTol);
}

TEST(Rot, InverseRoundTrip) {
  Rot r(1.234);
  Vec2 v(2.5, -1.5);
  Vec2 w = mulT(r, mul(r, v));
  EXPECT_NEAR(w.x, v.x, kTol);
  EXPECT_NEAR(w.y, v.y, kTol);
  EXPECT_NEAR(mul(r, v).length(), v.length(), kTol);
}

TEST(Mat22, SolveAndInverse) {
  Mat22 m(2, 1, 1, 3);  // symmetric [[2,1],[1,3]]
  Vec2 x = m.solve(Vec2(5, 10));
  Vec2 back = mul(m, x);
  EXPECT_NEAR(back.x, 5.0, kTol);
  EXPECT_NEAR(back.y, 10.0, kTol);
  Mat22 inv = m.inverse();
  Vec2 x2 = mul(inv, Vec2(5, 10));
  EXPECT_NEAR(x2.x, x.x, kTol);
  EXPECT_NEAR(x2.y, x.y, kTol);
}

TEST(Mat22, SingularSolveReturnsZeroInsteadOfNaN) {
  Mat22 m(1, 2, 2, 4);
  Vec2 x = m.solve(Vec2(1, 1));
  EXPECT_EQ(x, Vec2(0, 0));
  Mat22 inv = m.inverse();
  EXPECT_EQ(mul(inv, Vec2(1, 1)), Vec2(0, 0));
}

TEST(Transform, RoundTrip) {
  Transform t(Vec2(3, -2), Rot(0.7));
  Vec2 p(1, 1);
  Vec2 w = mul(t, p);
  Vec2 back = mulT(t, w);
  EXPECT_NEAR(back.x, p.x, kTol);
  EXPECT_NEAR(back.y, p.y, kTol);
  Transform id;
  EXPECT_EQ(mul(id, p), p);
}

TEST(Helpers, ClampMinMax) {
  EXPECT_EQ(clamp(5.0, 0.0, 1.0), 1.0);
  EXPECT_EQ(clamp(-5.0, 0.0, 1.0), 0.0);
  EXPECT_EQ(clamp(0.5, 0.0, 1.0), 0.5);
  EXPECT_EQ(minReal(1.0, 2.0), 1.0);
  EXPECT_EQ(maxReal(1.0, 2.0), 2.0);
}

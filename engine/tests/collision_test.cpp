#include <gtest/gtest.h>

#include <cmath>
#include <set>
#include <utility>
#include <vector>

#include <physics/collision.hpp>

using namespace physics;

namespace {

Transform T(real x, real y, real angle = 0) {
  return Transform(Vec2(x, y), Rot(angle));
}

WorldManifold world(const Manifold& m, const Shape& a, const Transform& xa, const Shape& b, const Transform& xb) {
  return computeWorldManifold(m, a, xa, b, xb);
}

}  // namespace

// ---- AABB & broadphase ----------------------------------------------------------

TEST(Aabb, CircleBoundsIncludeRadiusAndOffset) {
  AABB box = computeAABB(Shape::circle(0.5, Vec2(1, 0)), T(2, 3));
  EXPECT_NEAR(box.lower.x, 2.5, 1e-12);
  EXPECT_NEAR(box.upper.x, 3.5, 1e-12);
  EXPECT_NEAR(box.lower.y, 2.5, 1e-12);
  EXPECT_NEAR(box.upper.y, 3.5, 1e-12);
}

TEST(Aabb, RotatedBoxBoundsAreExpanded) {
  AABB box = computeAABB(Shape::box(2, 2), T(0, 0, kPi / 4));
  EXPECT_NEAR(box.upper.x, std::sqrt(2.0), 1e-9);
  EXPECT_NEAR(box.lower.y, -std::sqrt(2.0), 1e-9);
}

TEST(Aabb, TouchingBoxesOverlapSeparatedDoNot) {
  AABB a{Vec2(0, 0), Vec2(1, 1)};
  EXPECT_TRUE(overlaps(a, AABB{Vec2(1, 0), Vec2(2, 1)}));
  EXPECT_TRUE(overlaps(a, AABB{Vec2(0.5, 0.5), Vec2(2, 2)}));
  EXPECT_FALSE(overlaps(a, AABB{Vec2(1.01, 0), Vec2(2, 1)}));
  EXPECT_FALSE(overlaps(a, AABB{Vec2(0, 1.01), Vec2(1, 2)}));
}

TEST(Broadphase, ReportsOnlyOverlappingPairsSortedByIds) {
  std::vector<BroadphaseEntry> e;
  e.push_back({AABB{Vec2(5, 0), Vec2(6, 1)}, 0});
  e.push_back({AABB{Vec2(0, 0), Vec2(1, 1)}, 1});
  e.push_back({AABB{Vec2(0.5, 0.5), Vec2(1.5, 1.5)}, 2});
  e.push_back({AABB{Vec2(0.5, 3), Vec2(1.5, 4)}, 3});  // x-overlaps 1 and 2 but not in y
  auto pairs = findOverlappingPairs(e);
  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_EQ(pairs[0], std::make_pair(1u, 2u));
}

TEST(Broadphase, EmptyAndSingleInputGiveNoPairs) {
  EXPECT_TRUE(findOverlappingPairs({}).empty());
  EXPECT_TRUE(findOverlappingPairs({BroadphaseEntry{AABB{Vec2(0, 0), Vec2(1, 1)}, 7}}).empty());
}

TEST(Broadphase, MatchesBruteForceOnManyBoxes) {
  std::vector<BroadphaseEntry> e;
  unsigned seed = 12345;
  auto rnd = [&]() {
    seed = seed * 1664525u + 1013904223u;
    return static_cast<real>((seed >> 8) & 0xFFFF) / 65535.0;
  };
  const int n = 2000;
  for (int i = 0; i < n; ++i) {
    real x = rnd() * 100.0;
    real y = rnd() * 100.0;
    e.push_back({AABB{Vec2(x, y), Vec2(x + 1.0, y + 1.0)}, static_cast<std::uint32_t>(i)});
  }
  std::set<std::pair<std::uint32_t, std::uint32_t>> expected;
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      if (overlaps(e[i].box, e[j].box)) {
        expected.insert(std::make_pair(static_cast<std::uint32_t>(i), static_cast<std::uint32_t>(j)));
      }
    }
  }
  auto pairs = findOverlappingPairs(e);
  std::set<std::pair<std::uint32_t, std::uint32_t>> got(pairs.begin(), pairs.end());
  EXPECT_EQ(got.size(), pairs.size());  // no duplicates
  EXPECT_EQ(got, expected);
  EXPECT_GT(expected.size(), 0u);
}

// ---- circle vs circle -----------------------------------------------------------

TEST(CollideCircles, OverlapGivesNormalDepthAndMidpoint) {
  Shape a = Shape::circle(1.0);
  Shape b = Shape::circle(1.0);
  Manifold m = collideCircles(a, T(0, 0), b, T(1.5, 0));
  ASSERT_EQ(m.pointCount, 1);
  WorldManifold w = world(m, a, T(0, 0), b, T(1.5, 0));
  EXPECT_NEAR(w.normal.x, 1.0, 1e-9);
  EXPECT_NEAR(w.normal.y, 0.0, 1e-9);
  EXPECT_NEAR(w.separations[0], -0.5, 1e-9);
  EXPECT_NEAR(w.points[0].x, 0.75, 1e-9);
}

TEST(CollideCircles, TouchingProducesZeroDepthContact) {
  Shape c = Shape::circle(1.0);
  Manifold m = collideCircles(c, T(0, 0), c, T(2, 0));
  ASSERT_EQ(m.pointCount, 1);
  EXPECT_NEAR(world(m, c, T(0, 0), c, T(2, 0)).separations[0], 0.0, 1e-9);
}

TEST(CollideCircles, SeparatedGivesNoContact) {
  Shape c = Shape::circle(1.0);
  EXPECT_EQ(collideCircles(c, T(0, 0), c, T(2.01, 0)).pointCount, 0);
}

TEST(CollideCircles, NormalPointsFromAToBInAnyDirection) {
  Shape c = Shape::circle(1.0);
  Manifold m = collideCircles(c, T(0, 0), c, T(-0.6, -0.8));
  ASSERT_EQ(m.pointCount, 1);
  WorldManifold w = world(m, c, T(0, 0), c, T(-0.6, -0.8));
  EXPECT_NEAR(w.normal.x, -0.6, 1e-9);
  EXPECT_NEAR(w.normal.y, -0.8, 1e-9);
  EXPECT_NEAR(w.separations[0], -1.0, 1e-9);
}

TEST(CollideCircles, OffsetCentresAndRotationAreRespected) {
  Shape a = Shape::circle(0.5, Vec2(1, 0));  // world centre (0,1) after a 90 degree turn
  Shape b = Shape::circle(0.5);
  Manifold m = collideCircles(a, T(0, 0, kPi / 2), b, T(0, 1.8));
  ASSERT_EQ(m.pointCount, 1);
  WorldManifold w = world(m, a, T(0, 0, kPi / 2), b, T(0, 1.8));
  EXPECT_NEAR(w.normal.y, 1.0, 1e-9);
  EXPECT_NEAR(w.separations[0], -0.2, 1e-9);
}

TEST(CollideCircles, CoincidentCentresStillGiveUnitNormal) {
  Shape c = Shape::circle(1.0);
  Manifold m = collideCircles(c, T(0, 0), c, T(0, 0));
  ASSERT_EQ(m.pointCount, 1);
  WorldManifold w = world(m, c, T(0, 0), c, T(0, 0));
  EXPECT_NEAR(w.normal.length(), 1.0, 1e-9);
  EXPECT_NEAR(w.separations[0], -2.0, 1e-9);
}

// ---- polygon vs circle ----------------------------------------------------------

TEST(CollidePolygonCircle, FaceOverlap) {
  Shape box = Shape::box(1, 1);
  Shape c = Shape::circle(0.5);
  Manifold m = collidePolygonAndCircle(box, T(0, 0), c, T(0, 0.9));
  ASSERT_EQ(m.pointCount, 1);
  WorldManifold w = world(m, box, T(0, 0), c, T(0, 0.9));
  EXPECT_NEAR(w.normal.x, 0.0, 1e-9);
  EXPECT_NEAR(w.normal.y, 1.0, 1e-9);
  EXPECT_NEAR(w.separations[0], -0.1, 1e-9);
  EXPECT_NEAR(w.points[0].y, 0.45, 1e-9);
}

TEST(CollidePolygonCircle, TouchingFaceAndSeparated) {
  Shape box = Shape::box(1, 1);
  Shape c = Shape::circle(0.5);
  Manifold touching = collidePolygonAndCircle(box, T(0, 0), c, T(0, 1.0));
  ASSERT_EQ(touching.pointCount, 1);
  EXPECT_NEAR(world(touching, box, T(0, 0), c, T(0, 1.0)).separations[0], 0.0, 1e-9);
  EXPECT_EQ(collidePolygonAndCircle(box, T(0, 0), c, T(0, 1.01)).pointCount, 0);
}

TEST(CollidePolygonCircle, VertexRegionUsesDiagonalNormal) {
  Shape box = Shape::box(1, 1);
  Shape c = Shape::circle(0.5);
  Transform xb = T(0.8, 0.8);
  Manifold m = collidePolygonAndCircle(box, T(0, 0), c, xb);
  ASSERT_EQ(m.pointCount, 1);
  WorldManifold w = world(m, box, T(0, 0), c, xb);
  EXPECT_NEAR(w.normal.x, std::sqrt(0.5), 1e-9);
  EXPECT_NEAR(w.normal.y, std::sqrt(0.5), 1e-9);
  EXPECT_NEAR(w.separations[0], 0.3 * std::sqrt(2.0) - 0.5, 1e-9);
}

TEST(CollidePolygonCircle, NearCornerButOutsideRadiusIsSeparated) {
  Shape box = Shape::box(1, 1);
  Shape c = Shape::circle(0.5);
  // Within the box's AABB-expanded corner square but farther than r from the vertex.
  EXPECT_EQ(collidePolygonAndCircle(box, T(0, 0), c, T(1.0, 1.0)).pointCount, 0);
}

TEST(CollidePolygonCircle, CentreInsidePolygonPushesOutThroughNearestFace) {
  Shape box = Shape::box(1, 1);
  Shape c = Shape::circle(0.1);
  Manifold m = collidePolygonAndCircle(box, T(0, 0), c, T(0.4, 0.0));
  ASSERT_EQ(m.pointCount, 1);
  WorldManifold w = world(m, box, T(0, 0), c, T(0.4, 0.0));
  EXPECT_NEAR(w.normal.x, 1.0, 1e-9);
  EXPECT_NEAR(w.separations[0], -0.2, 1e-9);
}

TEST(CollidePolygonCircle, RotatedPolygonNormalFollowsRotation) {
  Shape box = Shape::box(2, 1);
  Shape c = Shape::circle(0.5);
  Transform xa = T(0, 0, kPi / 2);  // box is now 1 wide and 2 tall
  Manifold m = collidePolygonAndCircle(box, xa, c, T(0, 1.4));
  ASSERT_EQ(m.pointCount, 1);
  WorldManifold w = world(m, box, xa, c, T(0, 1.4));
  EXPECT_NEAR(w.normal.y, 1.0, 1e-9);
  EXPECT_NEAR(w.separations[0], -0.1, 1e-9);
}

// ---- polygon vs polygon ---------------------------------------------------------

TEST(CollidePolygons, StackedBoxesGiveTwoPointsWithNormalAndDepth) {
  Shape box = Shape::box(1, 1);
  Manifold m = collidePolygons(box, T(0, 0), box, T(0, 0.9));
  ASSERT_EQ(m.pointCount, 2);
  WorldManifold w = world(m, box, T(0, 0), box, T(0, 0.9));
  EXPECT_NEAR(w.normal.x, 0.0, 1e-9);
  EXPECT_NEAR(w.normal.y, 1.0, 1e-9);
  for (int i = 0; i < 2; ++i) {
    EXPECT_NEAR(w.separations[i], -0.1, 1e-9);
    EXPECT_NEAR(w.points[i].y, 0.45, 1e-9);
    EXPECT_NEAR(std::fabs(w.points[i].x), 0.5, 1e-9);
  }
  EXPECT_NE(w.points[0].x, w.points[1].x);
}

TEST(CollidePolygons, TouchingGivesZeroDepthAndSeparatedGivesNone) {
  Shape box = Shape::box(1, 1);
  Manifold touching = collidePolygons(box, T(0, 0), box, T(0, 1.0));
  ASSERT_EQ(touching.pointCount, 2);
  WorldManifold w = world(touching, box, T(0, 0), box, T(0, 1.0));
  EXPECT_NEAR(w.separations[0], 0.0, 1e-9);
  EXPECT_NEAR(w.separations[1], 0.0, 1e-9);
  EXPECT_EQ(collidePolygons(box, T(0, 0), box, T(0, 1.01)).pointCount, 0);
  EXPECT_EQ(collidePolygons(box, T(0, 0), box, T(-1.01, 0)).pointCount, 0);
}

TEST(CollidePolygons, NormalAlwaysPointsFromAToB) {
  Shape box = Shape::box(1, 1);
  Manifold right = collidePolygons(box, T(0, 0), box, T(0.9, 0));
  EXPECT_NEAR(world(right, box, T(0, 0), box, T(0.9, 0)).normal.x, 1.0, 1e-9);
  Manifold left = collidePolygons(box, T(0, 0), box, T(-0.9, 0));
  EXPECT_NEAR(world(left, box, T(0, 0), box, T(-0.9, 0)).normal.x, -1.0, 1e-9);
  Manifold below = collidePolygons(box, T(0, 0), box, T(0, -0.9));
  EXPECT_NEAR(world(below, box, T(0, 0), box, T(0, -0.9)).normal.y, -1.0, 1e-9);
  // Swapping the arguments flips the normal (reference face on B).
  Manifold swapped = collidePolygons(box, T(0, 0.9), box, T(0, 0));
  WorldManifold w = world(swapped, box, T(0, 0.9), box, T(0, 0));
  EXPECT_NEAR(w.normal.y, -1.0, 1e-9);
  ASSERT_EQ(swapped.pointCount, 2);
  EXPECT_NEAR(w.separations[0], -0.1, 1e-9);
}

TEST(CollidePolygons, PartialOverlapIsClippedToReferenceFace) {
  Shape box = Shape::box(1, 1);
  Manifold m = collidePolygons(box, T(0, 0), box, T(0.75, 0.9));
  ASSERT_EQ(m.pointCount, 2);
  WorldManifold w = world(m, box, T(0, 0), box, T(0.75, 0.9));
  real lo = minReal(w.points[0].x, w.points[1].x);
  real hi = maxReal(w.points[0].x, w.points[1].x);
  EXPECT_NEAR(lo, 0.25, 1e-9);
  EXPECT_NEAR(hi, 0.5, 1e-9);
}

TEST(CollidePolygons, TiltedBoxLandingOnVertexGivesSinglePoint) {
  Shape box = Shape::box(1, 1);
  Transform xb = T(0, 0.5 + std::sqrt(0.5) - 0.05, kPi / 4);
  Manifold m = collidePolygons(box, T(0, 0), box, xb);
  ASSERT_EQ(m.pointCount, 1);
  WorldManifold w = world(m, box, T(0, 0), box, xb);
  EXPECT_NEAR(w.normal.y, 1.0, 1e-9);
  EXPECT_NEAR(w.separations[0], -0.05, 1e-9);
  EXPECT_NEAR(w.points[0].x, 0.0, 1e-9);
}

TEST(CollidePolygons, SeparationOnlyAlongAFaceNormalOfBIsDetected) {
  // The diamond overlaps the square on both of the square's axes; only the
  // diamond's own face normal separates them.
  Shape box = Shape::box(1, 1);
  EXPECT_EQ(collidePolygons(box, T(0, 0), box, T(1.0, 1.0, kPi / 4)).pointCount, 0);
  EXPECT_EQ(collidePolygons(box, T(1.0, 1.0, kPi / 4), box, T(0, 0)).pointCount, 0);
}

TEST(CollidePolygons, DiamondAgainstFacePenetration) {
  Shape box = Shape::box(1, 1);
  Manifold m = collidePolygons(box, T(0, 0), box, T(1.15, 0, kPi / 4));
  ASSERT_GE(m.pointCount, 1);
  WorldManifold w = world(m, box, T(0, 0), box, T(1.15, 0, kPi / 4));
  EXPECT_NEAR(w.normal.x, 1.0, 1e-9);
  EXPECT_NEAR(w.separations[0], (1.15 - std::sqrt(0.5)) - 0.5, 1e-9);
}

TEST(CollidePolygons, TriangleOnBoxAndTransformsApply) {
  Shape tri = Shape::polygon({Vec2(-0.5, 0), Vec2(0.5, 0), Vec2(0, 1)});
  Shape ground = Shape::box(10, 1);
  Manifold m = collidePolygons(ground, T(3, 0), tri, T(3.2, 0.45));
  ASSERT_EQ(m.pointCount, 2);
  WorldManifold w = world(m, ground, T(3, 0), tri, T(3.2, 0.45));
  EXPECT_NEAR(w.normal.y, 1.0, 1e-9);
  EXPECT_NEAR(w.separations[0], -0.05, 1e-9);
}

TEST(CollidePolygons, FeatureIdsAreDistinctAndStableUnderSliding) {
  Shape box = Shape::box(1, 1);
  Shape narrow = Shape::box(0.6, 1);  // stays inside the reference face, so no vertex is clipped
  Manifold a = collidePolygons(box, T(0, 0), narrow, T(0.00, 0.95));
  Manifold b = collidePolygons(box, T(0, 0), narrow, T(0.05, 0.94));
  ASSERT_EQ(a.pointCount, 2);
  ASSERT_EQ(b.pointCount, 2);
  EXPECT_NE(a.points[0].id, a.points[1].id);
  std::set<std::uint32_t> ia = {a.points[0].id, a.points[1].id};
  std::set<std::uint32_t> ib = {b.points[0].id, b.points[1].id};
  EXPECT_EQ(ia, ib);
}

TEST(CollidePolygons, ImpulseFieldsStartAtZero) {
  Shape box = Shape::box(1, 1);
  Manifold m = collidePolygons(box, T(0, 0), box, T(0, 0.9));
  EXPECT_EQ(m.points[0].normalImpulse, 0.0);
  EXPECT_EQ(m.points[1].tangentImpulse, 0.0);
}

// ---- dispatch -------------------------------------------------------------------

TEST(Collide, DispatchesOnShapeTypes) {
  Shape box = Shape::box(1, 1);
  Shape c = Shape::circle(0.5);
  EXPECT_EQ(collide(c, T(0, 0), c, T(0.9, 0)).pointCount, 1);
  EXPECT_EQ(collide(box, T(0, 0), c, T(0, 0.9)).pointCount, 1);
  EXPECT_EQ(collide(box, T(0, 0), box, T(0, 0.9)).pointCount, 2);
  EXPECT_EQ(collide(box, T(0, 0), c, T(0, 3)).pointCount, 0);
}

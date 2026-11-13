#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "math.hpp"
#include "shape.hpp"

namespace physics {

// ---- broadphase ----------------------------------------------------------------

struct AABB {
  Vec2 lower;
  Vec2 upper;
};

/// Closed-interval test: boxes that merely touch overlap.
bool overlaps(const AABB& a, const AABB& b);
AABB computeAABB(const Shape& shape, const Transform& xf);

struct BroadphaseEntry {
  AABB box;
  std::uint32_t id = 0;
};

/// Sort-and-sweep along the x axis. Returns every pair of entry ids whose boxes
/// overlap, as (smaller id, larger id), sorted ascending (deterministic).
std::vector<std::pair<std::uint32_t, std::uint32_t>> findOverlappingPairs(std::vector<BroadphaseEntry> entries);

// ---- narrowphase ---------------------------------------------------------------

enum class ManifoldType { Circles, FaceA, FaceB };

struct ManifoldPoint {
  Vec2 localPoint;  ///< see Manifold for the frame it lives in
  std::uint32_t id = 0;  ///< feature id, stable while the same features touch (warm starting)
  real normalImpulse = 0;
  real tangentImpulse = 0;
};

/// Contact manifold in body-local data so it can be re-evaluated while bodies move.
///  Circles: localPoint = centre of A (frame A); points[0].localPoint = centre of B (frame B).
///  FaceA:   localNormal/localPoint = reference face of A (frame A); points are in frame B.
///  FaceB:   localNormal/localPoint = reference face of B (frame B); points are in frame A.
struct Manifold {
  ManifoldType type = ManifoldType::Circles;
  Vec2 localNormal;
  Vec2 localPoint;
  ManifoldPoint points[2];
  int pointCount = 0;
};

struct WorldManifold {
  Vec2 normal;  ///< unit, from A to B
  Vec2 points[2];  ///< midway between the two surfaces
  real separations[2] = {0, 0};  ///< negative when penetrating
};

WorldManifold computeWorldManifold(const Manifold& m, const Shape& shapeA, const Transform& xfA, const Shape& shapeB,
                                   const Transform& xfB);

/// All functions return pointCount == 0 when the shapes are separated.
Manifold collideCircles(const Shape& a, const Transform& xfA, const Shape& b, const Transform& xfB);
/// polygon = A, circle = B.
Manifold collidePolygonAndCircle(const Shape& polygon, const Transform& xfA, const Shape& circle, const Transform& xfB);
Manifold collidePolygons(const Shape& a, const Transform& xfA, const Shape& b, const Transform& xfB);
/// Dispatches on shape types. For mixed pairs A must be the polygon and B the
/// circle (swap before calling); a circle-first mixed pair yields no contact.
Manifold collide(const Shape& a, const Transform& xfA, const Shape& b, const Transform& xfB);

}  // namespace physics

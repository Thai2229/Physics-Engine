#include <physics/collision.hpp>

namespace physics {

bool overlaps(const AABB&, const AABB&) { return false; }
AABB computeAABB(const Shape&, const Transform&) { return AABB(); }
std::vector<std::pair<std::uint32_t, std::uint32_t>> findOverlappingPairs(std::vector<BroadphaseEntry>) { return {}; }
WorldManifold computeWorldManifold(const Manifold&, const Shape&, const Transform&, const Shape&, const Transform&) {
  return WorldManifold();
}
Manifold collideCircles(const Shape&, const Transform&, const Shape&, const Transform&) { return Manifold(); }
Manifold collidePolygonAndCircle(const Shape&, const Transform&, const Shape&, const Transform&) { return Manifold(); }
Manifold collidePolygons(const Shape&, const Transform&, const Shape&, const Transform&) { return Manifold(); }
Manifold collide(const Shape&, const Transform&, const Shape&, const Transform&) { return Manifold(); }

}  // namespace physics

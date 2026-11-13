#pragma once

#include <vector>

#include "math.hpp"

namespace physics {

enum class ShapeType { Circle, Polygon };

constexpr int kMaxPolygonVertices = 8;

struct MassData {
  real mass = 0;
  Vec2 center;       ///< centroid in the body frame
  real inertia = 0;  ///< rotational inertia about the body-frame origin
};

/// Convex shape in body-local coordinates. Invalid construction yields a shape
/// with isValid() == false (no exceptions, so it is safe under Emscripten).
class Shape {
 public:
  static Shape circle(real radius, Vec2 center = Vec2(0, 0));
  static Shape box(real width, real height, Vec2 center = Vec2(0, 0), real angle = 0);
  /// Convex polygon, 3..kMaxPolygonVertices vertices; either winding accepted.
  static Shape polygon(const std::vector<Vec2>& vertices);

  ShapeType type() const { return type_; }
  bool isValid() const { return valid_; }
  real radius() const { return radius_; }
  Vec2 center() const { return center_; }
  int vertexCount() const { return count_; }
  const Vec2& vertex(int i) const { return vertices_[i]; }
  const Vec2& normal(int i) const { return normals_[i]; }

  real area() const;
  MassData computeMass(real density) const;

 private:
  ShapeType type_ = ShapeType::Circle;
  bool valid_ = false;
  real radius_ = 0;
  Vec2 center_;
  int count_ = 0;
  Vec2 vertices_[kMaxPolygonVertices];
  Vec2 normals_[kMaxPolygonVertices];
};

struct Filter {
  unsigned category = 0x0001;
  unsigned mask = 0xFFFF;
  int group = 0;
};

/// A shape plus material and collision-filter data (used by M4 collision).
struct Fixture {
  Shape shape;
  real density = 1.0;
  real friction = 0.3;
  real restitution = 0.0;
  Filter filter;
};

}  // namespace physics

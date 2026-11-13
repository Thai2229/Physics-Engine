#include <physics/shape.hpp>

#include <cmath>

namespace physics {

Shape Shape::circle(real radius, Vec2 center) {
  Shape s;
  s.type_ = ShapeType::Circle;
  s.radius_ = radius;
  s.center_ = center;
  s.valid_ = radius > 0.0 && std::isfinite(radius);
  return s;
}

Shape Shape::box(real width, real height, Vec2 center, real angle) {
  real hx = 0.5 * width;
  real hy = 0.5 * height;
  Rot q(angle);
  std::vector<Vec2> v = {Vec2(-hx, -hy), Vec2(hx, -hy), Vec2(hx, hy), Vec2(-hx, hy)};
  for (Vec2& p : v) {
    p = mul(q, p) + center;
  }
  return polygon(v);
}

Shape Shape::polygon(const std::vector<Vec2>& vertices) {
  Shape s;
  s.type_ = ShapeType::Polygon;
  int n = static_cast<int>(vertices.size());
  if (n < 3 || n > kMaxPolygonVertices) {
    return s;
  }
  real twiceArea = 0;
  for (int i = 0; i < n; ++i) {
    twiceArea += cross(vertices[i], vertices[(i + 1) % n]);
  }
  if (!(std::fabs(twiceArea) > 1e-9)) {
    return s;
  }
  s.count_ = n;
  for (int i = 0; i < n; ++i) {
    if (twiceArea > 0.0) {
      s.vertices_[i] = vertices[i];
    } else {
      s.vertices_[i] = vertices[n - 1 - i];
    }
  }
  for (int i = 0; i < n; ++i) {
    Vec2 edge = s.vertices_[(i + 1) % n] - s.vertices_[i];
    s.normals_[i] = normalized(cross(edge, 1.0));
  }
  s.valid_ = true;
  return s;
}

real Shape::area() const {
  return computeMass(1.0).mass;
}

MassData Shape::computeMass(real density) const {
  MassData md;
  if (!valid_) {
    return md;
  }
  if (type_ == ShapeType::Circle) {
    md.mass = density * kPi * radius_ * radius_;
    md.center = center_;
    md.inertia = md.mass * (0.5 * radius_ * radius_ + dot(center_, center_));
    return md;
  }
  Vec2 c(0, 0);
  real area = 0;
  real inertia = 0;
  Vec2 ref = vertices_[0];
  for (int i = 0; i < count_; ++i) {
    Vec2 e1 = vertices_[i] - ref;
    Vec2 e2 = vertices_[(i + 1) % count_] - ref;
    real d = cross(e1, e2);
    real triArea = 0.5 * d;
    area += triArea;
    c += (triArea / 3.0) * (e1 + e2);
    real intx2 = e1.x * e1.x + e2.x * e1.x + e2.x * e2.x;
    real inty2 = e1.y * e1.y + e2.y * e1.y + e2.y * e2.y;
    inertia += (0.25 / 3.0 * d) * (intx2 + inty2);
  }
  md.mass = density * area;
  c *= 1.0 / area;
  md.center = ref + c;
  inertia *= density;
  md.inertia = inertia + md.mass * (dot(md.center, md.center) - dot(c, c));
  return md;
}

}  // namespace physics

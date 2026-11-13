#include <physics/collision.hpp>

#include <algorithm>
#include <cmath>

namespace physics {

namespace {

// Feature id layout: indexA | indexB << 8 | typeA << 16 | typeB << 24.
constexpr std::uint32_t kVertex = 0;
constexpr std::uint32_t kFace = 1;

struct ClipVertex {
  Vec2 v;
  std::uint32_t indexA = 0;
  std::uint32_t indexB = 0;
  std::uint32_t typeA = kVertex;
  std::uint32_t typeB = kVertex;
};

std::uint32_t packId(std::uint32_t indexA, std::uint32_t indexB, std::uint32_t typeA, std::uint32_t typeB) {
  return indexA | (indexB << 8) | (typeA << 16) | (typeB << 24);
}

/// Largest separation of poly2 from the faces of poly1 (world space), and the face index.
real findMaxSeparation(int& edgeIndex, const Shape& poly1, const Transform& xf1, const Shape& poly2,
                       const Transform& xf2) {
  int bestIndex = 0;
  real maxSeparation = -1e300;
  for (int i = 0; i < poly1.vertexCount(); ++i) {
    Vec2 n = mul(xf1.q, poly1.normal(i));
    Vec2 v1 = mul(xf1, poly1.vertex(i));
    real si = 1e300;
    for (int j = 0; j < poly2.vertexCount(); ++j) {
      si = minReal(si, dot(n, mul(xf2, poly2.vertex(j)) - v1));
    }
    if (si > maxSeparation) {
      maxSeparation = si;
      bestIndex = i;
    }
  }
  edgeIndex = bestIndex;
  return maxSeparation;
}

/// Edge of poly2 most anti-parallel to face edge1 of poly1, as world-space clip vertices.
void findIncidentEdge(ClipVertex c[2], const Shape& poly1, const Transform& xf1, int edge1, const Shape& poly2,
                      const Transform& xf2) {
  Vec2 normal1 = mulT(xf2.q, mul(xf1.q, poly1.normal(edge1)));
  int index = 0;
  real minDot = 1e300;
  for (int i = 0; i < poly2.vertexCount(); ++i) {
    real d = dot(normal1, poly2.normal(i));
    if (d < minDot) {
      minDot = d;
      index = i;
    }
  }
  int i1 = index;
  int i2 = (i1 + 1) % poly2.vertexCount();
  c[0].v = mul(xf2, poly2.vertex(i1));
  c[0].indexA = static_cast<std::uint32_t>(edge1);
  c[0].indexB = static_cast<std::uint32_t>(i1);
  c[0].typeA = kFace;
  c[0].typeB = kVertex;
  c[1].v = mul(xf2, poly2.vertex(i2));
  c[1].indexA = static_cast<std::uint32_t>(edge1);
  c[1].indexB = static_cast<std::uint32_t>(i2);
  c[1].typeA = kFace;
  c[1].typeB = kVertex;
}

/// Sutherland-Hodgman clip of a segment against the half-space dot(normal, p) <= offset.
int clipSegmentToLine(ClipVertex out[2], const ClipVertex in[2], const Vec2& normal, real offset,
                      std::uint32_t vertexIndexA) {
  int n = 0;
  real d0 = dot(normal, in[0].v) - offset;
  real d1 = dot(normal, in[1].v) - offset;
  if (d0 <= 0.0) {
    out[n++] = in[0];
  }
  if (d1 <= 0.0) {
    out[n++] = in[1];
  }
  if (d0 * d1 < 0.0) {
    real interp = d0 / (d0 - d1);
    out[n].v = in[0].v + interp * (in[1].v - in[0].v);
    out[n].indexA = vertexIndexA;
    out[n].indexB = in[0].indexB;
    out[n].typeA = kVertex;
    out[n].typeB = kFace;
    ++n;
  }
  return n;
}

}  // namespace

// ---- broadphase ----------------------------------------------------------------

bool overlaps(const AABB& a, const AABB& b) {
  return a.lower.x <= b.upper.x && b.lower.x <= a.upper.x && a.lower.y <= b.upper.y && b.lower.y <= a.upper.y;
}

AABB computeAABB(const Shape& shape, const Transform& xf) {
  AABB box;
  if (shape.type() == ShapeType::Circle) {
    Vec2 c = mul(xf, shape.center());
    box.lower = Vec2(c.x - shape.radius(), c.y - shape.radius());
    box.upper = Vec2(c.x + shape.radius(), c.y + shape.radius());
    return box;
  }
  Vec2 lo = mul(xf, shape.vertex(0));
  Vec2 hi = lo;
  for (int i = 1; i < shape.vertexCount(); ++i) {
    Vec2 v = mul(xf, shape.vertex(i));
    lo = Vec2(minReal(lo.x, v.x), minReal(lo.y, v.y));
    hi = Vec2(maxReal(hi.x, v.x), maxReal(hi.y, v.y));
  }
  box.lower = lo;
  box.upper = hi;
  return box;
}

std::vector<std::pair<std::uint32_t, std::uint32_t>> findOverlappingPairs(std::vector<BroadphaseEntry> entries) {
  std::sort(entries.begin(), entries.end(), [](const BroadphaseEntry& a, const BroadphaseEntry& b) {
    if (a.box.lower.x != b.box.lower.x) {
      return a.box.lower.x < b.box.lower.x;
    }
    return a.id < b.id;
  });
  std::vector<std::pair<std::uint32_t, std::uint32_t>> pairs;
  for (std::size_t i = 0; i < entries.size(); ++i) {
    for (std::size_t j = i + 1; j < entries.size(); ++j) {
      if (entries[j].box.lower.x > entries[i].box.upper.x) {
        break;
      }
      if (overlaps(entries[i].box, entries[j].box)) {
        pairs.push_back(std::make_pair(std::min(entries[i].id, entries[j].id), std::max(entries[i].id, entries[j].id)));
      }
    }
  }
  std::sort(pairs.begin(), pairs.end());
  return pairs;
}

// ---- world manifold --------------------------------------------------------------

WorldManifold computeWorldManifold(const Manifold& m, const Shape& shapeA, const Transform& xfA, const Shape& shapeB,
                                   const Transform& xfB) {
  WorldManifold w;
  if (m.pointCount == 0) {
    return w;
  }
  real radiusA = shapeA.radius();
  real radiusB = shapeB.radius();
  if (shapeA.type() == ShapeType::Polygon) {
    radiusA = 0;
  }
  if (shapeB.type() == ShapeType::Polygon) {
    radiusB = 0;
  }
  if (m.type == ManifoldType::Circles) {
    Vec2 pointA = mul(xfA, m.localPoint);
    Vec2 pointB = mul(xfB, m.points[0].localPoint);
    w.normal = Vec2(1, 0);
    Vec2 d = pointB - pointA;
    if (d.lengthSquared() > kEpsilon * kEpsilon) {
      w.normal = normalized(d);
    }
    Vec2 cA = pointA + radiusA * w.normal;
    Vec2 cB = pointB - radiusB * w.normal;
    w.points[0] = 0.5 * (cA + cB);
    w.separations[0] = dot(cB - cA, w.normal);
    return w;
  }
  if (m.type == ManifoldType::FaceA) {
    w.normal = mul(xfA.q, m.localNormal);
    Vec2 planePoint = mul(xfA, m.localPoint);
    for (int i = 0; i < m.pointCount; ++i) {
      Vec2 clipPoint = mul(xfB, m.points[i].localPoint);
      Vec2 cA = clipPoint + (radiusA - dot(clipPoint - planePoint, w.normal)) * w.normal;
      Vec2 cB = clipPoint - radiusB * w.normal;
      w.points[i] = 0.5 * (cA + cB);
      w.separations[i] = dot(cB - cA, w.normal);
    }
    return w;
  }
  Vec2 normal = mul(xfB.q, m.localNormal);
  Vec2 planePoint = mul(xfB, m.localPoint);
  for (int i = 0; i < m.pointCount; ++i) {
    Vec2 clipPoint = mul(xfA, m.points[i].localPoint);
    Vec2 cB = clipPoint + (radiusB - dot(clipPoint - planePoint, normal)) * normal;
    Vec2 cA = clipPoint - radiusA * normal;
    w.points[i] = 0.5 * (cA + cB);
    w.separations[i] = dot(cA - cB, normal);
  }
  w.normal = -normal;
  return w;
}

// ---- narrowphase -----------------------------------------------------------------

Manifold collideCircles(const Shape& a, const Transform& xfA, const Shape& b, const Transform& xfB) {
  Manifold m;
  Vec2 pA = mul(xfA, a.center());
  Vec2 pB = mul(xfB, b.center());
  real radius = a.radius() + b.radius();
  if ((pB - pA).lengthSquared() > radius * radius) {
    return m;
  }
  m.type = ManifoldType::Circles;
  m.localPoint = a.center();
  m.localNormal = Vec2(0, 0);
  m.pointCount = 1;
  m.points[0].localPoint = b.center();
  m.points[0].id = 0;
  return m;
}

Manifold collidePolygonAndCircle(const Shape& polygon, const Transform& xfA, const Shape& circle,
                                 const Transform& xfB) {
  Manifold m;
  Vec2 c = mul(xfB, circle.center());
  Vec2 cLocal = mulT(xfA, c);
  real radius = circle.radius();
  int count = polygon.vertexCount();

  int normalIndex = 0;
  real separation = -1e300;
  for (int i = 0; i < count; ++i) {
    real s = dot(polygon.normal(i), cLocal - polygon.vertex(i));
    if (s > radius) {
      return m;
    }
    if (s > separation) {
      separation = s;
      normalIndex = i;
    }
  }

  Vec2 v1 = polygon.vertex(normalIndex);
  Vec2 v2 = polygon.vertex((normalIndex + 1) % count);
  m.type = ManifoldType::FaceA;
  m.pointCount = 1;
  m.points[0].localPoint = circle.center();
  m.points[0].id = 0;

  if (separation < kEpsilon) {
    // Centre inside the polygon: push out through the closest face.
    m.localNormal = polygon.normal(normalIndex);
    m.localPoint = 0.5 * (v1 + v2);
    return m;
  }

  real u1 = dot(cLocal - v1, v2 - v1);
  real u2 = dot(cLocal - v2, v1 - v2);
  if (u1 <= 0.0) {
    if ((cLocal - v1).lengthSquared() > radius * radius) {
      m.pointCount = 0;
      return m;
    }
    m.localNormal = normalized(cLocal - v1);
    m.localPoint = v1;
  } else if (u2 <= 0.0) {
    if ((cLocal - v2).lengthSquared() > radius * radius) {
      m.pointCount = 0;
      return m;
    }
    m.localNormal = normalized(cLocal - v2);
    m.localPoint = v2;
  } else {
    Vec2 faceCenter = 0.5 * (v1 + v2);
    if (dot(cLocal - faceCenter, polygon.normal(normalIndex)) > radius) {
      m.pointCount = 0;
      return m;
    }
    m.localNormal = polygon.normal(normalIndex);
    m.localPoint = faceCenter;
  }
  return m;
}

Manifold collidePolygons(const Shape& a, const Transform& xfA, const Shape& b, const Transform& xfB) {
  Manifold m;
  int edgeA = 0;
  real separationA = findMaxSeparation(edgeA, a, xfA, b, xfB);
  if (separationA > 0.0) {
    return m;
  }
  int edgeB = 0;
  real separationB = findMaxSeparation(edgeB, b, xfB, a, xfA);
  if (separationB > 0.0) {
    return m;
  }

  const Shape* poly1 = &a;
  const Shape* poly2 = &b;
  Transform xf1 = xfA;
  Transform xf2 = xfB;
  int edge1 = edgeA;
  bool flip = false;
  // Prefer A's face when the two are nearly equal, for temporal coherence.
  const real kTol = 0.1 * 0.005;
  if (separationB > separationA + kTol) {
    poly1 = &b;
    poly2 = &a;
    xf1 = xfB;
    xf2 = xfA;
    edge1 = edgeB;
    m.type = ManifoldType::FaceB;
    flip = true;
  } else {
    m.type = ManifoldType::FaceA;
  }

  ClipVertex incident[2];
  findIncidentEdge(incident, *poly1, xf1, edge1, *poly2, xf2);

  int iv1 = edge1;
  int iv2 = (edge1 + 1) % poly1->vertexCount();
  Vec2 v11 = poly1->vertex(iv1);
  Vec2 v12 = poly1->vertex(iv2);
  Vec2 localTangent = normalized(v12 - v11);
  Vec2 localNormal = cross(localTangent, 1.0);
  Vec2 planePoint = 0.5 * (v11 + v12);

  Vec2 tangent = mul(xf1.q, localTangent);
  Vec2 normal = cross(tangent, 1.0);
  v11 = mul(xf1, v11);
  v12 = mul(xf1, v12);

  real frontOffset = dot(normal, v11);
  real sideOffset1 = -dot(tangent, v11);
  real sideOffset2 = dot(tangent, v12);

  ClipVertex clip1[2];
  ClipVertex clip2[2];
  int np = clipSegmentToLine(clip1, incident, -tangent, sideOffset1, static_cast<std::uint32_t>(iv1));
  if (np < 2) {
    return m;
  }
  np = clipSegmentToLine(clip2, clip1, tangent, sideOffset2, static_cast<std::uint32_t>(iv2));
  if (np < 2) {
    return m;
  }

  m.localNormal = localNormal;
  m.localPoint = planePoint;
  int count = 0;
  for (int i = 0; i < 2; ++i) {
    real separation = dot(normal, clip2[i].v) - frontOffset;
    if (separation <= 0.0) {
      ManifoldPoint& cp = m.points[count];
      cp.localPoint = mulT(xf2, clip2[i].v);
      if (flip) {
        cp.id = packId(clip2[i].indexB, clip2[i].indexA, clip2[i].typeB, clip2[i].typeA);
      } else {
        cp.id = packId(clip2[i].indexA, clip2[i].indexB, clip2[i].typeA, clip2[i].typeB);
      }
      ++count;
    }
  }
  m.pointCount = count;
  return m;
}

Manifold collide(const Shape& a, const Transform& xfA, const Shape& b, const Transform& xfB) {
  if (a.type() == ShapeType::Circle && b.type() == ShapeType::Circle) {
    return collideCircles(a, xfA, b, xfB);
  }
  if (a.type() == ShapeType::Polygon && b.type() == ShapeType::Circle) {
    return collidePolygonAndCircle(a, xfA, b, xfB);
  }
  if (a.type() == ShapeType::Polygon && b.type() == ShapeType::Polygon) {
    return collidePolygons(a, xfA, b, xfB);
  }
  return Manifold();
}

}  // namespace physics

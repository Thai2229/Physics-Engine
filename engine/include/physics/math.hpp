#pragma once

#include <cmath>

namespace physics {

using real = double;

constexpr real kPi = 3.14159265358979323846;
constexpr real kEpsilon = 1e-12;

inline real minReal(real a, real b) {
  if (a < b) {
    return a;
  }
  return b;
}

inline real maxReal(real a, real b) {
  if (a > b) {
    return a;
  }
  return b;
}

inline real clamp(real v, real lo, real hi) {
  return maxReal(lo, minReal(v, hi));
}

struct Vec2 {
  real x = 0;
  real y = 0;

  constexpr Vec2() = default;
  constexpr Vec2(real x_, real y_) : x(x_), y(y_) {}

  Vec2 operator-() const { return Vec2(-x, -y); }
  Vec2& operator+=(const Vec2& o) {
    x += o.x;
    y += o.y;
    return *this;
  }
  Vec2& operator-=(const Vec2& o) {
    x -= o.x;
    y -= o.y;
    return *this;
  }
  Vec2& operator*=(real s) {
    x *= s;
    y *= s;
    return *this;
  }

  real lengthSquared() const { return x * x + y * y; }
  real length() const { return std::sqrt(lengthSquared()); }

  /// Normalises in place and returns the previous length. Vectors shorter than
  /// kEpsilon are set to zero and 0 is returned.
  real normalize() {
    real len = length();
    if (len < kEpsilon) {
      x = 0;
      y = 0;
      return 0;
    }
    real inv = 1.0 / len;
    x *= inv;
    y *= inv;
    return len;
  }
};

inline Vec2 operator+(Vec2 a, const Vec2& b) { return a += b; }
inline Vec2 operator-(Vec2 a, const Vec2& b) { return a -= b; }
inline Vec2 operator*(Vec2 a, real s) { return a *= s; }
inline Vec2 operator*(real s, Vec2 a) { return a *= s; }
inline bool operator==(const Vec2& a, const Vec2& b) { return a.x == b.x && a.y == b.y; }
inline bool operator!=(const Vec2& a, const Vec2& b) { return !(a == b); }

inline real dot(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }
inline real cross(const Vec2& a, const Vec2& b) { return a.x * b.y - a.y * b.x; }
inline Vec2 cross(real s, const Vec2& a) { return Vec2(-s * a.y, s * a.x); }
inline Vec2 cross(const Vec2& a, real s) { return Vec2(s * a.y, -s * a.x); }
inline real distance(const Vec2& a, const Vec2& b) { return (a - b).length(); }
inline Vec2 normalized(Vec2 a) {
  a.normalize();
  return a;
}

/// 2x2 matrix stored by columns.
struct Mat22 {
  Vec2 ex;
  Vec2 ey;

  Mat22() = default;
  Mat22(const Vec2& c1, const Vec2& c2) : ex(c1), ey(c2) {}
  Mat22(real a11, real a12, real a21, real a22) : ex(a11, a21), ey(a12, a22) {}

  Mat22 inverse() const {
    real a = ex.x;
    real b = ey.x;
    real c = ex.y;
    real d = ey.y;
    real det = a * d - b * c;
    if (det != 0.0) {
      det = 1.0 / det;
    }
    Mat22 m;
    m.ex = Vec2(det * d, -det * c);
    m.ey = Vec2(-det * b, det * a);
    return m;
  }

  /// Solves A x = b. Returns zero for singular matrices.
  Vec2 solve(const Vec2& rhs) const {
    real a11 = ex.x;
    real a12 = ey.x;
    real a21 = ex.y;
    real a22 = ey.y;
    real det = a11 * a22 - a12 * a21;
    if (det != 0.0) {
      det = 1.0 / det;
    }
    return Vec2(det * (a22 * rhs.x - a12 * rhs.y), det * (a11 * rhs.y - a21 * rhs.x));
  }
};

inline Vec2 mul(const Mat22& m, const Vec2& v) {
  return Vec2(m.ex.x * v.x + m.ey.x * v.y, m.ex.y * v.x + m.ey.y * v.y);
}

/// Rotation stored as sine/cosine.
struct Rot {
  real s = 0;
  real c = 1;

  Rot() = default;
  explicit Rot(real angle) : s(std::sin(angle)), c(std::cos(angle)) {}

  real angle() const { return std::atan2(s, c); }
};

inline Vec2 mul(const Rot& q, const Vec2& v) { return Vec2(q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y); }
inline Vec2 mulT(const Rot& q, const Vec2& v) { return Vec2(q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y); }

struct Transform {
  Vec2 p;
  Rot q;

  Transform() = default;
  Transform(const Vec2& position, const Rot& rotation) : p(position), q(rotation) {}
};

inline Vec2 mul(const Transform& t, const Vec2& v) { return mul(t.q, v) + t.p; }
inline Vec2 mulT(const Transform& t, const Vec2& v) { return mulT(t.q, v - t.p); }

}  // namespace physics

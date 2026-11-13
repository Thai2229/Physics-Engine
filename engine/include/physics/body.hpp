#pragma once

#include <vector>

#include "math.hpp"
#include "shape.hpp"

namespace physics {

enum class BodyType { Static, Kinematic, Dynamic };

struct BodyDef {
  BodyType type = BodyType::Dynamic;
  Vec2 position;  ///< body origin
  real angle = 0;
  Vec2 linearVelocity;  ///< of the centre of mass
  real angularVelocity = 0;
  real linearDamping = 0;
  real angularDamping = 0;
  real gravityScale = 1.0;
};

class RigidBody {
 public:
  explicit RigidBody(const BodyDef& def);

  /// Attaches a fixture and recomputes mass properties. Returns false (and
  /// leaves the body unchanged) when the shape is invalid.
  bool addShape(const Fixture& fixture);
  const std::vector<Fixture>& shapes() const { return fixtures_; }

  BodyType type() const { return type_; }
  Vec2 position() const { return position_; }
  real angle() const { return angle_; }
  Vec2 center() const { return center_; }
  Vec2 localCenter() const { return localCenter_; }
  Vec2 linearVelocity() const { return linearVelocity_; }
  real angularVelocity() const { return angularVelocity_; }
  real mass() const { return mass_; }
  real invMass() const { return invMass_; }
  real inertia() const { return inertia_; }  ///< about the centre of mass
  real invInertia() const { return invInertia_; }
  Vec2 force() const { return force_; }
  real torque() const { return torque_; }
  real linearDamping() const { return linearDamping_; }
  real angularDamping() const { return angularDamping_; }
  real gravityScale() const { return gravityScale_; }
  Rot rotation() const { return Rot(angle_); }

  void setTransform(Vec2 position, real angle);
  /// Places the centre of mass directly (used by the solver).
  void setCenterAndAngle(Vec2 center, real angle);
  void setLinearVelocity(Vec2 v);
  void setAngularVelocity(real w);
  void setLinearDamping(real d) { linearDamping_ = d; }
  void setAngularDamping(real d) { angularDamping_ = d; }

  void applyForce(Vec2 f);
  void applyForceAtPoint(Vec2 f, Vec2 worldPoint);
  void applyTorque(real t);
  void applyLinearImpulse(Vec2 impulse, Vec2 worldPoint);
  void clearForces();

  Vec2 localToWorld(Vec2 p) const;
  Vec2 worldToLocal(Vec2 p) const;
  Vec2 velocityAtWorldPoint(Vec2 p) const;

 private:
  void resetMass();
  void syncCenter();

  BodyType type_;
  Vec2 position_;
  real angle_;
  Vec2 localCenter_;
  Vec2 center_;
  Vec2 linearVelocity_;
  real angularVelocity_;
  Vec2 force_;
  real torque_ = 0;
  real mass_ = 0;
  real invMass_ = 0;
  real inertia_ = 0;
  real invInertia_ = 0;
  real linearDamping_;
  real angularDamping_;
  real gravityScale_;
  std::vector<Fixture> fixtures_;
};

}  // namespace physics

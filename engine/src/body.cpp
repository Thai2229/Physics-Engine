#include <physics/body.hpp>

namespace physics {

RigidBody::RigidBody(const BodyDef& def)
    : type_(def.type),
      position_(def.position),
      angle_(def.angle),
      linearVelocity_(def.linearVelocity),
      angularVelocity_(def.angularVelocity),
      linearDamping_(def.linearDamping),
      angularDamping_(def.angularDamping),
      gravityScale_(def.gravityScale) {
  if (type_ == BodyType::Static) {
    linearVelocity_ = Vec2(0, 0);
    angularVelocity_ = 0;
  }
  resetMass();
  syncCenter();
}

bool RigidBody::addShape(const Fixture& fixture) {
  if (!fixture.shape.isValid()) {
    return false;
  }
  fixtures_.push_back(fixture);
  resetMass();
  return true;
}

void RigidBody::resetMass() {
  mass_ = 0;
  invMass_ = 0;
  inertia_ = 0;
  invInertia_ = 0;
  Vec2 oldCenter = center_;
  localCenter_ = Vec2(0, 0);
  if (type_ != BodyType::Dynamic) {
    syncCenter();
    return;
  }
  Vec2 weighted(0, 0);
  real inertiaOrigin = 0;
  for (const Fixture& f : fixtures_) {
    if (f.density <= 0.0) {
      continue;
    }
    MassData md = f.shape.computeMass(f.density);
    mass_ += md.mass;
    weighted += md.mass * md.center;
    inertiaOrigin += md.inertia;
  }
  if (mass_ > 0.0) {
    invMass_ = 1.0 / mass_;
    localCenter_ = invMass_ * weighted;
    inertia_ = inertiaOrigin - mass_ * dot(localCenter_, localCenter_);
    if (inertia_ > 0.0) {
      invInertia_ = 1.0 / inertia_;
    } else {
      inertia_ = 0;
    }
  } else {
    mass_ = 1.0;
    invMass_ = 1.0;
  }
  syncCenter();
  // Keep the centre-of-mass velocity consistent with the new centre.
  linearVelocity_ += cross(angularVelocity_, center_ - oldCenter);
}

void RigidBody::syncCenter() {
  center_ = position_ + mul(Rot(angle_), localCenter_);
}

void RigidBody::setTransform(Vec2 position, real angle) {
  position_ = position;
  angle_ = angle;
  syncCenter();
}

void RigidBody::setCenterAndAngle(Vec2 center, real angle) {
  center_ = center;
  angle_ = angle;
  position_ = center - mul(Rot(angle), localCenter_);
}

void RigidBody::setLinearVelocity(Vec2 v) {
  if (type_ == BodyType::Static) {
    return;
  }
  linearVelocity_ = v;
}

void RigidBody::setAngularVelocity(real w) {
  if (type_ == BodyType::Static) {
    return;
  }
  angularVelocity_ = w;
}

void RigidBody::applyForce(Vec2 f) {
  if (type_ != BodyType::Dynamic) {
    return;
  }
  force_ += f;
}

void RigidBody::applyForceAtPoint(Vec2 f, Vec2 worldPoint) {
  if (type_ != BodyType::Dynamic) {
    return;
  }
  force_ += f;
  torque_ += cross(worldPoint - center_, f);
}

void RigidBody::applyTorque(real t) {
  if (type_ != BodyType::Dynamic) {
    return;
  }
  torque_ += t;
}

void RigidBody::applyLinearImpulse(Vec2 impulse, Vec2 worldPoint) {
  if (type_ != BodyType::Dynamic) {
    return;
  }
  linearVelocity_ += invMass_ * impulse;
  angularVelocity_ += invInertia_ * cross(worldPoint - center_, impulse);
}

void RigidBody::clearForces() {
  force_ = Vec2(0, 0);
  torque_ = 0;
}

Vec2 RigidBody::localToWorld(Vec2 p) const {
  return mul(Transform(position_, Rot(angle_)), p);
}

Vec2 RigidBody::worldToLocal(Vec2 p) const {
  return mulT(Transform(position_, Rot(angle_)), p);
}

Vec2 RigidBody::velocityAtWorldPoint(Vec2 p) const {
  return linearVelocity_ + cross(angularVelocity_, p - center_);
}

}  // namespace physics

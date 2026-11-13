#include <physics/joint.hpp>

namespace physics {

namespace {

/// Soft-constraint coefficients (Box2D style): gamma, bias and effective mass
/// for a spring of the given frequency acting on a constraint of mass m.
struct Soft {
  real gamma = 0;
  real bias = 0;
};

Soft makeSoft(real m, real hertz, real dampingRatio, real C, real dt) {
  real omega = 2.0 * kPi * hertz;
  real d = 2.0 * m * dampingRatio * omega;
  real k = m * omega * omega;
  Soft s;
  s.gamma = dt * (d + dt * k);
  if (s.gamma != 0.0) {
    s.gamma = 1.0 / s.gamma;
  }
  s.bias = C * dt * k * s.gamma;
  return s;
}

}  // namespace

// ============================================================================
// RevoluteJoint
// ============================================================================

RevoluteJoint::RevoluteJoint(const RevoluteJointDef& def, RigidBody* a, RigidBody* b)
    : Joint(JointType::Revolute, a, b, def.collideConnected),
      localAnchorA_(a->worldToLocal(def.anchor)),
      localAnchorB_(b->worldToLocal(def.anchor)),
      referenceAngle_(b->angle() - a->angle()),
      enableLimit_(def.enableLimit),
      lowerAngle_(minReal(def.lowerAngle, def.upperAngle)),
      upperAngle_(maxReal(def.lowerAngle, def.upperAngle)),
      enableMotor_(def.enableMotor),
      motorSpeed_(def.motorSpeed),
      maxMotorTorque_(def.maxMotorTorque),
      enableSpring_(def.enableSpring),
      springHertz_(def.springHertz),
      springDampingRatio_(def.springDampingRatio),
      springRestAngle_(def.springRestAngle) {}

Vec2 RevoluteJoint::anchorA() const {
  return bodyA()->localToWorld(localAnchorA_);
}

Vec2 RevoluteJoint::anchorB() const {
  return bodyB()->localToWorld(localAnchorB_);
}

real RevoluteJoint::jointAngle() const {
  return bodyB()->angle() - bodyA()->angle() - referenceAngle_;
}

real RevoluteJoint::jointSpeed() const {
  return bodyB()->angularVelocity() - bodyA()->angularVelocity();
}

void RevoluteJoint::setLimitEnabled(bool enabled) {
  if (enabled != enableLimit_) {
    enableLimit_ = enabled;
    lowerImpulse_ = 0;
    upperImpulse_ = 0;
  }
}

void RevoluteJoint::setLimits(real lower, real upper) {
  lowerAngle_ = minReal(lower, upper);
  upperAngle_ = maxReal(lower, upper);
  lowerImpulse_ = 0;
  upperImpulse_ = 0;
}

void RevoluteJoint::setMotorEnabled(bool enabled) {
  if (enabled != enableMotor_) {
    enableMotor_ = enabled;
    motorImpulse_ = 0;
  }
}

void RevoluteJoint::setSpring(bool enabled, real hertz, real dampingRatio) {
  if (enabled != enableSpring_) {
    springImpulse_ = 0;
  }
  enableSpring_ = enabled;
  springHertz_ = hertz;
  springDampingRatio_ = dampingRatio;
}

void RevoluteJoint::initVelocityConstraints(const StepContext& ctx) {
  RigidBody* a = bodyA();
  RigidBody* b = bodyB();
  mA_ = a->invMass();
  mB_ = b->invMass();
  iA_ = a->invInertia();
  iB_ = b->invInertia();
  rA_ = mul(Rot(a->angle()), localAnchorA_ - a->localCenter());
  rB_ = mul(Rot(b->angle()), localAnchorB_ - b->localCenter());

  K_.ex.x = mA_ + mB_ + rA_.y * rA_.y * iA_ + rB_.y * rB_.y * iB_;
  K_.ey.x = -rA_.y * rA_.x * iA_ - rB_.y * rB_.x * iB_;
  K_.ex.y = K_.ey.x;
  K_.ey.y = mA_ + mB_ + rA_.x * rA_.x * iA_ + rB_.x * rB_.x * iB_;

  real axialInv = iA_ + iB_;
  axialMass_ = 0;
  if (axialInv > 0.0) {
    axialMass_ = 1.0 / axialInv;
  }
  angle_ = jointAngle();

  springGamma_ = 0;
  springBias_ = 0;
  springMass_ = 0;
  if (enableSpring_ && springHertz_ > 0.0 && axialInv > 0.0) {
    Soft s = makeSoft(axialMass_, springHertz_, springDampingRatio_, angle_ - springRestAngle_, ctx.dt);
    springGamma_ = s.gamma;
    springBias_ = s.bias;
    springMass_ = 1.0 / (axialInv + s.gamma);
  } else {
    springImpulse_ = 0;
  }
  if (!enableMotor_) {
    motorImpulse_ = 0;
  }
  if (!enableLimit_) {
    lowerImpulse_ = 0;
    upperImpulse_ = 0;
  }

  if (ctx.warmStarting) {
    impulse_ *= ctx.dtRatio;
    springImpulse_ *= ctx.dtRatio;
    motorImpulse_ *= ctx.dtRatio;
    lowerImpulse_ *= ctx.dtRatio;
    upperImpulse_ *= ctx.dtRatio;
    real axialImpulse = springImpulse_ + motorImpulse_ + lowerImpulse_ - upperImpulse_;
    Vec2 P = impulse_;
    a->setLinearVelocity(a->linearVelocity() - mA_ * P);
    a->setAngularVelocity(a->angularVelocity() - iA_ * (cross(rA_, P) + axialImpulse));
    b->setLinearVelocity(b->linearVelocity() + mB_ * P);
    b->setAngularVelocity(b->angularVelocity() + iB_ * (cross(rB_, P) + axialImpulse));
  } else {
    impulse_ = Vec2(0, 0);
    springImpulse_ = 0;
    motorImpulse_ = 0;
    lowerImpulse_ = 0;
    upperImpulse_ = 0;
  }
}

void RevoluteJoint::solveVelocityConstraints(const StepContext& ctx) {
  RigidBody* a = bodyA();
  RigidBody* b = bodyB();
  Vec2 vA = a->linearVelocity();
  Vec2 vB = b->linearVelocity();
  real wA = a->angularVelocity();
  real wB = b->angularVelocity();
  bool fixedRotation = (iA_ + iB_ == 0.0);

  if (enableSpring_ && springMass_ > 0.0 && !fixedRotation) {
    real Cdot = wB - wA;
    real impulse = -springMass_ * (Cdot + springBias_ + springGamma_ * springImpulse_);
    springImpulse_ += impulse;
    wA -= iA_ * impulse;
    wB += iB_ * impulse;
  }

  if (enableMotor_ && !fixedRotation) {
    real Cdot = wB - wA - motorSpeed_;
    real impulse = -axialMass_ * Cdot;
    real maxImpulse = ctx.dt * maxMotorTorque_;
    real newImpulse = clamp(motorImpulse_ + impulse, -maxImpulse, maxImpulse);
    impulse = newImpulse - motorImpulse_;
    motorImpulse_ = newImpulse;
    wA -= iA_ * impulse;
    wB += iB_ * impulse;
  }

  if (enableLimit_ && !fixedRotation) {
    {
      real C = angle_ - lowerAngle_;
      real Cdot = wB - wA;
      real impulse = -axialMass_ * (Cdot + maxReal(C, 0.0) * ctx.invDt);
      real newImpulse = maxReal(lowerImpulse_ + impulse, 0.0);
      impulse = newImpulse - lowerImpulse_;
      lowerImpulse_ = newImpulse;
      wA -= iA_ * impulse;
      wB += iB_ * impulse;
    }
    {
      real C = upperAngle_ - angle_;
      real Cdot = wA - wB;
      real impulse = -axialMass_ * (Cdot + maxReal(C, 0.0) * ctx.invDt);
      real newImpulse = maxReal(upperImpulse_ + impulse, 0.0);
      impulse = newImpulse - upperImpulse_;
      upperImpulse_ = newImpulse;
      wA += iA_ * impulse;
      wB -= iB_ * impulse;
    }
  }

  {
    Vec2 Cdot = vB + cross(wB, rB_) - vA - cross(wA, rA_);
    // Second-order term: the anchors follow arcs, not tangent lines, over a step.
    // Targeting the post-integration anchor gap (instead of the instantaneous
    // anchor velocity) keeps pendulums from bleeding energy every step.
    Vec2 arc = 0.5 * ctx.dt * (wB * wB * rB_ - wA * wA * rA_);
    Vec2 impulse = K_.solve(-(Cdot - arc));
    impulse_ += impulse;
    vA -= mA_ * impulse;
    wA -= iA_ * cross(rA_, impulse);
    vB += mB_ * impulse;
    wB += iB_ * cross(rB_, impulse);
  }

  a->setLinearVelocity(vA);
  a->setAngularVelocity(wA);
  b->setLinearVelocity(vB);
  b->setAngularVelocity(wB);
}

bool RevoluteJoint::solvePositionConstraints() {
  RigidBody* a = bodyA();
  RigidBody* b = bodyB();
  Vec2 cA = a->center();
  Vec2 cB = b->center();
  real aA = a->angle();
  real aB = b->angle();
  bool fixedRotation = (iA_ + iB_ == 0.0);
  real angularError = 0;

  if (enableLimit_ && !fixedRotation) {
    real angle = aB - aA - referenceAngle_;
    real C = 0;
    if (lowerAngle_ == upperAngle_) {
      C = clamp(angle - lowerAngle_, -kMaxAngularCorrection, kMaxAngularCorrection);
    } else if (angle <= lowerAngle_) {
      C = clamp(angle - lowerAngle_ + kAngularSlop, -kMaxAngularCorrection, 0.0);
    } else if (angle >= upperAngle_) {
      C = clamp(angle - upperAngle_ - kAngularSlop, 0.0, kMaxAngularCorrection);
    }
    real limitImpulse = -axialMass_ * C;
    aA -= iA_ * limitImpulse;
    aB += iB_ * limitImpulse;
    angularError = std::fabs(C);
  }

  real positionError = 0;
  {
    Vec2 rA = mul(Rot(aA), localAnchorA_ - a->localCenter());
    Vec2 rB = mul(Rot(aB), localAnchorB_ - b->localCenter());
    Vec2 C = cB + rB - cA - rA;
    positionError = C.length();

    Mat22 K;
    K.ex.x = mA_ + mB_ + rA.y * rA.y * iA_ + rB.y * rB.y * iB_;
    K.ey.x = -rA.y * rA.x * iA_ - rB.y * rB.x * iB_;
    K.ex.y = K.ey.x;
    K.ey.y = mA_ + mB_ + rA.x * rA.x * iA_ + rB.x * rB.x * iB_;
    Vec2 impulse = -K.solve(C);

    cA -= mA_ * impulse;
    aA -= iA_ * cross(rA, impulse);
    cB += mB_ * impulse;
    aB += iB_ * cross(rB, impulse);
  }

  a->setCenterAndAngle(cA, aA);
  b->setCenterAndAngle(cB, aB);
  return positionError <= kPositionTolerance && angularError <= kAngularSlop;
}

// ============================================================================
// DistanceJoint
// ============================================================================

DistanceJoint::DistanceJoint(const DistanceJointDef& def, RigidBody* a, RigidBody* b)
    : Joint(JointType::Distance, a, b, def.collideConnected),
      localAnchorA_(a->worldToLocal(def.anchorA)),
      localAnchorB_(b->worldToLocal(def.anchorB)),
      length_(def.length),
      hertz_(def.hertz),
      dampingRatio_(def.dampingRatio) {
  if (length_ < 0.0) {
    length_ = distance(def.anchorA, def.anchorB);
  }
}

Vec2 DistanceJoint::anchorA() const {
  return bodyA()->localToWorld(localAnchorA_);
}

Vec2 DistanceJoint::anchorB() const {
  return bodyB()->localToWorld(localAnchorB_);
}

void DistanceJoint::setLength(real length) {
  if (length >= 0.0) {
    length_ = length;
  }
}

void DistanceJoint::setSpring(real hertz, real dampingRatio) {
  hertz_ = hertz;
  dampingRatio_ = dampingRatio;
  impulse_ = 0;
}

void DistanceJoint::initVelocityConstraints(const StepContext& ctx) {
  RigidBody* a = bodyA();
  RigidBody* b = bodyB();
  mA_ = a->invMass();
  mB_ = b->invMass();
  iA_ = a->invInertia();
  iB_ = b->invInertia();
  rA_ = mul(Rot(a->angle()), localAnchorA_ - a->localCenter());
  rB_ = mul(Rot(b->angle()), localAnchorB_ - b->localCenter());
  u_ = b->center() + rB_ - a->center() - rA_;
  real len = u_.length();
  length0_ = len;
  if (len > kLinearSlop) {
    u_ *= 1.0 / len;
  } else {
    u_ = Vec2(0, 0);
  }

  real crAu = cross(rA_, u_);
  real crBu = cross(rB_, u_);
  real invMass = mA_ + mB_ + iA_ * crAu * crAu + iB_ * crBu * crBu;
  mass_ = 0;
  if (invMass != 0.0) {
    mass_ = 1.0 / invMass;
  }

  if (hertz_ > 0.0) {
    Soft s = makeSoft(mass_, hertz_, dampingRatio_, len - length_, ctx.dt);
    gamma_ = s.gamma;
    bias_ = s.bias;
    softMass_ = 1.0 / (invMass + s.gamma);
  } else {
    gamma_ = 0;
    bias_ = 0;
    softMass_ = mass_;
  }

  if (ctx.warmStarting) {
    impulse_ *= ctx.dtRatio;
    Vec2 P = impulse_ * u_;
    a->setLinearVelocity(a->linearVelocity() - mA_ * P);
    a->setAngularVelocity(a->angularVelocity() - iA_ * cross(rA_, P));
    b->setLinearVelocity(b->linearVelocity() + mB_ * P);
    b->setAngularVelocity(b->angularVelocity() + iB_ * cross(rB_, P));
  } else {
    impulse_ = 0;
  }
}

void DistanceJoint::solveVelocityConstraints(const StepContext& ctx) {
  RigidBody* a = bodyA();
  RigidBody* b = bodyB();
  Vec2 vA = a->linearVelocity();
  Vec2 vB = b->linearVelocity();
  real wA = a->angularVelocity();
  real wB = b->angularVelocity();

  Vec2 vpA = vA + cross(wA, rA_);
  Vec2 vpB = vB + cross(wB, rB_);
  Vec2 dv = vpB - vpA;
  real Cdot = dot(u_, dv);
  if (hertz_ <= 0.0 && length0_ > kLinearSlop) {
    // Second-order arc correction (see RevoluteJoint): keeps the rod length
    // constant after integration, which conserves energy for swinging rods.
    real perp2 = maxReal(dv.lengthSquared() - Cdot * Cdot, 0.0);
    real arc = 0.5 * ctx.dt * dot(u_, wB * wB * rB_ - wA * wA * rA_) - 0.5 * ctx.dt * perp2 / length0_;
    Cdot -= arc;
  }
  real impulse = -softMass_ * (Cdot + bias_ + gamma_ * impulse_);
  impulse_ += impulse;
  Vec2 P = impulse * u_;
  vA -= mA_ * P;
  wA -= iA_ * cross(rA_, P);
  vB += mB_ * P;
  wB += iB_ * cross(rB_, P);

  a->setLinearVelocity(vA);
  a->setAngularVelocity(wA);
  b->setLinearVelocity(vB);
  b->setAngularVelocity(wB);
}

bool DistanceJoint::solvePositionConstraints() {
  if (hertz_ > 0.0) {
    return true;  // soft joints are allowed to stretch
  }
  RigidBody* a = bodyA();
  RigidBody* b = bodyB();
  Vec2 cA = a->center();
  Vec2 cB = b->center();
  real aA = a->angle();
  real aB = b->angle();

  Vec2 rA = mul(Rot(aA), localAnchorA_ - a->localCenter());
  Vec2 rB = mul(Rot(aB), localAnchorB_ - b->localCenter());
  Vec2 u = cB + rB - cA - rA;
  real len = u.normalize();
  real C = clamp(len - length_, -kMaxLinearCorrection, kMaxLinearCorrection);

  real crAu = cross(rA, u);
  real crBu = cross(rB, u);
  real invMass = mA_ + mB_ + iA_ * crAu * crAu + iB_ * crBu * crBu;
  real impulse = 0;
  if (invMass != 0.0) {
    impulse = -C / invMass;
  }
  Vec2 P = impulse * u;
  cA -= mA_ * P;
  aA -= iA_ * cross(rA, P);
  cB += mB_ * P;
  aB += iB_ * cross(rB, P);

  a->setCenterAndAngle(cA, aA);
  b->setCenterAndAngle(cB, aB);
  return std::fabs(C) < kPositionTolerance;
}

// ============================================================================
// MouseJoint
// ============================================================================

MouseJoint::MouseJoint(RigidBody* body, Vec2 worldPoint, real maxForce, real hertz, real dampingRatio)
    : Joint(JointType::Mouse, nullptr, body, false),
      target_(worldPoint),
      localAnchorB_(body->worldToLocal(worldPoint)),
      maxForce_(maxForce),
      hertz_(hertz),
      dampingRatio_(dampingRatio) {}

Vec2 MouseJoint::anchorB() const {
  return bodyB()->localToWorld(localAnchorB_);
}

void MouseJoint::initVelocityConstraints(const StepContext& ctx) {
  RigidBody* b = bodyB();
  mB_ = b->invMass();
  iB_ = b->invInertia();
  rB_ = mul(Rot(b->angle()), localAnchorB_ - b->localCenter());

  real mass = b->mass();
  real omega = 2.0 * kPi * hertz_;
  real d = 2.0 * mass * dampingRatio_ * omega;
  real k = mass * omega * omega;
  gamma_ = ctx.dt * (d + ctx.dt * k);
  if (gamma_ != 0.0) {
    gamma_ = 1.0 / gamma_;
  }
  real beta = ctx.dt * k * gamma_;

  Mat22 K;
  K.ex.x = mB_ + iB_ * rB_.y * rB_.y + gamma_;
  K.ey.x = -iB_ * rB_.x * rB_.y;
  K.ex.y = K.ey.x;
  K.ey.y = mB_ + iB_ * rB_.x * rB_.x + gamma_;
  mass_ = K.inverse();

  C_ = b->center() + rB_ - target_;
  C_ *= beta;

  if (ctx.warmStarting) {
    impulse_ *= ctx.dtRatio;
    b->setLinearVelocity(b->linearVelocity() + mB_ * impulse_);
    b->setAngularVelocity(b->angularVelocity() + iB_ * cross(rB_, impulse_));
  } else {
    impulse_ = Vec2(0, 0);
  }
}

void MouseJoint::solveVelocityConstraints(const StepContext& ctx) {
  RigidBody* b = bodyB();
  Vec2 vB = b->linearVelocity();
  real wB = b->angularVelocity();

  Vec2 Cdot = vB + cross(wB, rB_);
  Vec2 impulse = mul(mass_, -(Cdot + C_ + gamma_ * impulse_));

  Vec2 oldImpulse = impulse_;
  impulse_ += impulse;
  real maxImpulse = ctx.dt * maxForce_;
  if (impulse_.lengthSquared() > maxImpulse * maxImpulse) {
    impulse_ *= maxImpulse / impulse_.length();
  }
  impulse = impulse_ - oldImpulse;

  vB += mB_ * impulse;
  wB += iB_ * cross(rB_, impulse);
  b->setLinearVelocity(vB);
  b->setAngularVelocity(wB);
}

}  // namespace physics

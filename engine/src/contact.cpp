#include <physics/contact.hpp>

namespace physics {

namespace {

Transform transformOf(const RigidBody& b) {
  return Transform(b.position(), b.rotation());
}

}  // namespace

Contact::Contact(BodyId idA, RigidBody* a, int fixtureA, BodyId idB, RigidBody* b, int fixtureB)
    : idA_(idA), idB_(idB), bodyA_(a), bodyB_(b), fixtureA_(fixtureA), fixtureB_(fixtureB) {
  const Fixture& fa = a->shapes()[static_cast<std::size_t>(fixtureA)];
  const Fixture& fb = b->shapes()[static_cast<std::size_t>(fixtureB)];
  friction_ = std::sqrt(fa.friction * fb.friction);
  restitution_ = maxReal(fa.restitution, fb.restitution);
}

WorldManifold Contact::worldManifold() const {
  return computeWorldManifold(manifold_, fixtureA().shape, transformOf(*bodyA_), fixtureB().shape,
                              transformOf(*bodyB_));
}

void Contact::initVelocityConstraints(const StepContext& ctx) {
  WorldManifold wm = worldManifold();
  normal_ = wm.normal;
  Vec2 tangent = cross(normal_, 1.0);
  real mA = bodyA_->invMass();
  real mB = bodyB_->invMass();
  real iA = bodyA_->invInertia();
  real iB = bodyB_->invInertia();

  for (int i = 0; i < manifold_.pointCount; ++i) {
    ManifoldPoint& mp = manifold_.points[i];
    Point& p = points_[i];
    p.rA = wm.points[i] - bodyA_->center();
    p.rB = wm.points[i] - bodyB_->center();

    real rnA = cross(p.rA, normal_);
    real rnB = cross(p.rB, normal_);
    real kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
    p.normalMass = 0;
    if (kNormal > 0.0) {
      p.normalMass = 1.0 / kNormal;
    }
    real rtA = cross(p.rA, tangent);
    real rtB = cross(p.rB, tangent);
    real kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
    p.tangentMass = 0;
    if (kTangent > 0.0) {
      p.tangentMass = 1.0 / kTangent;
    }

    // Restitution: bounce only when closing faster than the threshold.
    p.velocityBias = 0;
    real vRel = dot(normal_, bodyB_->velocityAtWorldPoint(wm.points[i]) - bodyA_->velocityAtWorldPoint(wm.points[i]));
    if (vRel < -kVelocityThreshold) {
      p.velocityBias = -restitution_ * vRel;
    }

    if (ctx.warmStarting) {
      mp.normalImpulse *= ctx.dtRatio;
      mp.tangentImpulse *= ctx.dtRatio;
      Vec2 P = mp.normalImpulse * normal_ + mp.tangentImpulse * tangent;
      bodyA_->setAngularVelocity(bodyA_->angularVelocity() - iA * cross(p.rA, P));
      bodyA_->setLinearVelocity(bodyA_->linearVelocity() - mA * P);
      bodyB_->setAngularVelocity(bodyB_->angularVelocity() + iB * cross(p.rB, P));
      bodyB_->setLinearVelocity(bodyB_->linearVelocity() + mB * P);
    } else {
      mp.normalImpulse = 0;
      mp.tangentImpulse = 0;
    }
  }
}

void Contact::solveVelocityConstraints() {
  Vec2 tangent = cross(normal_, 1.0);
  real mA = bodyA_->invMass();
  real mB = bodyB_->invMass();
  real iA = bodyA_->invInertia();
  real iB = bodyB_->invInertia();
  Vec2 vA = bodyA_->linearVelocity();
  real wA = bodyA_->angularVelocity();
  Vec2 vB = bodyB_->linearVelocity();
  real wB = bodyB_->angularVelocity();

  // Friction first: its limit depends on the normal impulse accumulated so far.
  for (int i = 0; i < manifold_.pointCount; ++i) {
    ManifoldPoint& mp = manifold_.points[i];
    Point& p = points_[i];
    Vec2 dv = vB + cross(wB, p.rB) - vA - cross(wA, p.rA);
    real lambda = -p.tangentMass * dot(dv, tangent);
    real maxFriction = friction_ * mp.normalImpulse;
    real newImpulse = clamp(mp.tangentImpulse + lambda, -maxFriction, maxFriction);
    lambda = newImpulse - mp.tangentImpulse;
    mp.tangentImpulse = newImpulse;
    Vec2 P = lambda * tangent;
    vA -= mA * P;
    wA -= iA * cross(p.rA, P);
    vB += mB * P;
    wB += iB * cross(p.rB, P);
  }

  for (int i = 0; i < manifold_.pointCount; ++i) {
    ManifoldPoint& mp = manifold_.points[i];
    Point& p = points_[i];
    Vec2 dv = vB + cross(wB, p.rB) - vA - cross(wA, p.rA);
    real vn = dot(dv, normal_);
    real lambda = -p.normalMass * (vn - p.velocityBias);
    real newImpulse = maxReal(mp.normalImpulse + lambda, 0.0);
    lambda = newImpulse - mp.normalImpulse;
    mp.normalImpulse = newImpulse;
    Vec2 P = lambda * normal_;
    vA -= mA * P;
    wA -= iA * cross(p.rA, P);
    vB += mB * P;
    wB += iB * cross(p.rB, P);
  }

  bodyA_->setLinearVelocity(vA);
  bodyA_->setAngularVelocity(wA);
  bodyB_->setLinearVelocity(vB);
  bodyB_->setAngularVelocity(wB);
}

real Contact::solvePositionConstraints() {
  real mA = bodyA_->invMass();
  real mB = bodyB_->invMass();
  real iA = bodyA_->invInertia();
  real iB = bodyB_->invInertia();
  real minSeparation = 0;

  // Re-evaluate the manifold after each point so the second point sees the first one's correction.
  for (int i = 0; i < manifold_.pointCount; ++i) {
    WorldManifold wm = worldManifold();
    Vec2 normal = wm.normal;
    Vec2 rA = wm.points[i] - bodyA_->center();
    Vec2 rB = wm.points[i] - bodyB_->center();
    real separation = wm.separations[i];
    minSeparation = minReal(minSeparation, separation);

    real C = clamp(kBaumgarte * (separation + kLinearSlop), -kMaxLinearCorrection, 0.0);
    real rnA = cross(rA, normal);
    real rnB = cross(rB, normal);
    real K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
    real impulse = 0;
    if (K > 0.0) {
      impulse = -C / K;
    }
    Vec2 P = impulse * normal;
    if (mA > 0.0 || iA > 0.0) {
      bodyA_->setCenterAndAngle(bodyA_->center() - mA * P, bodyA_->angle() - iA * cross(rA, P));
    }
    if (mB > 0.0 || iB > 0.0) {
      bodyB_->setCenterAndAngle(bodyB_->center() + mB * P, bodyB_->angle() + iB * cross(rB, P));
    }
  }
  return minSeparation;
}

}  // namespace physics

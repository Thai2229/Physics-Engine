#include <physics/world.hpp>

#include <memory>

namespace physics {

World::World(Vec2 gravity) : gravity_(gravity) {}

// ---- bodies -----------------------------------------------------------------

BodyId World::createBody(const BodyDef& def) {
  return bodies_.insert(std::make_unique<RigidBody>(def));
}

bool World::destroyBody(BodyId id) {
  RigidBody* b = bodies_.get(id);
  if (b == nullptr) {
    return false;
  }
  std::vector<JointId> attached;
  joints_.forEach([&](JointId jid, Joint& j) {
    if (j.bodyA() == b || j.bodyB() == b) {
      attached.push_back(jid);
    }
  });
  for (JointId jid : attached) {
    joints_.remove(jid);
  }
  return bodies_.remove(id);
}

RigidBody* World::body(BodyId id) {
  return bodies_.get(id);
}

const RigidBody* World::body(BodyId id) const {
  return bodies_.get(id);
}

bool World::addShape(BodyId id, const Fixture& fixture) {
  RigidBody* b = bodies_.get(id);
  if (b == nullptr) {
    return false;
  }
  return b->addShape(fixture);
}

bool World::setBodyTransform(BodyId id, Vec2 position, real angle) {
  RigidBody* b = bodies_.get(id);
  if (b == nullptr) {
    return false;
  }
  b->setTransform(position, angle);
  return true;
}

bool World::setBodyVelocity(BodyId id, Vec2 linearVelocity, real angularVelocity) {
  RigidBody* b = bodies_.get(id);
  if (b == nullptr) {
    return false;
  }
  b->setLinearVelocity(linearVelocity);
  b->setAngularVelocity(angularVelocity);
  return true;
}

// ---- joints -----------------------------------------------------------------

JointId World::addJoint(std::unique_ptr<Joint> joint) {
  return joints_.insert(std::move(joint));
}

JointId World::createRevoluteJoint(const RevoluteJointDef& def) {
  RigidBody* a = bodies_.get(def.bodyA);
  RigidBody* b = bodies_.get(def.bodyB);
  if (a == nullptr || b == nullptr || a == b || (a->invMass() == 0.0 && b->invMass() == 0.0)) {
    return JointId();
  }
  return addJoint(std::make_unique<RevoluteJoint>(def, a, b));
}

JointId World::createDistanceJoint(const DistanceJointDef& def) {
  RigidBody* a = bodies_.get(def.bodyA);
  RigidBody* b = bodies_.get(def.bodyB);
  if (a == nullptr || b == nullptr || a == b || (a->invMass() == 0.0 && b->invMass() == 0.0)) {
    return JointId();
  }
  return addJoint(std::make_unique<DistanceJoint>(def, a, b));
}

JointId World::createMouseJoint(BodyId body, Vec2 worldPoint, real maxForce, real hertz, real dampingRatio) {
  RigidBody* b = bodies_.get(body);
  if (b == nullptr || b->type() != BodyType::Dynamic) {
    return JointId();
  }
  return addJoint(std::make_unique<MouseJoint>(b, worldPoint, maxForce, hertz, dampingRatio));
}

bool World::destroyJoint(JointId id) {
  return joints_.remove(id);
}

Joint* World::joint(JointId id) {
  return joints_.get(id);
}

const Joint* World::joint(JointId id) const {
  return joints_.get(id);
}

RevoluteJoint* World::revoluteJoint(JointId id) {
  Joint* j = joints_.get(id);
  if (j == nullptr || j->type() != JointType::Revolute) {
    return nullptr;
  }
  return static_cast<RevoluteJoint*>(j);
}

DistanceJoint* World::distanceJoint(JointId id) {
  Joint* j = joints_.get(id);
  if (j == nullptr || j->type() != JointType::Distance) {
    return nullptr;
  }
  return static_cast<DistanceJoint*>(j);
}

MouseJoint* World::mouseJoint(JointId id) {
  Joint* j = joints_.get(id);
  if (j == nullptr || j->type() != JointType::Mouse) {
    return nullptr;
  }
  return static_cast<MouseJoint*>(j);
}

bool World::setMouseTarget(JointId id, Vec2 target) {
  MouseJoint* j = mouseJoint(id);
  if (j == nullptr) {
    return false;
  }
  j->setTarget(target);
  return true;
}

// ---- stepping ---------------------------------------------------------------

void World::step(real dt, int velocityIterations, int positionIterations) {
  if (!(dt > 0.0)) {
    return;
  }
  StepContext ctx;
  ctx.dt = dt;
  ctx.invDt = 1.0 / dt;
  ctx.dtRatio = 1.0;
  if (lastDt_ > 0.0) {
    ctx.dtRatio = dt / lastDt_;
  }
  ctx.warmStarting = warmStarting_;
  lastDt_ = dt;

  // 1. integrate velocities (semi-implicit Euler)
  bodies_.forEach([&](BodyId, RigidBody& b) {
    if (b.type() == BodyType::Dynamic) {
      Vec2 v = b.linearVelocity();
      real w = b.angularVelocity();
      v += dt * (b.gravityScale() * gravity_ + b.invMass() * b.force());
      w += dt * b.invInertia() * b.torque();
      v *= 1.0 / (1.0 + dt * b.linearDamping());
      w *= 1.0 / (1.0 + dt * b.angularDamping());
      b.setLinearVelocity(v);
      b.setAngularVelocity(w);
    }
    b.clearForces();
  });

  // 2. constraints: init (+ warm start) and iterate velocities
  joints_.forEach([&](JointId, Joint& j) { j.initVelocityConstraints(ctx); });
  for (int i = 0; i < velocityIterations; ++i) {
    joints_.forEach([&](JointId, Joint& j) { j.solveVelocityConstraints(ctx); });
  }

  // 3. integrate positions
  bodies_.forEach([&](BodyId, RigidBody& b) {
    if (b.type() == BodyType::Static) {
      return;
    }
    b.setCenterAndAngle(b.center() + dt * b.linearVelocity(), b.angle() + dt * b.angularVelocity());
  });

  // 4. position correction
  for (int i = 0; i < positionIterations; ++i) {
    bool allSolved = true;
    joints_.forEach([&](JointId, Joint& j) {
      if (!j.solvePositionConstraints()) {
        allSolved = false;
      }
    });
    if (allSolved) {
      break;
    }
  }
}

void World::setFixedTimeStep(real dt) {
  if (dt > 0.0) {
    fixedTimeStep_ = dt;
  }
}

int World::advance(real frameDt) {
  if (!(frameDt > 0.0)) {
    return 0;
  }
  accumulator_ += minReal(frameDt, maxFrameTime_);
  int steps = 0;
  // The tolerance absorbs floating-point drift when frame times divide the step evenly.
  while (accumulator_ + 1e-9 >= fixedTimeStep_) {
    step(fixedTimeStep_, velocityIterations_, positionIterations_);
    accumulator_ -= fixedTimeStep_;
    ++steps;
  }
  accumulator_ = maxReal(accumulator_, 0.0);
  return steps;
}

// ---- stats & buffers ------------------------------------------------------------

real World::kineticEnergy() const {
  real e = 0;
  bodies_.forEach([&](BodyId, const RigidBody& b) {
    if (b.type() == BodyType::Dynamic) {
      e += 0.5 * b.mass() * b.linearVelocity().lengthSquared();
      e += 0.5 * b.inertia() * b.angularVelocity() * b.angularVelocity();
    }
  });
  return e;
}

real World::potentialEnergy() const {
  real e = 0;
  bodies_.forEach([&](BodyId, const RigidBody& b) {
    if (b.type() == BodyType::Dynamic) {
      e -= b.mass() * b.gravityScale() * dot(gravity_, b.center());
    }
  });
  return e;
}

const std::vector<float>& World::bodyStates() const {
  bodyStates_.clear();
  bodyStates_.reserve(bodies_.size() * 3);
  bodies_.forEach([&](BodyId, const RigidBody& b) {
    bodyStates_.push_back(static_cast<float>(b.position().x));
    bodyStates_.push_back(static_cast<float>(b.position().y));
    bodyStates_.push_back(static_cast<float>(b.angle()));
  });
  return bodyStates_;
}

const std::vector<std::int32_t>& World::bodyIds() const {
  bodyIds_.clear();
  bodyIds_.reserve(bodies_.size());
  bodies_.forEach([&](BodyId id, const RigidBody&) { bodyIds_.push_back(id.toInt()); });
  return bodyIds_;
}

const std::vector<float>& World::jointAnchors() const {
  jointAnchors_.clear();
  jointAnchors_.reserve(joints_.size() * 4);
  joints_.forEach([&](JointId, const Joint& j) {
    Vec2 a = j.anchorA();
    Vec2 b = j.anchorB();
    jointAnchors_.push_back(static_cast<float>(a.x));
    jointAnchors_.push_back(static_cast<float>(a.y));
    jointAnchors_.push_back(static_cast<float>(b.x));
    jointAnchors_.push_back(static_cast<float>(b.y));
  });
  return jointAnchors_;
}

const std::vector<std::int32_t>& World::jointIds() const {
  jointIds_.clear();
  joints_.forEach([&](JointId id, const Joint&) { jointIds_.push_back(id.toInt()); });
  return jointIds_;
}

}  // namespace physics

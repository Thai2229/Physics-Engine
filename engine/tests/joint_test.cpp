#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include <physics/world.hpp>

using namespace physics;

namespace {

constexpr real kDt = 1.0 / 120.0;

BodyId addStatic(World& w, Vec2 p) {
  BodyDef def;
  def.type = BodyType::Static;
  def.position = p;
  return w.createBody(def);
}

BodyId addBox(World& w, Vec2 p, real width, real height, real density = 1.0) {
  BodyDef def;
  def.position = p;
  BodyId id = w.createBody(def);
  Fixture f;
  f.shape = Shape::box(width, height);
  f.density = density;
  w.addShape(id, f);
  return id;
}

BodyId addBob(World& w, Vec2 p, real radius, real density = 1.0) {
  BodyDef def;
  def.position = p;
  BodyId id = w.createBody(def);
  Fixture f;
  f.shape = Shape::circle(radius);
  f.density = density;
  w.addShape(id, f);
  return id;
}

RevoluteJointDef revolute(BodyId a, BodyId b, Vec2 anchor) {
  RevoluteJointDef def;
  def.bodyA = a;
  def.bodyB = b;
  def.anchor = anchor;
  return def;
}

real anchorGap(const Joint* j) {
  return distance(j->anchorA(), j->anchorB());
}

}  // namespace

// ---- RevoluteJoint -----------------------------------------------------------

TEST(RevoluteJoint, PendulumPeriodMatchesTheory) {
  const real L = 1.0;
  const real g = 9.81;
  const real theta0 = 0.1;
  World w(Vec2(0, -g));
  BodyId ground = addStatic(w, Vec2(0, 5));
  BodyId bob = addBob(w, Vec2(L * std::sin(theta0), 5 - L * std::cos(theta0)), 0.01);
  ASSERT_TRUE(w.createRevoluteJoint(revolute(ground, bob, Vec2(0, 5))).isValid());

  // Record zero crossings of x (they alternate every half period).
  std::vector<real> crossings;
  real prevX = w.body(bob)->position().x;
  real t = 0;
  for (int i = 0; i < 120 * 6 && crossings.size() < 3; ++i) {
    w.step(kDt, 8, 3);
    t += kDt;
    real x = w.body(bob)->position().x;
    if (prevX * x < 0.0) {
      real frac = prevX / (prevX - x);
      crossings.push_back(t - kDt + frac * kDt);
    }
    prevX = x;
  }
  ASSERT_EQ(crossings.size(), 3u);
  real period = crossings[2] - crossings[0];
  real expected = 2.0 * kPi * std::sqrt(L / g);
  EXPECT_NEAR(period, expected, 0.02 * expected);
}

TEST(RevoluteJoint, FiveLinkChainAnchorDriftBelowOneMillimetre) {
  World w(Vec2(0, -9.81));
  w.setVelocityIterations(10);
  w.setPositionIterations(8);
  BodyId prev = addStatic(w, Vec2(0, 10));
  std::vector<JointId> joints;
  const real linkLen = 0.5;
  for (int i = 0; i < 5; ++i) {
    BodyId link = addBox(w, Vec2((i + 0.5) * linkLen, 10), linkLen, 0.1);
    joints.push_back(w.createRevoluteJoint(revolute(prev, link, Vec2(i * linkLen, 10))));
    prev = link;
  }
  real maxDrift = 0;
  for (int i = 0; i < 1200; ++i) {
    w.step(kDt, w.velocityIterations(), w.positionIterations());
    for (JointId j : joints) {
      maxDrift = std::max(maxDrift, anchorGap(w.joint(j)));
    }
  }
  EXPECT_LT(maxDrift, 0.001);
  // chain actually swung down
  EXPECT_LT(w.body(prev)->center().y, 9.5);
}

TEST(RevoluteJoint, LowerLimitIsRespected) {
  World w(Vec2(0, -9.81));
  BodyId ground = addStatic(w, Vec2(0, 5));
  BodyId arm = addBox(w, Vec2(0.5, 5), 1.0, 0.1);  // starts horizontal, falls clockwise (negative angle)
  RevoluteJointDef def = revolute(ground, arm, Vec2(0, 5));
  def.enableLimit = true;
  def.lowerAngle = -1.0;
  def.upperAngle = 0.5;
  JointId id = w.createRevoluteJoint(def);
  RevoluteJoint* j = w.revoluteJoint(id);
  ASSERT_NE(j, nullptr);
  real minAngle = 0;
  for (int i = 0; i < 600; ++i) {
    w.step(kDt, 8, 3);
    minAngle = std::min(minAngle, j->jointAngle());
  }
  EXPECT_GT(minAngle, -1.0 - 0.02);
  EXPECT_LT(minAngle, -1.0 + 0.05);  // it really reached the stop
  EXPECT_NEAR(j->jointAngle(), -1.0, 0.03);
  EXPECT_LT(anchorGap(j), 1e-3);
}

TEST(RevoluteJoint, UpperLimitHoldsAgainstMotor) {
  World w(Vec2(0, 0));
  BodyId ground = addStatic(w, Vec2(0, 0));
  BodyId arm = addBox(w, Vec2(0.5, 0), 1.0, 0.1);
  RevoluteJointDef def = revolute(ground, arm, Vec2(0, 0));
  def.enableLimit = true;
  def.lowerAngle = -0.5;
  def.upperAngle = 0.5;
  def.enableMotor = true;
  def.motorSpeed = 5.0;
  def.maxMotorTorque = 1000.0;
  RevoluteJoint* j = w.revoluteJoint(w.createRevoluteJoint(def));
  real maxAngle = -10;
  for (int i = 0; i < 600; ++i) {
    w.step(kDt, 8, 3);
    maxAngle = std::max(maxAngle, j->jointAngle());
  }
  EXPECT_LT(maxAngle, 0.5 + 0.02);
  EXPECT_NEAR(j->jointAngle(), 0.5, 0.02);
  EXPECT_NEAR(w.body(arm)->angularVelocity(), 0.0, 0.05);
}

TEST(RevoluteJoint, EqualLimitsLockTheJoint) {
  World w(Vec2(0, -9.81));
  BodyId ground = addStatic(w, Vec2(0, 0));
  BodyId arm = addBox(w, Vec2(0.5, 0), 1.0, 0.1);
  RevoluteJointDef def = revolute(ground, arm, Vec2(0, 0));
  def.enableLimit = true;
  RevoluteJoint* j = w.revoluteJoint(w.createRevoluteJoint(def));
  for (int i = 0; i < 600; ++i) {
    w.step(kDt, 8, 3);
  }
  EXPECT_NEAR(j->jointAngle(), 0.0, 0.01);
}

TEST(RevoluteJoint, MotorReachesTargetSpeed) {
  World w(Vec2(0, 0));
  BodyId ground = addStatic(w, Vec2(0, 0));
  BodyId arm = addBox(w, Vec2(0.5, 0), 1.0, 0.1);
  RevoluteJointDef def = revolute(ground, arm, Vec2(0, 0));
  def.enableMotor = true;
  def.motorSpeed = 2.0;
  def.maxMotorTorque = 100.0;
  RevoluteJoint* j = w.revoluteJoint(w.createRevoluteJoint(def));
  for (int i = 0; i < 120; ++i) {
    w.step(kDt, 8, 3);
  }
  EXPECT_NEAR(j->jointSpeed(), 2.0, 1e-3);
  EXPECT_NEAR(w.body(arm)->angularVelocity(), 2.0, 1e-3);
}

TEST(RevoluteJoint, MotorTorqueIsCapped) {
  World w(Vec2(0, 0));
  BodyId ground = addStatic(w, Vec2(0, 0));
  BodyId arm = addBox(w, Vec2(0.5, 0), 1.0, 0.1);
  RevoluteJointDef def = revolute(ground, arm, Vec2(0, 0));
  def.enableMotor = true;
  def.motorSpeed = 100.0;
  def.maxMotorTorque = 0.05;
  RevoluteJoint* j = w.revoluteJoint(w.createRevoluteJoint(def));
  for (int i = 0; i < 120; ++i) {
    w.step(kDt, 8, 3);
  }
  // Inertia about the pivot is I_c + m d^2 and the speed can only grow by torque/I per second.
  const RigidBody* b = w.body(arm);
  real iPivot = b->inertia() + b->mass() * 0.25;
  EXPECT_NEAR(j->jointSpeed(), 0.05 / iPivot * 1.0, 0.02 * 0.05 / iPivot);
}

TEST(RevoluteJoint, MotorHoldsAgainstGravityWithEnoughTorque) {
  World w(Vec2(0, -9.81));
  BodyId ground = addStatic(w, Vec2(0, 0));
  BodyId arm = addBox(w, Vec2(0.5, 0), 1.0, 0.1);
  RevoluteJointDef def = revolute(ground, arm, Vec2(0, 0));
  def.enableMotor = true;
  def.motorSpeed = 0.0;
  def.maxMotorTorque = 1000.0;
  RevoluteJoint* j = w.revoluteJoint(w.createRevoluteJoint(def));
  for (int i = 0; i < 240; ++i) {
    w.step(kDt, 8, 3);
  }
  EXPECT_NEAR(j->jointAngle(), 0.0, 0.01);
}

TEST(RevoluteJoint, UndampedPendulumConservesEnergy) {
  World w(Vec2(0, -9.81));
  const real L = 1.0;
  const real theta0 = kPi / 3.0;
  // pivot at y = L so the lowest point of the swing is y = 0
  BodyId ground = addStatic(w, Vec2(0, L));
  BodyId bob = addBob(w, Vec2(L * std::sin(theta0), L - L * std::cos(theta0)), 0.05);
  w.createRevoluteJoint(revolute(ground, bob, Vec2(0, L)));
  real e0 = w.kineticEnergy() + w.potentialEnergy();
  ASSERT_GT(e0, 0.0);
  real worst = 0;
  for (int i = 0; i < 120 * 30; ++i) {
    w.step(kDt, 8, 3);
    real e = w.kineticEnergy() + w.potentialEnergy();
    worst = std::max(worst, std::fabs(e - e0) / e0);
  }
  EXPECT_LT(worst, 0.05);
}

TEST(RevoluteJoint, SpringPullsTowardRestAngle) {
  World w(Vec2(0, 0));
  BodyId ground = addStatic(w, Vec2(0, 0));
  BodyId arm = addBox(w, Vec2(0.5, 0), 1.0, 0.1);
  RevoluteJointDef def = revolute(ground, arm, Vec2(0, 0));
  def.enableSpring = true;
  def.springHertz = 2.0;
  def.springDampingRatio = 1.0;
  def.springRestAngle = 0.0;
  RevoluteJoint* j = w.revoluteJoint(w.createRevoluteJoint(def));
  // disturb the arm by 0.5 rad about the pivot
  w.setBodyTransform(arm, Vec2(0, 0) + mul(Rot(0.5), Vec2(0.5, 0)), 0.5);
  EXPECT_NEAR(j->jointAngle(), 0.5, 1e-9);
  real peak = 0;
  for (int i = 0; i < 120 * 5; ++i) {
    w.step(kDt, 8, 3);
    peak = std::max(peak, std::fabs(j->jointAngle()));
  }
  EXPECT_LT(std::fabs(j->jointAngle()), 0.01);
  EXPECT_LE(peak, 0.5 + 1e-6);
}

TEST(RevoluteJoint, SpringSupportsGravityLoadAtOffsetEquilibrium) {
  World w(Vec2(0, -9.81));
  BodyId ground = addStatic(w, Vec2(0, 0));
  BodyId arm = addBox(w, Vec2(0.5, 0), 1.0, 0.1);
  RevoluteJointDef def = revolute(ground, arm, Vec2(0, 0));
  def.enableSpring = true;
  def.springHertz = 1.0;
  def.springDampingRatio = 1.0;
  RevoluteJoint* j = w.revoluteJoint(w.createRevoluteJoint(def));
  for (int i = 0; i < 120 * 10; ++i) {
    w.step(kDt, 8, 3);
  }
  // Sags below the rest angle but does not fall through.
  EXPECT_LT(j->jointAngle(), -0.01);
  EXPECT_GT(j->jointAngle(), -1.5);
  EXPECT_NEAR(w.body(arm)->angularVelocity(), 0.0, 1e-3);
}

TEST(RevoluteJoint, SettersToggleFeatures) {
  World w(Vec2(0, 0));
  BodyId ground = addStatic(w, Vec2(0, 0));
  BodyId arm = addBox(w, Vec2(0.5, 0), 1.0, 0.1);
  RevoluteJoint* j = w.revoluteJoint(w.createRevoluteJoint(revolute(ground, arm, Vec2(0, 0))));
  j->setMotorEnabled(true);
  j->setMotorSpeed(1.0);
  j->setMaxMotorTorque(50.0);
  for (int i = 0; i < 120; ++i) {
    w.step(kDt, 8, 3);
  }
  EXPECT_NEAR(j->jointSpeed(), 1.0, 1e-3);
  j->setMotorEnabled(false);
  j->setLimitEnabled(true);
  j->setLimits(-1.0, j->jointAngle() + 0.1);
  for (int i = 0; i < 240; ++i) {
    w.step(kDt, 8, 3);
  }
  EXPECT_LT(j->jointAngle(), j->upperAngle() + 0.02);
}

// ---- DistanceJoint -----------------------------------------------------------

TEST(DistanceJoint, RigidRodKeepsLength) {
  World w(Vec2(0, -9.81));
  BodyId ground = addStatic(w, Vec2(0, 5));
  BodyId bob = addBob(w, Vec2(2, 5), 0.1);
  DistanceJointDef def;
  def.bodyA = ground;
  def.bodyB = bob;
  def.anchorA = Vec2(0, 5);
  def.anchorB = Vec2(2, 5);
  JointId id = w.createDistanceJoint(def);
  ASSERT_TRUE(id.isValid());
  DistanceJoint* j = w.distanceJoint(id);
  ASSERT_NE(j, nullptr);
  EXPECT_NEAR(j->length(), 2.0, 1e-12);
  real worst = 0;
  real lowest = 5.0;
  for (int i = 0; i < 120 * 10; ++i) {
    w.step(kDt, 8, 3);
    worst = std::max(worst, std::fabs(anchorGap(j) - 2.0));
    lowest = std::min(lowest, w.body(bob)->position().y);
  }
  EXPECT_LT(worst, 0.001);
  EXPECT_LT(lowest, 3.5);  // it swung down towards y = 3
}

TEST(DistanceJoint, ExplicitLengthPullsBodiesToRestLength) {
  World w(Vec2(0, 0));
  BodyId a = addBob(w, Vec2(0, 0), 0.1);
  BodyId b = addBob(w, Vec2(3, 0), 0.1);
  DistanceJointDef def;
  def.bodyA = a;
  def.bodyB = b;
  def.anchorA = Vec2(0, 0);
  def.anchorB = Vec2(3, 0);
  def.length = 1.0;
  w.createDistanceJoint(def);
  for (int i = 0; i < 240; ++i) {
    w.step(kDt, 8, 3);
  }
  EXPECT_NEAR(distance(w.body(a)->center(), w.body(b)->center()), 1.0, 0.002);
}

TEST(DistanceJoint, SoftSpringSettlesAtStaticExtension) {
  World w(Vec2(0, -9.81));
  BodyId ground = addStatic(w, Vec2(0, 5));
  BodyId bob = addBob(w, Vec2(0, 4), 0.1);
  DistanceJointDef def;
  def.bodyA = ground;
  def.bodyB = bob;
  def.anchorA = Vec2(0, 5);
  def.anchorB = Vec2(0, 4);
  def.hertz = 2.0;
  def.dampingRatio = 1.0;
  DistanceJoint* j = w.distanceJoint(w.createDistanceJoint(def));
  for (int i = 0; i < 120 * 8; ++i) {
    w.step(kDt, 8, 3);
  }
  real omega = 2.0 * kPi * 2.0;
  real extension = 9.81 / (omega * omega);
  EXPECT_NEAR(anchorGap(j), 1.0 + extension, 0.005);
  EXPECT_NEAR(w.body(bob)->linearVelocity().y, 0.0, 1e-3);
}

// ---- MouseJoint --------------------------------------------------------------

TEST(MouseJoint, DragsBodyToTarget) {
  World w(Vec2(0, -9.81));
  BodyId body = addBox(w, Vec2(0, 0), 1, 1);
  JointId id = w.createMouseJoint(body, Vec2(0, 0), 10000.0);
  ASSERT_TRUE(id.isValid());
  w.setMouseTarget(id, Vec2(3, 2));
  for (int i = 0; i < 120 * 4; ++i) {
    w.step(kDt, 8, 3);
  }
  EXPECT_NEAR(w.body(body)->position().x, 3.0, 0.05);
  EXPECT_NEAR(w.body(body)->position().y, 2.0, 0.1);  // small sag from gravity on a soft spring
}

TEST(MouseJoint, GrabPointOffCentreProducesRotation) {
  World w(Vec2(0, 0));
  BodyId body = addBox(w, Vec2(0, 0), 2, 2);
  JointId id = w.createMouseJoint(body, Vec2(1, 0.9), 10000.0);
  w.setMouseTarget(id, Vec2(1, 3));
  real maxOmega = 0;
  for (int i = 0; i < 60; ++i) {
    w.step(kDt, 8, 3);
    maxOmega = std::max(maxOmega, std::fabs(w.body(body)->angularVelocity()));
  }
  EXPECT_GT(maxOmega, 0.1);
}

TEST(MouseJoint, MaxForceLimitsPull) {
  World w(Vec2(0, -10));
  BodyId body = addBox(w, Vec2(0, 0), 1, 1, 100.0);  // heavy: 100 kg
  JointId id = w.createMouseJoint(body, Vec2(0, 0), 10.0);  // far less than weight (1000 N)
  w.setMouseTarget(id, Vec2(0, 10));
  for (int i = 0; i < 240; ++i) {
    w.step(kDt, 8, 3);
  }
  EXPECT_LT(w.body(body)->position().y, 0.0);
}

TEST(MouseJoint, AnchorsReportTargetAndGrabPoint) {
  World w(Vec2(0, 0));
  BodyId body = addBox(w, Vec2(1, 1), 1, 1);
  JointId id = w.createMouseJoint(body, Vec2(1.2, 1.1), 100.0);
  w.setMouseTarget(id, Vec2(5, 6));
  const Joint* j = w.joint(id);
  ASSERT_NE(j, nullptr);
  EXPECT_NEAR(j->anchorA().x, 5.0, 1e-12);
  EXPECT_NEAR(j->anchorA().y, 6.0, 1e-12);
  EXPECT_NEAR(j->anchorB().x, 1.2, 1e-12);
  EXPECT_NEAR(j->anchorB().y, 1.1, 1e-12);
}

TEST(MouseJoint, RejectsStaticOrInvalidBody) {
  World w;
  BodyId s = addStatic(w, Vec2(0, 0));
  EXPECT_FALSE(w.createMouseJoint(s, Vec2(0, 0), 10.0).isValid());
  EXPECT_FALSE(w.createMouseJoint(BodyId(), Vec2(0, 0), 10.0).isValid());
  EXPECT_FALSE(w.setMouseTarget(JointId(), Vec2(0, 0)));
}

// ---- Warm starting -----------------------------------------------------------

namespace {
real chainDriftWithFewIterations(bool warm) {
  World w(Vec2(0, -9.81));
  w.setWarmStarting(warm);
  BodyId prev = addStatic(w, Vec2(0, 10));
  std::vector<JointId> joints;
  for (int i = 0; i < 5; ++i) {
    BodyId link = addBox(w, Vec2(0, 10 - (i + 0.5) * 0.5), 0.1, 0.5);
    joints.push_back(w.createRevoluteJoint(revolute(prev, link, Vec2(0, 10 - i * 0.5))));
    prev = link;
  }
  real worst = 0;
  for (int i = 0; i < 240; ++i) {
    w.step(kDt, 1, 0);  // one velocity iteration, no position correction
    if (i > 60) {
      for (JointId j : joints) {
        worst = std::max(worst, anchorGap(w.joint(j)));
      }
    }
  }
  return worst;
}
}  // namespace

TEST(WarmStarting, AccumulatedImpulsesAreCarriedOver) {
  World w(Vec2(0, -9.81));
  BodyId ground = addStatic(w, Vec2(0, 5));
  BodyId bob = addBob(w, Vec2(0, 4), 0.1);
  RevoluteJoint* j = w.revoluteJoint(w.createRevoluteJoint(revolute(ground, bob, Vec2(0, 5))));
  w.step(kDt, 8, 3);
  EXPECT_GT(j->impulse().length(), 0.0);
  EXPECT_TRUE(w.warmStarting());
}

TEST(WarmStarting, ReducesDriftWithFewIterations) {
  real warm = chainDriftWithFewIterations(true);
  real cold = chainDriftWithFewIterations(false);
  EXPECT_LT(warm, cold);
}

// ---- World integration -------------------------------------------------------

TEST(JointLifecycle, DestroyBodyRemovesAttachedJoints) {
  World w;
  BodyId a = addBob(w, Vec2(0, 0), 0.5);
  BodyId b = addBob(w, Vec2(1, 0), 0.5);
  BodyId c = addBob(w, Vec2(2, 0), 0.5);
  JointId ab = w.createRevoluteJoint(revolute(a, b, Vec2(0.5, 0)));
  JointId bc = w.createRevoluteJoint(revolute(b, c, Vec2(1.5, 0)));
  EXPECT_EQ(w.jointCount(), 2u);
  w.destroyBody(a);
  EXPECT_EQ(w.jointCount(), 1u);
  EXPECT_EQ(w.joint(ab), nullptr);
  EXPECT_NE(w.joint(bc), nullptr);
  w.step(kDt, 8, 3);  // must not touch freed bodies
  EXPECT_TRUE(w.destroyJoint(bc));
  EXPECT_FALSE(w.destroyJoint(bc));
  EXPECT_EQ(w.jointCount(), 0u);
}

TEST(JointLifecycle, InvalidCreationIsRejected) {
  World w;
  BodyId a = addBob(w, Vec2(0, 0), 0.5);
  EXPECT_FALSE(w.createRevoluteJoint(revolute(a, a, Vec2(0, 0))).isValid());
  EXPECT_FALSE(w.createRevoluteJoint(revolute(a, BodyId(), Vec2(0, 0))).isValid());
  BodyId s1 = addStatic(w, Vec2(0, 0));
  BodyId s2 = addStatic(w, Vec2(1, 0));
  EXPECT_FALSE(w.createRevoluteJoint(revolute(s1, s2, Vec2(0, 0))).isValid());
  DistanceJointDef d;
  d.bodyA = a;
  d.bodyB = a;
  EXPECT_FALSE(w.createDistanceJoint(d).isValid());
  EXPECT_EQ(w.jointCount(), 0u);
}

TEST(JointLifecycle, JointHandlesAreStableAcrossRemoval) {
  World w;
  BodyId a = addStatic(w, Vec2(0, 0));
  BodyId b = addBob(w, Vec2(1, 0), 0.1);
  BodyId c = addBob(w, Vec2(2, 0), 0.1);
  JointId j1 = w.createRevoluteJoint(revolute(a, b, Vec2(0, 0)));
  JointId j2 = w.createRevoluteJoint(revolute(b, c, Vec2(1.5, 0)));
  w.destroyJoint(j1);
  JointId j3 = w.createRevoluteJoint(revolute(a, c, Vec2(0, 0)));
  EXPECT_EQ(j3.index, j1.index);
  EXPECT_NE(j3.generation, j1.generation);
  EXPECT_EQ(w.joint(j1), nullptr);
  EXPECT_NE(w.joint(j2), nullptr);
  EXPECT_EQ(JointId::fromInt(j3.toInt()), j3);
  EXPECT_EQ(w.revoluteJoint(j2)->type(), JointType::Revolute);
  EXPECT_EQ(w.distanceJoint(j2), nullptr);  // wrong kind
}

TEST(JointBuffers, AnchorsArePackedAsAxAyBxBy) {
  World w(Vec2(0, 0));
  BodyId a = addStatic(w, Vec2(0, 0));
  BodyId b = addBob(w, Vec2(2, 0), 0.1);
  w.createRevoluteJoint(revolute(a, b, Vec2(1, 0)));
  const std::vector<float>& anchors = w.jointAnchors();
  ASSERT_EQ(anchors.size(), 4u);
  EXPECT_FLOAT_EQ(anchors[0], 1.0f);
  EXPECT_FLOAT_EQ(anchors[1], 0.0f);
  EXPECT_FLOAT_EQ(anchors[2], 1.0f);
  EXPECT_FLOAT_EQ(anchors[3], 0.0f);
  w.setBodyTransform(b, Vec2(2, 1), 0.0);  // anchors diverge until the solver runs
  EXPECT_FLOAT_EQ(w.jointAnchors()[3], 1.0f);
  EXPECT_EQ(w.jointIds().size(), 1u);
}

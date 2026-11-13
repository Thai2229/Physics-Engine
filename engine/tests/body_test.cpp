#include <gtest/gtest.h>

#include <physics/body.hpp>

using namespace physics;

namespace {
constexpr real kTol = 1e-9;

Fixture boxFixture(real w, real h, real density, Vec2 c = Vec2(0, 0)) {
  Fixture f;
  f.shape = Shape::box(w, h, c);
  f.density = density;
  return f;
}
}  // namespace

TEST(RigidBody, DynamicBodyMassFromShape) {
  BodyDef def;
  def.type = BodyType::Dynamic;
  RigidBody b(def);
  ASSERT_TRUE(b.addShape(boxFixture(2, 1, 3)));
  EXPECT_NEAR(b.mass(), 6.0, kTol);
  EXPECT_NEAR(b.invMass(), 1.0 / 6.0, kTol);
  EXPECT_NEAR(b.inertia(), 6.0 * 5.0 / 12.0, kTol);
  EXPECT_NEAR(b.invInertia(), 1.0 / b.inertia(), kTol);
  EXPECT_EQ(b.shapes().size(), 1u);
}

TEST(RigidBody, StaticBodyHasZeroInverseMass) {
  BodyDef def;
  def.type = BodyType::Static;
  RigidBody b(def);
  ASSERT_TRUE(b.addShape(boxFixture(2, 1, 3)));
  EXPECT_EQ(b.invMass(), 0.0);
  EXPECT_EQ(b.invInertia(), 0.0);
  b.applyForce(Vec2(100, 100));
  b.applyLinearImpulse(Vec2(10, 10), b.center());
  EXPECT_EQ(b.linearVelocity(), Vec2(0, 0));
  EXPECT_EQ(b.force(), Vec2(0, 0));
}

TEST(RigidBody, KinematicBodyHasZeroInverseMass) {
  BodyDef def;
  def.type = BodyType::Kinematic;
  def.linearVelocity = Vec2(1, 0);
  RigidBody b(def);
  b.addShape(boxFixture(1, 1, 1));
  EXPECT_EQ(b.invMass(), 0.0);
  EXPECT_EQ(b.linearVelocity(), Vec2(1, 0));
}

TEST(RigidBody, DynamicBodyWithoutShapeDefaultsToUnitMassNoRotation) {
  BodyDef def;
  def.type = BodyType::Dynamic;
  RigidBody b(def);
  EXPECT_EQ(b.mass(), 1.0);
  EXPECT_EQ(b.invMass(), 1.0);
  EXPECT_EQ(b.invInertia(), 0.0);
}

TEST(RigidBody, ZeroDensityShapeFallsBackToUnitMass) {
  BodyDef def;
  RigidBody b(def);
  b.addShape(boxFixture(1, 1, 0.0));
  EXPECT_EQ(b.mass(), 1.0);
}

TEST(RigidBody, InvalidShapeIsRejected) {
  BodyDef def;
  RigidBody b(def);
  Fixture f;
  f.shape = Shape::circle(-1);
  EXPECT_FALSE(b.addShape(f));
  EXPECT_TRUE(b.shapes().empty());
}

TEST(RigidBody, CenterOfMassOffsetAndInertiaAboutCenter) {
  BodyDef def;
  def.position = Vec2(10, 0);
  RigidBody b(def);
  b.addShape(boxFixture(1, 1, 1, Vec2(2, 0)));
  EXPECT_NEAR(b.localCenter().x, 2.0, kTol);
  EXPECT_NEAR(b.center().x, 12.0, kTol);
  EXPECT_NEAR(b.position().x, 10.0, kTol);
  EXPECT_NEAR(b.inertia(), 1.0 / 6.0, kTol);  // about the centre of mass, not the origin
}

TEST(RigidBody, TwoShapesCombineMassAndCentre) {
  BodyDef def;
  RigidBody b(def);
  b.addShape(boxFixture(1, 1, 1, Vec2(-1, 0)));
  b.addShape(boxFixture(1, 1, 3, Vec2(1, 0)));
  EXPECT_NEAR(b.mass(), 4.0, kTol);
  EXPECT_NEAR(b.localCenter().x, 0.5, kTol);
  // I = sum(I_i + m_i d_i^2)
  real expected = (1.0 / 6.0 + 1.0 * 1.5 * 1.5) + (3.0 * 2.0 / 12.0 + 3.0 * 0.5 * 0.5);
  EXPECT_NEAR(b.inertia(), expected, kTol);
}

TEST(RigidBody, WorldCenterFollowsRotation) {
  BodyDef def;
  def.angle = kPi / 2;
  RigidBody b(def);
  b.addShape(boxFixture(1, 1, 1, Vec2(2, 0)));
  EXPECT_NEAR(b.center().x, 0.0, 1e-9);
  EXPECT_NEAR(b.center().y, 2.0, 1e-9);
  b.setTransform(Vec2(1, 1), 0.0);
  EXPECT_NEAR(b.center().x, 3.0, 1e-9);
  EXPECT_NEAR(b.center().y, 1.0, 1e-9);
}

TEST(RigidBody, ForcesImpulsesAndPointVelocity) {
  BodyDef def;
  RigidBody b(def);
  b.addShape(boxFixture(1, 1, 2));  // m = 2
  b.applyForce(Vec2(1, 0));
  b.applyForce(Vec2(1, 2));
  EXPECT_EQ(b.force(), Vec2(2, 2));
  b.applyTorque(0.5);
  EXPECT_EQ(b.torque(), 0.5);
  b.clearForces();
  EXPECT_EQ(b.force(), Vec2(0, 0));
  EXPECT_EQ(b.torque(), 0.0);

  b.applyForceAtPoint(Vec2(0, 1), b.center() + Vec2(1, 0));
  EXPECT_NEAR(b.torque(), 1.0, kTol);

  b.applyLinearImpulse(Vec2(2, 0), b.center());
  EXPECT_NEAR(b.linearVelocity().x, 1.0, kTol);
  EXPECT_NEAR(b.angularVelocity(), 0.0, kTol);

  b.setAngularVelocity(2.0);
  Vec2 v = b.velocityAtWorldPoint(b.center() + Vec2(1, 0));
  EXPECT_NEAR(v.x, 1.0, kTol);
  EXPECT_NEAR(v.y, 2.0, kTol);
}

TEST(RigidBody, LocalWorldRoundTrip) {
  BodyDef def;
  def.position = Vec2(1, 2);
  def.angle = 0.8;
  RigidBody b(def);
  Vec2 p(0.3, -0.4);
  Vec2 back = b.worldToLocal(b.localToWorld(p));
  EXPECT_NEAR(back.x, p.x, kTol);
  EXPECT_NEAR(back.y, p.y, kTol);
}

TEST(RigidBody, FixtureCarriesSurfaceAndFilterData) {
  Fixture f;
  EXPECT_GT(f.friction, 0.0);
  EXPECT_EQ(f.restitution, 0.0);
  EXPECT_EQ(f.filter.group, 0);
  EXPECT_NE(f.filter.category, 0u);
  EXPECT_NE(f.filter.mask, 0u);
}

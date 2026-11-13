#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <physics/world.hpp>

using namespace physics;

namespace {

BodyId addCircle(World& w, Vec2 p, real radius = 0.5, real density = 1.0, BodyType type = BodyType::Dynamic) {
  BodyDef def;
  def.type = type;
  def.position = p;
  BodyId id = w.createBody(def);
  Fixture f;
  f.shape = Shape::circle(radius);
  f.density = density;
  w.addShape(id, f);
  return id;
}

}  // namespace

TEST(World, FreeFallMatchesAnalyticSolution) {
  World w(Vec2(0, -9.81));
  BodyId id = addCircle(w, Vec2(0, 100));
  const real dt = 1.0 / 120.0;
  for (int i = 0; i < 120; ++i) {
    w.step(dt, 8, 3);
  }
  real expected = 100.0 - 0.5 * 9.81 * 1.0;
  EXPECT_NEAR(w.body(id)->position().y, expected, 0.02 * 0.5 * 9.81);
  EXPECT_NEAR(w.body(id)->linearVelocity().y, -9.81, 0.02 * 9.81);
  EXPECT_NEAR(w.body(id)->position().x, 0.0, 1e-12);
}

TEST(World, ZeroGravityConservesMomentum) {
  World w(Vec2(0, 0));
  BodyId a = addCircle(w, Vec2(0, 0), 0.5, 1.0);
  BodyId b = addCircle(w, Vec2(5, 1), 0.25, 4.0);
  w.setBodyVelocity(a, Vec2(1, 2), 0.5);
  w.setBodyVelocity(b, Vec2(-0.5, 0.25), -1.0);
  auto momentum = [&]() {
    Vec2 p = w.body(a)->mass() * w.body(a)->linearVelocity() + w.body(b)->mass() * w.body(b)->linearVelocity();
    return p;
  };
  Vec2 p0 = momentum();
  real l0 = w.body(a)->inertia() * w.body(a)->angularVelocity();
  for (int i = 0; i < 1200; ++i) {
    w.step(1.0 / 120.0, 8, 3);
  }
  EXPECT_NEAR(momentum().x, p0.x, 1e-9);
  EXPECT_NEAR(momentum().y, p0.y, 1e-9);
  EXPECT_NEAR(w.body(a)->inertia() * w.body(a)->angularVelocity(), l0, 1e-9);
  // and it actually moved: a travels v * t = (10, 20) over 10 s
  EXPECT_NEAR(w.body(a)->position().x, 10.0, 0.05);
  EXPECT_NEAR(w.body(a)->position().y, 20.0, 0.05);
}

TEST(World, LinearDampingDecaysVelocity) {
  World w(Vec2(0, 0));
  BodyDef def;
  def.linearDamping = 1.0;
  def.angularDamping = 2.0;
  BodyId id = w.createBody(def);
  w.setBodyVelocity(id, Vec2(10, 0), 3.0);
  for (int i = 0; i < 120; ++i) {
    w.step(1.0 / 120.0, 8, 3);
  }
  EXPECT_NEAR(w.body(id)->linearVelocity().x, 10.0 * std::exp(-1.0), 0.02 * 10.0);
  EXPECT_NEAR(w.body(id)->angularVelocity(), 3.0 * std::exp(-2.0), 0.02 * 3.0);
  EXPECT_LT(w.body(id)->linearVelocity().x, 10.0);
}

TEST(World, StaticBodiesNeverMove) {
  World w(Vec2(0, -9.81));
  BodyId id = addCircle(w, Vec2(1, 2), 1.0, 1.0, BodyType::Static);
  w.body(id)->applyForce(Vec2(1000, 1000));
  for (int i = 0; i < 240; ++i) {
    w.step(1.0 / 120.0, 8, 3);
  }
  EXPECT_EQ(w.body(id)->position(), Vec2(1, 2));
  EXPECT_EQ(w.body(id)->linearVelocity(), Vec2(0, 0));
  EXPECT_EQ(w.body(id)->angle(), 0.0);
}

TEST(World, KinematicBodiesMoveAtConstantVelocityIgnoringGravity) {
  World w(Vec2(0, -9.81));
  BodyDef def;
  def.type = BodyType::Kinematic;
  def.linearVelocity = Vec2(1, 0);
  def.angularVelocity = 1.0;
  BodyId id = w.createBody(def);
  for (int i = 0; i < 120; ++i) {
    w.step(1.0 / 120.0, 8, 3);
  }
  EXPECT_NEAR(w.body(id)->position().x, 1.0, 1e-9);
  EXPECT_NEAR(w.body(id)->position().y, 0.0, 1e-12);
  EXPECT_NEAR(w.body(id)->angle(), 1.0, 1e-9);
}

TEST(World, AppliedForceAccelerates) {
  World w(Vec2(0, 0));
  BodyId id = addCircle(w, Vec2(0, 0), 0.5, 2.0 / kPi / 0.25);  // mass 2
  EXPECT_NEAR(w.body(id)->mass(), 2.0, 1e-9);
  w.body(id)->applyForce(Vec2(4, 0));
  w.step(0.5, 8, 3);
  EXPECT_NEAR(w.body(id)->linearVelocity().x, 1.0, 1e-9);
  // forces are cleared after a step
  w.step(0.5, 8, 3);
  EXPECT_NEAR(w.body(id)->linearVelocity().x, 1.0, 1e-9);
}

TEST(World, GravityScaleAndSetGravity) {
  World w;
  EXPECT_NEAR(w.gravity().y, -9.81, 1e-12);
  w.setGravity(Vec2(0, -1));
  EXPECT_EQ(w.gravity(), Vec2(0, -1));
  BodyDef def;
  def.gravityScale = 0.0;
  BodyId id = w.createBody(def);
  w.step(1.0, 1, 1);
  EXPECT_EQ(w.body(id)->linearVelocity(), Vec2(0, 0));
}

TEST(World, NonPositiveDtIsIgnored) {
  World w(Vec2(0, -10));
  BodyId id = addCircle(w, Vec2(0, 0));
  w.step(0.0, 8, 3);
  w.step(-1.0, 8, 3);
  EXPECT_EQ(w.body(id)->position(), Vec2(0, 0));
}

// ---- handles --------------------------------------------------------------

TEST(Handles, StaleIdsAreRejectedAndOthersSurviveRemoval) {
  World w;
  BodyId a = addCircle(w, Vec2(0, 0));
  BodyId b = addCircle(w, Vec2(1, 0));
  BodyId c = addCircle(w, Vec2(2, 0));
  EXPECT_EQ(w.bodyCount(), 3u);
  EXPECT_TRUE(w.destroyBody(b));
  EXPECT_EQ(w.bodyCount(), 2u);
  EXPECT_EQ(w.body(b), nullptr);
  EXPECT_FALSE(w.destroyBody(b));
  ASSERT_NE(w.body(a), nullptr);
  ASSERT_NE(w.body(c), nullptr);
  EXPECT_EQ(w.body(c)->position().x, 2.0);
}

TEST(Handles, ReusedSlotGetsNewGeneration) {
  World w;
  BodyId a = addCircle(w, Vec2(0, 0));
  w.destroyBody(a);
  BodyId a2 = addCircle(w, Vec2(5, 0));
  EXPECT_EQ(a2.index, a.index);
  EXPECT_NE(a2.generation, a.generation);
  EXPECT_EQ(w.body(a), nullptr);
  ASSERT_NE(w.body(a2), nullptr);
  EXPECT_EQ(w.body(a2)->position().x, 5.0);
}

TEST(Handles, InvalidAndOutOfRangeIdsAreSafe) {
  World w;
  BodyId none;
  EXPECT_FALSE(none.isValid());
  EXPECT_EQ(w.body(none), nullptr);
  BodyId bogus;
  bogus.index = 9999;
  bogus.generation = 1;
  EXPECT_EQ(w.body(bogus), nullptr);
  EXPECT_FALSE(w.destroyBody(bogus));
  EXPECT_FALSE(w.setBodyTransform(bogus, Vec2(0, 0), 0));
  EXPECT_FALSE(w.setBodyVelocity(bogus, Vec2(0, 0), 0));
  EXPECT_FALSE(w.addShape(bogus, Fixture()));
}

TEST(Handles, PackedIntRoundTrip) {
  World w;
  BodyId a = addCircle(w, Vec2(0, 0));
  for (int i = 0; i < 5; ++i) {
    w.destroyBody(a);
    a = addCircle(w, Vec2(0, 0));
  }
  std::int32_t packed = a.toInt();
  EXPECT_GE(packed, 0);
  BodyId back = BodyId::fromInt(packed);
  EXPECT_EQ(back, a);
  EXPECT_NE(w.body(back), nullptr);
  EXPECT_EQ(BodyId().toInt(), -1);
  EXPECT_FALSE(BodyId::fromInt(-1).isValid());
}

TEST(Handles, GenerationWrapsWithoutBecomingInvalid) {
  World w;
  BodyId a = addCircle(w, Vec2(0, 0));
  for (int i = 0; i < 0x8000 + 3; ++i) {
    w.destroyBody(a);
    a = addCircle(w, Vec2(0, 0));
    ASSERT_TRUE(a.isValid());
    ASSERT_LE(a.generation, BodyId::kMaxGeneration);
  }
  EXPECT_NE(w.body(a), nullptr);
}

// ---- fixed step accumulator -------------------------------------------------

TEST(Advance, RunsWholeFixedStepsAndCarriesRemainder) {
  World w;
  w.setFixedTimeStep(1.0 / 120.0);
  EXPECT_EQ(w.advance(1.0 / 60.0), 2);
  EXPECT_EQ(w.advance(0.004), 0);  // 4 ms < 8.33 ms step
  EXPECT_EQ(w.advance(0.005), 1);  // 9 ms accumulated
  EXPECT_NEAR(w.accumulator(), 0.009 - 1.0 / 120.0, 1e-12);
}

TEST(Advance, ClampsHugeFrameTimes) {
  World w;
  w.setFixedTimeStep(1.0 / 120.0);
  w.setMaxFrameTime(0.1);
  int steps = w.advance(10.0);
  EXPECT_LE(steps, 12);
  EXPECT_GE(steps, 11);
  EXPECT_EQ(w.advance(-1.0), 0);
}

TEST(Advance, FrameRateDoesNotChangeTheSimulation) {
  auto run = [](real frameDt, int frames) {
    World w(Vec2(0, -9.81));
    BodyId id = addCircle(w, Vec2(0, 50));
    for (int i = 0; i < frames; ++i) {
      w.advance(frameDt);
    }
    return w.body(id)->position().y;
  };
  real a = run(1.0 / 60.0, 60);
  real b = run(1.0 / 30.0, 30);
  real c = run(1.0 / 240.0, 240);
  EXPECT_NEAR(a, b, 1e-9);
  EXPECT_NEAR(a, c, 1e-9);
}

// ---- buffers & stats --------------------------------------------------------

TEST(Buffers, BodyStatesAndIdsAreContiguousInSlotOrder) {
  World w(Vec2(0, 0));
  BodyId a = addCircle(w, Vec2(1, 2));
  BodyId b = addCircle(w, Vec2(3, 4));
  BodyId c = addCircle(w, Vec2(5, 6));
  w.setBodyTransform(b, Vec2(3, 4), 0.5);
  w.destroyBody(a);
  const std::vector<float>& states = w.bodyStates();
  const std::vector<std::int32_t>& ids = w.bodyIds();
  ASSERT_EQ(ids.size(), 2u);
  ASSERT_EQ(states.size(), 6u);
  EXPECT_EQ(ids[0], b.toInt());
  EXPECT_EQ(ids[1], c.toInt());
  EXPECT_FLOAT_EQ(states[0], 3.0f);
  EXPECT_FLOAT_EQ(states[1], 4.0f);
  EXPECT_FLOAT_EQ(states[2], 0.5f);
  EXPECT_FLOAT_EQ(states[3], 5.0f);
  EXPECT_FLOAT_EQ(states[4], 6.0f);
  // buffers refresh after stepping
  w.setBodyVelocity(c, Vec2(1, 0), 0);
  w.step(1.0, 1, 1);
  EXPECT_FLOAT_EQ(w.bodyStates()[3], 6.0f);
}

TEST(Buffers, EmptyWorldHasEmptyBuffers) {
  World w;
  EXPECT_TRUE(w.bodyStates().empty());
  EXPECT_TRUE(w.bodyIds().empty());
  EXPECT_TRUE(w.jointAnchors().empty());
}

TEST(Stats, EnergyAndCounts) {
  World w(Vec2(0, -10));
  BodyId id = addCircle(w, Vec2(0, 3), 0.5, 1.0);
  addCircle(w, Vec2(0, 100), 1.0, 1.0, BodyType::Static);  // statics contribute nothing
  real m = w.body(id)->mass();
  EXPECT_EQ(w.bodyCount(), 2u);
  EXPECT_NEAR(w.potentialEnergy(), m * 10.0 * 3.0, 1e-9);
  EXPECT_NEAR(w.kineticEnergy(), 0.0, 1e-12);
  w.setBodyVelocity(id, Vec2(2, 0), 2.0);
  real expectedKe = 0.5 * m * 4.0 + 0.5 * w.body(id)->inertia() * 4.0;
  EXPECT_NEAR(w.kineticEnergy(), expectedKe, 1e-9);
}

TEST(Stats, FreeFallConservesTotalEnergyApproximately) {
  World w(Vec2(0, -10));
  addCircle(w, Vec2(0, 100));
  real e0 = w.kineticEnergy() + w.potentialEnergy();
  for (int i = 0; i < 240; ++i) {
    w.step(1.0 / 120.0, 8, 3);
  }
  real e1 = w.kineticEnergy() + w.potentialEnergy();
  EXPECT_NEAR(e1, e0, 0.01 * e0);
}

TEST(World, ManyBodiesStayFinite) {
  World w;
  for (int i = 0; i < 10000; ++i) {
    addCircle(w, Vec2(i * 0.1, 0));
  }
  w.advance(1.0 / 60.0);
  EXPECT_EQ(w.bodyCount(), 10000u);
  EXPECT_EQ(w.bodyStates().size(), 30000u);
  EXPECT_TRUE(std::isfinite(w.bodyStates()[1]));
}

TEST(WorldSpeedLimits, LinearAndAngularSpeedAreCapped) {
  World w(Vec2(0, 0));
  BodyDef def;
  def.linearVelocity = Vec2(1.0e4, 0);
  def.angularVelocity = 1.0e4;
  BodyId id = w.createBody(def);
  Fixture f;
  f.shape = Shape::circle(0.5);
  w.addShape(id, f);
  w.step(1.0 / 120.0, 8, 3);
  EXPECT_LE(w.body(id)->linearVelocity().length(), w.maxLinearSpeed() + 1e-9);
  EXPECT_LE(std::abs(w.body(id)->angularVelocity()), w.maxAngularSpeed() + 1e-9);
}

TEST(WorldSpeedLimits, NormalSpeedsAreUntouched) {
  World w(Vec2(0, -9.81));
  BodyDef def;
  def.linearVelocity = Vec2(3.0, 0);
  BodyId id = w.createBody(def);
  Fixture f;
  f.shape = Shape::circle(0.5);
  w.addShape(id, f);
  w.step(1.0 / 120.0, 8, 3);
  EXPECT_NEAR(w.body(id)->linearVelocity().x, 3.0, 1e-9);
}

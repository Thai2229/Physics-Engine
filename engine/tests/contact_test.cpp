#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include <physics/world.hpp>

using namespace physics;

namespace {

const real kDt = 1.0 / 120.0;

BodyId addBox(World& w, Vec2 p, real hx, real hy, BodyType type = BodyType::Dynamic, real friction = 0.3,
              real restitution = 0.0, real angle = 0, Filter filter = Filter()) {
  BodyDef def;
  def.type = type;
  def.position = p;
  def.angle = angle;
  BodyId id = w.createBody(def);
  Fixture f;
  f.shape = Shape::box(2 * hx, 2 * hy);
  f.friction = friction;
  f.restitution = restitution;
  f.filter = filter;
  w.addShape(id, f);
  return id;
}

BodyId addBall(World& w, Vec2 p, real r, BodyType type = BodyType::Dynamic, real friction = 0.3,
               real restitution = 0.0) {
  BodyDef def;
  def.type = type;
  def.position = p;
  BodyId id = w.createBody(def);
  Fixture f;
  f.shape = Shape::circle(r);
  f.friction = friction;
  f.restitution = restitution;
  w.addShape(id, f);
  return id;
}

/// Static ground whose top surface is y = 0.
BodyId addGround(World& w, real friction = 0.3, real restitution = 0.0) {
  return addBox(w, Vec2(0, -0.5), 50, 0.5, BodyType::Static, friction, restitution);
}

}  // namespace

// ---- stacking & resting ------------------------------------------------------------

TEST(Contacts, CircleRestsOnGround) {
  World w;
  addGround(w);
  BodyId b = addBall(w, Vec2(0, 0.5), 0.5);
  for (int i = 0; i < 600; ++i) {
    w.step(kDt, 8, 3);
  }
  EXPECT_NEAR(w.body(b)->position().y, 0.5, 0.01);
  EXPECT_LT(w.body(b)->linearVelocity().length(), 0.01);
}

TEST(Contacts, BoxRestsOnGroundWithoutRotatingOrSinking) {
  World w;
  addGround(w);
  BodyId b = addBox(w, Vec2(0, 0.6), 0.5, 0.5);
  for (int i = 0; i < 600; ++i) {
    w.step(kDt, 8, 3);
  }
  EXPECT_NEAR(w.body(b)->position().y, 0.5, 0.01);
  EXPECT_NEAR(w.body(b)->angle(), 0.0, 1e-3);
  EXPECT_NEAR(w.body(b)->position().x, 0.0, 1e-3);
}

TEST(Contacts, TenBoxStackSettlesAndStaysStandingForTenSeconds) {
  World w;
  addGround(w);
  std::vector<BodyId> boxes;
  for (int i = 0; i < 10; ++i) {
    boxes.push_back(addBox(w, Vec2(0, 0.5 + i), 0.5, 0.5));
  }
  for (int s = 0; s < 1200; ++s) {
    w.step(kDt, 8, 3);
  }
  for (int i = 0; i < 10; ++i) {
    const RigidBody* b = w.body(boxes[i]);
    EXPECT_NEAR(b->position().x, 0.0, 0.05) << "box " << i;
    EXPECT_NEAR(b->position().y, 0.5 + i, 0.1) << "box " << i;
    EXPECT_NEAR(b->angle(), 0.0, 0.05) << "box " << i;
    EXPECT_LT(b->linearVelocity().length(), 0.1) << "box " << i;
  }
}

TEST(Contacts, StackWithInitialGapsAndOffsetsSettlesWithoutToppling) {
  World w;
  addGround(w);
  std::vector<BodyId> boxes;
  for (int i = 0; i < 6; ++i) {
    real offset = 0.02 * ((i % 2) * 2 - 1);
    boxes.push_back(addBox(w, Vec2(offset, 0.55 + 1.05 * i), 0.5, 0.5));
  }
  for (int s = 0; s < 1200; ++s) {
    w.step(kDt, 8, 3);
  }
  for (int i = 0; i < 6; ++i) {
    const RigidBody* b = w.body(boxes[i]);
    EXPECT_NEAR(b->position().y, 0.5 + i, 0.1) << "box " << i;
    EXPECT_LT(std::fabs(b->position().x), 0.1) << "box " << i;
  }
}

TEST(Contacts, StackIsStableWithoutWarmStartingToo) {
  World w;
  w.setWarmStarting(false);
  addGround(w);
  std::vector<BodyId> boxes;
  for (int i = 0; i < 3; ++i) {
    boxes.push_back(addBox(w, Vec2(0, 0.5 + i), 0.5, 0.5));
  }
  for (int s = 0; s < 600; ++s) {
    w.step(kDt, 10, 3);
  }
  EXPECT_NEAR(w.body(boxes[2])->position().y, 2.5, 0.1);
}

// ---- restitution -------------------------------------------------------------------

TEST(Contacts, PerfectlyElasticBallBouncesToAboutTheSameHeight) {
  World w;
  addGround(w, 0.3, 1.0);
  const real r = 0.25;
  BodyId b = addBall(w, Vec2(0, 5.0), r, BodyType::Dynamic, 0.3, 1.0);
  real apex = 0;
  bool bounced = false;
  real prevVy = 0;
  for (int s = 0; s < 1200 && !(bounced && prevVy > 0 && w.body(b)->linearVelocity().y <= 0); ++s) {
    prevVy = w.body(b)->linearVelocity().y;
    w.step(kDt, 8, 3);
    real vy = w.body(b)->linearVelocity().y;
    if (prevVy < 0 && vy > 0) {
      bounced = true;
    }
    if (bounced) {
      apex = maxReal(apex, w.body(b)->position().y);
    }
  }
  ASSERT_TRUE(bounced);
  EXPECT_NEAR(apex - r, 5.0 - r, 0.05 * (5.0 - r));
}

TEST(Contacts, ZeroRestitutionBallDoesNotBounce) {
  World w;
  addGround(w);
  BodyId b = addBall(w, Vec2(0, 3.0), 0.25);
  real maxYAfterImpact = 0;
  bool landed = false;
  for (int s = 0; s < 600; ++s) {
    w.step(kDt, 8, 3);
    if (w.body(b)->position().y < 0.27) {
      landed = true;
    }
    if (landed) {
      maxYAfterImpact = maxReal(maxYAfterImpact, w.body(b)->position().y);
    }
  }
  ASSERT_TRUE(landed);
  EXPECT_LT(maxYAfterImpact, 0.3);
}

TEST(Contacts, RestitutionIsIgnoredBelowVelocityThreshold) {
  World w;
  addGround(w, 0.3, 1.0);
  BodyId b = addBall(w, Vec2(0, 0.25 + 0.02), 0.25, BodyType::Dynamic, 0.3, 1.0);  // impact speed ~ 0.6 m/s
  real maxY = 0;
  for (int s = 0; s < 240; ++s) {
    w.step(kDt, 8, 3);
    maxY = maxReal(maxY, w.body(b)->position().y);
  }
  EXPECT_LT(maxY, 0.25 + 0.03);
}

TEST(Contacts, ElasticHeadOnCollisionOfEqualMassesExchangesVelocities) {
  World w(Vec2(0, 0));
  BodyId a = addBall(w, Vec2(-2, 0), 0.5, BodyType::Dynamic, 0.0, 1.0);
  BodyId b = addBall(w, Vec2(2, 0), 0.5, BodyType::Dynamic, 0.0, 1.0);
  w.setBodyVelocity(a, Vec2(3, 0), 0);
  w.setBodyVelocity(b, Vec2(-3, 0), 0);
  real p0 = w.body(a)->mass() * 3 - w.body(b)->mass() * 3;
  real e0 = w.kineticEnergy();
  for (int s = 0; s < 240; ++s) {
    w.step(kDt, 8, 3);
  }
  EXPECT_NEAR(w.body(a)->linearVelocity().x, -3.0, 0.05);
  EXPECT_NEAR(w.body(b)->linearVelocity().x, 3.0, 0.05);
  EXPECT_NEAR(w.body(a)->mass() * w.body(a)->linearVelocity().x + w.body(b)->mass() * w.body(b)->linearVelocity().x,
              p0, 1e-6);
  EXPECT_NEAR(w.kineticEnergy(), e0, 0.02 * e0);
}

// ---- friction ----------------------------------------------------------------------

TEST(Contacts, SlidingBoxStopsAtCoulombDistance) {
  World w;
  addGround(w, 0.5);
  BodyId b = addBox(w, Vec2(0, 0.5), 0.5, 0.5, BodyType::Dynamic, 0.5);
  w.setBodyVelocity(b, Vec2(5, 0), 0);
  for (int s = 0; s < 600; ++s) {
    w.step(kDt, 8, 3);
  }
  real expected = 25.0 / (2.0 * 0.5 * 9.81);
  EXPECT_NEAR(w.body(b)->position().x, expected, 0.1 * expected);
  EXPECT_LT(w.body(b)->linearVelocity().length(), 0.01);
}

TEST(Contacts, FrictionlessBoxKeepsSliding) {
  World w;
  addGround(w, 0.0);
  BodyId b = addBox(w, Vec2(0, 0.5), 0.5, 0.5, BodyType::Dynamic, 0.0);
  w.setBodyVelocity(b, Vec2(2, 0), 0);
  for (int s = 0; s < 240; ++s) {
    w.step(kDt, 8, 3);
  }
  EXPECT_NEAR(w.body(b)->linearVelocity().x, 2.0, 1e-3);
}

TEST(Contacts, FrictionIsGeometricMeanOfBothFixtures) {
  // mu = sqrt(0.8 * 0.2) = 0.4 -> stopping distance v^2 / (2 mu g)
  World w;
  addGround(w, 0.8);
  BodyId b = addBox(w, Vec2(0, 0.5), 0.5, 0.5, BodyType::Dynamic, 0.2);
  w.setBodyVelocity(b, Vec2(4, 0), 0);
  for (int s = 0; s < 600; ++s) {
    w.step(kDt, 8, 3);
  }
  real expected = 16.0 / (2.0 * 0.4 * 9.81);
  EXPECT_NEAR(w.body(b)->position().x, expected, 0.1 * expected);
}

namespace {

struct Slope {
  World w;
  BodyId box;
};

void buildSlope(Slope& s, real angle, real mu) {
  BodyDef gd;
  gd.type = BodyType::Static;
  gd.angle = angle;
  BodyId g = s.w.createBody(gd);
  Fixture gf;
  gf.shape = Shape::box(60, 1);
  gf.friction = mu;
  s.w.addShape(g, gf);
  // sit on the top surface (local y = 0.5) of the ground, box half height 0.5
  Vec2 pos = mul(Rot(angle), Vec2(0, 1.0));
  s.box = addBox(s.w, pos, 0.5, 0.5, BodyType::Dynamic, mu, 0.0, angle);
}

}  // namespace

TEST(Contacts, BoxOnSlopeBelowFrictionAngleStaysPut) {
  Slope s;
  real angle = 25.0 * kPi / 180.0;  // tan = 0.466 < mu
  buildSlope(s, angle, 0.6);
  Vec2 start = s.w.body(s.box)->position();
  for (int i = 0; i < 480; ++i) {
    s.w.step(kDt, 8, 3);
  }
  EXPECT_LT(distance(s.w.body(s.box)->position(), start), 0.05);
  EXPECT_LT(s.w.body(s.box)->linearVelocity().length(), 0.05);
}

TEST(Contacts, BoxOnSlopeAboveFrictionAngleSlidesWithExpectedAcceleration) {
  Slope s;
  real angle = 38.0 * kPi / 180.0;  // tan = 0.78 > mu
  buildSlope(s, angle, 0.6);
  for (int i = 0; i < 240; ++i) {
    s.w.step(kDt, 8, 3);
  }
  real a = 9.81 * (std::sin(angle) - 0.6 * std::cos(angle));
  real speed = s.w.body(s.box)->linearVelocity().length();
  EXPECT_NEAR(speed, a * 2.0, 0.15 * a * 2.0);
  EXPECT_LT(s.w.body(s.box)->linearVelocity().x, 0.0);  // slides down-slope
}

// ---- filters & joints -------------------------------------------------------------

namespace {

size_t contactsBetweenOverlappingBoxes(Filter fa, Filter fb) {
  World w(Vec2(0, 0));
  addBox(w, Vec2(0, 0), 0.5, 0.5, BodyType::Dynamic, 0.3, 0.0, 0, fa);
  addBox(w, Vec2(0.8, 0), 0.5, 0.5, BodyType::Dynamic, 0.3, 0.0, 0, fb);
  w.step(kDt, 8, 3);
  return w.contactCount();
}

}  // namespace

TEST(Filter, DefaultsCollide) {
  EXPECT_EQ(contactsBetweenOverlappingBoxes(Filter(), Filter()), 2u);
}

TEST(Filter, MaskExcludingOtherCategoryDisablesCollision) {
  Filter a;
  a.category = 0x2;
  a.mask = 0x4;  // does not include category 0x1 of b
  Filter b;
  EXPECT_EQ(contactsBetweenOverlappingBoxes(a, b), 0u);
  EXPECT_EQ(contactsBetweenOverlappingBoxes(b, a), 0u);
}

TEST(Filter, BothMasksMustAcceptTheOtherCategory) {
  Filter a;
  a.category = 0x2;
  a.mask = 0x4;
  Filter b;
  b.category = 0x4;
  b.mask = 0x2;
  EXPECT_EQ(contactsBetweenOverlappingBoxes(a, b), 2u);
}

TEST(Filter, NegativeSharedGroupNeverCollidesPositiveAlwaysDoes) {
  Filter neg;
  neg.group = -3;
  EXPECT_EQ(contactsBetweenOverlappingBoxes(neg, neg), 0u);
  Filter pos;
  pos.group = 5;
  pos.mask = 0;  // would normally reject everything
  EXPECT_EQ(contactsBetweenOverlappingBoxes(pos, pos), 2u);
  Filter other;
  other.group = -4;
  EXPECT_EQ(contactsBetweenOverlappingBoxes(neg, other), 2u);  // different groups fall back to masks
}

TEST(Filter, FilteredBodiesPassThroughEachOther) {
  World w;
  addGround(w);
  Filter ghost;
  ghost.category = 0x2;
  ghost.mask = 0x2;
  BodyId b = addBox(w, Vec2(0, 2), 0.5, 0.5, BodyType::Dynamic, 0.3, 0.0, 0, ghost);
  for (int s = 0; s < 240; ++s) {
    w.step(kDt, 8, 3);
  }
  EXPECT_LT(w.body(b)->position().y, -1.0);
}

TEST(CollideConnected, FalseDisablesCollisionBetweenJointedBodies) {
  for (int kind = 0; kind < 2; ++kind) {
    for (int cc = 0; cc < 2; ++cc) {
      World w(Vec2(0, 0));
      BodyId a = addBox(w, Vec2(0, 0), 0.5, 0.5);
      BodyId b = addBox(w, Vec2(0.8, 0), 0.5, 0.5);
      if (kind == 0) {
        RevoluteJointDef d;
        d.bodyA = a;
        d.bodyB = b;
        d.anchor = Vec2(0.4, 0);
        d.collideConnected = (cc == 1);
        ASSERT_TRUE(w.createRevoluteJoint(d).isValid());
      } else {
        DistanceJointDef d;
        d.bodyA = a;
        d.bodyB = b;
        d.anchorA = Vec2(0, 0);
        d.anchorB = Vec2(0.8, 0);
        d.collideConnected = (cc == 1);
        ASSERT_TRUE(w.createDistanceJoint(d).isValid());
      }
      w.step(kDt, 8, 3);
      if (cc == 1) {
        EXPECT_EQ(w.contactCount(), 2u) << "kind " << kind;
      } else {
        EXPECT_EQ(w.contactCount(), 0u) << "kind " << kind;
      }
    }
  }
}

TEST(CollideConnected, UnrelatedBodiesStillCollideWhenOthersAreJointed) {
  World w(Vec2(0, 0));
  BodyId a = addBox(w, Vec2(0, 0), 0.5, 0.5);
  BodyId b = addBox(w, Vec2(5, 0), 0.5, 0.5);
  addBox(w, Vec2(0.8, 0), 0.5, 0.5);
  RevoluteJointDef d;
  d.bodyA = a;
  d.bodyB = b;
  d.anchor = Vec2(2.5, 0);
  w.createRevoluteJoint(d);
  w.step(kDt, 8, 3);
  EXPECT_EQ(w.contactCount(), 2u);
}

// ---- kinematic / static / lifecycle ------------------------------------------------

TEST(Contacts, KinematicPlatformCarriesBoxUpward) {
  World w;
  BodyId plat = addBox(w, Vec2(0, -0.5), 2, 0.5, BodyType::Kinematic);
  w.setBodyVelocity(plat, Vec2(0, 1), 0);
  BodyId b = addBox(w, Vec2(0, 0.5), 0.5, 0.5);
  for (int s = 0; s < 240; ++s) {
    w.step(kDt, 8, 3);
  }
  EXPECT_NEAR(w.body(b)->position().y, 0.5 + 2.0, 0.1);
}

TEST(Contacts, StaticBodiesDoNotCollideWithEachOther) {
  World w;
  addBox(w, Vec2(0, 0), 1, 1, BodyType::Static);
  addBox(w, Vec2(0.5, 0), 1, 1, BodyType::Static);
  w.step(kDt, 8, 3);
  EXPECT_EQ(w.contactCount(), 0u);
}

TEST(Contacts, DestroyingABodyInContactIsSafe) {
  World w;
  addGround(w);
  BodyId b = addBox(w, Vec2(0, 0.5), 0.5, 0.5);
  w.step(kDt, 8, 3);
  ASSERT_GT(w.contactCount(), 0u);
  w.destroyBody(b);
  EXPECT_EQ(w.contactCount(), 0u);
  EXPECT_TRUE(w.contactPoints().empty());
  w.step(kDt, 8, 3);
  EXPECT_EQ(w.contactCount(), 0u);
}

TEST(Contacts, AddingShapesAfterContactsExistIsSafe) {
  World w;
  addGround(w);
  BodyId b = addBox(w, Vec2(0, 0.5), 0.5, 0.5);
  for (int i = 0; i < 10; ++i) {
    w.step(kDt, 8, 3);
  }
  ASSERT_GT(w.contactCount(), 0u);
  Fixture f;
  f.shape = Shape::circle(0.2, Vec2(0, 1.0));
  w.addShape(b, f);
  for (int i = 0; i < 10; ++i) {
    w.step(kDt, 8, 3);
  }
  EXPECT_GT(w.contactCount(), 0u);
}

TEST(Contacts, MultiFixtureBodyCollidesThroughEveryFixture) {
  World w;
  addGround(w);
  BodyDef def;
  def.position = Vec2(0, 1.0);
  BodyId b = w.createBody(def);
  Fixture left;
  left.shape = Shape::box(0.4, 0.4, Vec2(-1, 0));
  Fixture right;
  right.shape = Shape::circle(0.2, Vec2(1, 0));
  w.addShape(b, left);
  w.addShape(b, right);
  for (int s = 0; s < 600; ++s) {
    w.step(kDt, 8, 3);
  }
  // Lowest extents are the box bottom (-0.2) and the circle bottom (-0.2), so the body rests with its origin at ~0.2.
  EXPECT_NEAR(w.body(b)->position().y, 0.2, 0.03);
}

// ---- arm ---------------------------------------------------------------------------

TEST(Contacts, FiveLinkArmSwingingIntoGroundDoesNotTunnel) {
  World w;
  addGround(w);
  BodyId base = addBox(w, Vec2(0, 4.0), 0.1, 0.1, BodyType::Static);
  Filter armFilter;
  armFilter.group = -1;
  const real len = 1.0;
  const real thick = 0.1;
  BodyId prev = base;
  std::vector<BodyId> links;
  for (int i = 0; i < 5; ++i) {
    BodyId link = addBox(w, Vec2(len * (i + 0.5), 4.0), 0.5 * len, 0.5 * thick, BodyType::Dynamic, 0.3, 0.0, 0,
                         armFilter);
    RevoluteJointDef d;
    d.bodyA = prev;
    d.bodyB = link;
    d.anchor = Vec2(len * i, 4.0);
    ASSERT_TRUE(w.createRevoluteJoint(d).isValid());
    links.push_back(link);
    prev = link;
  }
  real lowest = 1e9;
  for (int s = 0; s < 600; ++s) {
    w.step(kDt, 8, 3);
    for (BodyId id : links) {
      lowest = minReal(lowest, w.body(id)->position().y);
    }
  }
  // Link centres must never sink meaningfully below the ground surface (y = 0), and the arm is touching it.
  EXPECT_GT(lowest, -0.15);
  EXPECT_GT(w.contactCount(), 0u);
  for (BodyId id : links) {
    EXPECT_GT(w.body(id)->position().y, -0.15);
  }
}

// ---- queries & buffers ------------------------------------------------------------

TEST(QueryPoint, FindsBodiesContainingThePoint) {
  World w;
  BodyId box = addBox(w, Vec2(2, 0), 1, 0.5);
  BodyId ball = addBall(w, Vec2(-3, 1), 0.5);
  EXPECT_EQ(w.queryPoint(Vec2(2.9, 0.4)), box);
  EXPECT_EQ(w.queryPoint(Vec2(-3, 1.4)), ball);
  EXPECT_FALSE(w.queryPoint(Vec2(0, 0)).isValid());
  EXPECT_FALSE(w.queryPoint(Vec2(3.1, 0)).isValid());
  EXPECT_FALSE(w.queryPoint(Vec2(-3.4, 1.4)).isValid());  // inside the ball's AABB, outside the disc
}

TEST(QueryPoint, RespectsBodyRotationAndStaticBodies) {
  World w;
  BodyId bar = addBox(w, Vec2(0, 0), 2, 0.1, BodyType::Static, 0.3, 0.0, kPi / 2);
  EXPECT_EQ(w.queryPoint(Vec2(0, 1.5)), bar);
  EXPECT_FALSE(w.queryPoint(Vec2(1.5, 0)).isValid());
}

TEST(QueryPoint, ReturnsTopmostMostRecentBodyAndIgnoresDestroyed) {
  World w;
  BodyId under = addBox(w, Vec2(0, 0), 1, 1);
  BodyId over = addBox(w, Vec2(0.5, 0), 1, 1);
  EXPECT_EQ(w.queryPoint(Vec2(0.2, 0)), over);
  w.destroyBody(over);
  EXPECT_EQ(w.queryPoint(Vec2(0.2, 0)), under);
  w.destroyBody(under);
  EXPECT_FALSE(w.queryPoint(Vec2(0.2, 0)).isValid());
}

TEST(ContactBuffer, RestingBoxReportsTwoPointsWithNormalFromAToB) {
  World w;
  addGround(w);  // created first -> body A
  addBox(w, Vec2(0, 0.5), 0.5, 0.5);
  w.step(kDt, 8, 3);
  ASSERT_EQ(w.contactCount(), 2u);
  const std::vector<float>& pts = w.contactPoints();
  ASSERT_EQ(pts.size(), 8u);
  for (int i = 0; i < 2; ++i) {
    EXPECT_NEAR(std::fabs(pts[4 * i + 0]), 0.5, 0.02);
    EXPECT_NEAR(pts[4 * i + 1], 0.0, 0.02);
    EXPECT_NEAR(pts[4 * i + 2], 0.0, 1e-6);
    EXPECT_NEAR(pts[4 * i + 3], 1.0, 1e-6);
  }
}

TEST(ContactBuffer, EmptyWhenNothingTouches) {
  World w;
  addBall(w, Vec2(0, 10), 0.5);
  w.step(kDt, 8, 3);
  EXPECT_EQ(w.contactCount(), 0u);
  EXPECT_TRUE(w.contactPoints().empty());
}

TEST(StepTiming, LastStepMsIsRecorded) {
  World w;
  EXPECT_EQ(w.lastStepMs(), 0.0);
  addGround(w);
  for (int i = 0; i < 300; ++i) {
    addBox(w, Vec2(0.0 + 0.9 * (i % 20) - 9, 0.5 + 1.1 * (i / 20)), 0.5, 0.5);
  }
  w.step(kDt, 8, 3);
  EXPECT_GT(w.lastStepMs(), 0.0);
  EXPECT_LT(w.lastStepMs(), 5000.0);
}

TEST(Contacts, ManyBodiesStaySane) {
  World w;
  addGround(w);
  addBox(w, Vec2(-12, 5), 0.5, 5, BodyType::Static);
  addBox(w, Vec2(12, 5), 0.5, 5, BodyType::Static);
  std::vector<BodyId> ids;
  for (int i = 0; i < 200; ++i) {
    Vec2 p(-8 + 1.1 * (i % 15), 1.0 + 1.1 * (i / 15));
    if (i % 2 == 0) {
      ids.push_back(addBox(w, p, 0.4, 0.4));
    } else {
      ids.push_back(addBall(w, p, 0.4));
    }
  }
  for (int s = 0; s < 600; ++s) {
    w.step(kDt, 8, 3);
  }
  for (BodyId id : ids) {
    Vec2 p = w.body(id)->position();
    EXPECT_TRUE(std::isfinite(p.x) && std::isfinite(p.y));
    EXPECT_GT(p.y, -0.2);
    EXPECT_LT(std::fabs(p.x), 12.0);
  }
}

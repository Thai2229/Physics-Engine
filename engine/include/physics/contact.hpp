#pragma once

#include "body.hpp"
#include "collision.hpp"
#include "handle.hpp"
#include "joint.hpp"
#include "math.hpp"

namespace physics {

constexpr real kVelocityThreshold = 1.0;  ///< closing speed (m/s) below which restitution is ignored
constexpr real kBaumgarte = 0.2;

/// One touching fixture pair and its sequential-impulse constraint. Normal
/// points from body A to body B; friction is solved before the normal impulse.
class Contact {
 public:
  Contact(BodyId idA, RigidBody* a, int fixtureA, BodyId idB, RigidBody* b, int fixtureB);

  BodyId bodyIdA() const { return idA_; }
  BodyId bodyIdB() const { return idB_; }
  RigidBody* bodyA() const { return bodyA_; }
  RigidBody* bodyB() const { return bodyB_; }
  int fixtureIndexA() const { return fixtureA_; }
  int fixtureIndexB() const { return fixtureB_; }
  const Fixture& fixtureA() const { return bodyA_->shapes()[static_cast<std::size_t>(fixtureA_)]; }
  const Fixture& fixtureB() const { return bodyB_->shapes()[static_cast<std::size_t>(fixtureB_)]; }

  Manifold& manifold() { return manifold_; }
  const Manifold& manifold() const { return manifold_; }
  real friction() const { return friction_; }
  real restitution() const { return restitution_; }

  /// Evaluates the manifold at the bodies' current transforms.
  WorldManifold worldManifold() const;

  /// Computes effective masses and restitution bias; must run for every contact before any warmStart().
  void initVelocityConstraints();
  void warmStart(const StepContext& ctx);
  void solveVelocityConstraints();
  /// Returns the smallest separation seen (negative = penetrating) after pushing the bodies apart.
  real solvePositionConstraints();

 private:
  struct Point {
    Vec2 rA;
    Vec2 rB;
    real normalMass = 0;
    real tangentMass = 0;
    real velocityBias = 0;
  };

  BodyId idA_;
  BodyId idB_;
  RigidBody* bodyA_;
  RigidBody* bodyB_;
  int fixtureA_;
  int fixtureB_;
  Manifold manifold_;
  real friction_ = 0;
  real restitution_ = 0;

  Point points_[2];
  Vec2 normal_;
};

}  // namespace physics

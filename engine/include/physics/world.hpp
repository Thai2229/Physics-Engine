#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "body.hpp"
#include "contact.hpp"
#include "handle.hpp"
#include "joint.hpp"
#include "math.hpp"
#include "slot_map.hpp"

namespace physics {

class World {
 public:
  explicit World(Vec2 gravity = Vec2(0, -9.81));

  Vec2 gravity() const { return gravity_; }
  void setGravity(Vec2 g) { gravity_ = g; }

  // ---- bodies ----
  BodyId createBody(const BodyDef& def);
  /// Also destroys every joint attached to the body.
  bool destroyBody(BodyId id);
  RigidBody* body(BodyId id);
  const RigidBody* body(BodyId id) const;
  bool addShape(BodyId id, const Fixture& fixture);
  bool setBodyTransform(BodyId id, Vec2 position, real angle);
  bool setBodyVelocity(BodyId id, Vec2 linearVelocity, real angularVelocity);

  // ---- joints (invalid id returned when bodies are missing/identical/both non-dynamic) ----
  JointId createRevoluteJoint(const RevoluteJointDef& def);
  JointId createDistanceJoint(const DistanceJointDef& def);
  /// Body must be dynamic. Defaults give a firm but soft drag spring.
  JointId createMouseJoint(BodyId body, Vec2 worldPoint, real maxForce, real hertz = 5.0, real dampingRatio = 0.7);
  bool destroyJoint(JointId id);
  Joint* joint(JointId id);
  const Joint* joint(JointId id) const;
  /// Typed lookups: null when the id is stale or refers to another joint kind.
  RevoluteJoint* revoluteJoint(JointId id);
  DistanceJoint* distanceJoint(JointId id);
  MouseJoint* mouseJoint(JointId id);
  bool setMouseTarget(JointId id, Vec2 target);

  // ---- stepping ----
  /// One explicit step (no accumulator). dt <= 0 is ignored.
  void step(real dt, int velocityIterations, int positionIterations);
  /// Fixed-step accumulator: runs floor(accumulated / fixedTimeStep) steps and
  /// returns how many. frameDt is clamped to maxFrameTime; negatives are ignored.
  int advance(real frameDt);

  real fixedTimeStep() const { return fixedTimeStep_; }
  void setFixedTimeStep(real dt);
  real maxFrameTime() const { return maxFrameTime_; }
  void setMaxFrameTime(real t) { maxFrameTime_ = t; }
  real accumulator() const { return accumulator_; }
  int velocityIterations() const { return velocityIterations_; }
  int positionIterations() const { return positionIterations_; }
  void setVelocityIterations(int n) { velocityIterations_ = n; }
  void setPositionIterations(int n) { positionIterations_ = n; }
  bool warmStarting() const { return warmStarting_; }
  void setWarmStarting(bool enabled) { warmStarting_ = enabled; }

  // ---- stats ----
  std::size_t bodyCount() const { return bodies_.size(); }
  std::size_t jointCount() const { return joints_.size(); }
  real kineticEnergy() const;
  real potentialEnergy() const;

  /// Static or dynamic body (dynamic preferred over static, else last in slot order) with a shape containing p; invalid id if none.
  BodyId queryPoint(Vec2 p) const;
  /// Contact points of the last step (one per manifold point, not per body pair).
  std::size_t contactCount() const;
  /// Wall time of the last step() in milliseconds; after advance(), the total of its sub-steps.
  double lastStepMs() const { return lastStepMs_; }

  // ---- contiguous buffers for bindings ----
  // Each call rebuilds the buffer from current state (O(n)); the returned
  // reference/data pointer is valid until the next call to the same accessor
  // or to any mutating method.
  /// [x, y, angle] per body (body origin), in bodyIds() order.
  const std::vector<float>& bodyStates() const;
  /// Packed BodyId::toInt() per live body, in slot order.
  const std::vector<std::int32_t>& bodyIds() const;
  /// [ax, ay, bx, by] per joint (world anchors), in jointIds() order.
  const std::vector<float>& jointAnchors() const;
  const std::vector<std::int32_t>& jointIds() const;
  /// [px, py, nx, ny] per contact point of the last step; normal points from body A to body B.
  const std::vector<float>& contactPoints();

 private:
  JointId addJoint(std::unique_ptr<Joint> joint);
  /// Rebuilds contacts_ from the current transforms, carrying accumulated impulses over by feature id.
  void updateContacts();
  void stepImpl(real dt, int velocityIterations, int positionIterations);

  Vec2 gravity_;
  SlotMap<RigidBody, BodyId> bodies_;
  SlotMap<Joint, JointId> joints_;
  std::vector<Contact> contacts_;

  real fixedTimeStep_ = 1.0 / 120.0;
  real maxFrameTime_ = 0.1;
  real accumulator_ = 0;
  real lastDt_ = 0;
  int velocityIterations_ = 8;
  int positionIterations_ = 3;
  bool warmStarting_ = true;
  double lastStepMs_ = 0;

  mutable std::vector<float> bodyStates_;
  mutable std::vector<std::int32_t> bodyIds_;
  mutable std::vector<float> jointAnchors_;
  mutable std::vector<std::int32_t> jointIds_;
  std::vector<float> contactPoints_;
};

}  // namespace physics

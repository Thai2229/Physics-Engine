#pragma once

#include "body.hpp"
#include "handle.hpp"
#include "math.hpp"

namespace physics {

enum class JointType { Revolute, Distance, Mouse };

constexpr real kLinearSlop = 0.005;
constexpr real kAngularSlop = 0.5 * kPi / 180.0;
constexpr real kPositionTolerance = 1e-5;  ///< position solver stops iterating below this error (m)
constexpr real kMaxLinearCorrection = 0.2;
constexpr real kMaxAngularCorrection = 8.0 * kPi / 180.0;

struct StepContext {
  real dt = 0;
  real invDt = 0;
  real dtRatio = 1;  ///< dt / previous dt, scales warm-start impulses
  bool warmStarting = true;
};

class Joint {
 public:
  virtual ~Joint() = default;

  JointType type() const { return type_; }
  RigidBody* bodyA() const { return bodyA_; }  ///< null for mouse joints
  RigidBody* bodyB() const { return bodyB_; }
  bool collideConnected() const { return collideConnected_; }

  /// World-space attachment points (equal when the joint is satisfied).
  virtual Vec2 anchorA() const = 0;
  virtual Vec2 anchorB() const = 0;

  virtual void initVelocityConstraints(const StepContext& ctx) = 0;
  virtual void solveVelocityConstraints(const StepContext& ctx) = 0;
  /// Returns true when the position error is within tolerance.
  virtual bool solvePositionConstraints() = 0;

 protected:
  Joint(JointType type, RigidBody* a, RigidBody* b, bool collideConnected)
      : type_(type), bodyA_(a), bodyB_(b), collideConnected_(collideConnected) {}

 private:
  JointType type_;
  RigidBody* bodyA_;
  RigidBody* bodyB_;
  bool collideConnected_;
};

struct RevoluteJointDef {
  BodyId bodyA;
  BodyId bodyB;
  Vec2 anchor;  ///< world-space pin position
  /// Angles are relative to the relative body angle at creation time.
  bool enableLimit = false;
  real lowerAngle = 0;
  real upperAngle = 0;
  bool enableMotor = false;
  real motorSpeed = 0;
  real maxMotorTorque = 0;
  bool enableSpring = false;
  real springHertz = 0;
  real springDampingRatio = 0;
  real springRestAngle = 0;
  bool collideConnected = false;
};

class RevoluteJoint : public Joint {
 public:
  RevoluteJoint(const RevoluteJointDef& def, RigidBody* a, RigidBody* b);

  Vec2 anchorA() const override;
  Vec2 anchorB() const override;
  void initVelocityConstraints(const StepContext& ctx) override;
  void solveVelocityConstraints(const StepContext& ctx) override;
  bool solvePositionConstraints() override;

  real jointAngle() const;  ///< relative angle minus the creation-time reference
  real jointSpeed() const;
  Vec2 impulse() const { return impulse_; }  ///< accumulated point-constraint impulse
  real motorImpulse() const { return motorImpulse_; }

  bool limitEnabled() const { return enableLimit_; }
  real lowerAngle() const { return lowerAngle_; }
  real upperAngle() const { return upperAngle_; }
  void setLimitEnabled(bool enabled);
  void setLimits(real lower, real upper);
  void setMotorEnabled(bool enabled);
  void setMotorSpeed(real speed) { motorSpeed_ = speed; }
  void setMaxMotorTorque(real torque) { maxMotorTorque_ = torque; }
  void setSpring(bool enabled, real hertz, real dampingRatio);
  void setSpringRestAngle(real angle) { springRestAngle_ = angle; }

 private:
  Vec2 localAnchorA_;
  Vec2 localAnchorB_;
  real referenceAngle_;
  bool enableLimit_;
  real lowerAngle_;
  real upperAngle_;
  bool enableMotor_;
  real motorSpeed_;
  real maxMotorTorque_;
  bool enableSpring_;
  real springHertz_;
  real springDampingRatio_;
  real springRestAngle_;

  Vec2 impulse_;
  real motorImpulse_ = 0;
  real lowerImpulse_ = 0;
  real upperImpulse_ = 0;
  real springImpulse_ = 0;

  // per-step cache
  Vec2 rA_;
  Vec2 rB_;
  real mA_ = 0;
  real mB_ = 0;
  real iA_ = 0;
  real iB_ = 0;
  real axialMass_ = 0;
  real angle_ = 0;
  real springGamma_ = 0;
  real springBias_ = 0;
  real springMass_ = 0;
  Mat22 K_;
};

struct DistanceJointDef {
  BodyId bodyA;
  BodyId bodyB;
  Vec2 anchorA;  ///< world-space
  Vec2 anchorB;  ///< world-space
  real length = -1;  ///< rest length; negative means use the initial anchor distance
  real hertz = 0;  ///< 0 = rigid rod, > 0 = soft spring
  real dampingRatio = 0;
  bool collideConnected = false;
};

class DistanceJoint : public Joint {
 public:
  DistanceJoint(const DistanceJointDef& def, RigidBody* a, RigidBody* b);

  Vec2 anchorA() const override;
  Vec2 anchorB() const override;
  void initVelocityConstraints(const StepContext& ctx) override;
  void solveVelocityConstraints(const StepContext& ctx) override;
  bool solvePositionConstraints() override;

  real length() const { return length_; }
  void setLength(real length);
  void setSpring(real hertz, real dampingRatio);
  real impulse() const { return impulse_; }

 private:
  Vec2 localAnchorA_;
  Vec2 localAnchorB_;
  real length_;
  real hertz_;
  real dampingRatio_;

  real impulse_ = 0;

  Vec2 u_;
  real length0_ = 0;  ///< current anchor distance at step start
  Vec2 rA_;
  Vec2 rB_;
  real mA_ = 0;
  real mB_ = 0;
  real iA_ = 0;
  real iB_ = 0;
  real mass_ = 0;
  real softMass_ = 0;
  real gamma_ = 0;
  real bias_ = 0;
};

/// Soft spring pulling a point on bodyB toward a world target (bodyA is null).
class MouseJoint : public Joint {
 public:
  MouseJoint(RigidBody* body, Vec2 worldPoint, real maxForce, real hertz, real dampingRatio);

  Vec2 anchorA() const override { return target_; }
  Vec2 anchorB() const override;
  void initVelocityConstraints(const StepContext& ctx) override;
  void solveVelocityConstraints(const StepContext& ctx) override;
  bool solvePositionConstraints() override { return true; }

  void setTarget(Vec2 target) { target_ = target; }
  Vec2 target() const { return target_; }
  void setMaxForce(real f) { maxForce_ = f; }
  real maxForce() const { return maxForce_; }

 private:
  Vec2 target_;
  Vec2 localAnchorB_;
  real maxForce_;
  real hertz_;
  real dampingRatio_;

  Vec2 impulse_;

  Vec2 rB_;
  real mB_ = 0;
  real iB_ = 0;
  Mat22 mass_;
  Vec2 C_;
  real gamma_ = 0;
};

}  // namespace physics

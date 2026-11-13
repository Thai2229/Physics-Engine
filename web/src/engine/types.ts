export type BodyType = 'static' | 'dynamic' | 'kinematic';

export type ShapeDef =
  | { kind: 'circle'; radius: number }
  | { kind: 'box'; w: number; h: number }
  | { kind: 'polygon'; vertices: { x: number; y: number }[] };

export interface BodyDef {
  type: BodyType;
  x: number;
  y: number;
  angle: number;
  shape: ShapeDef;
  density: number;
  friction: number;
  restitution: number;
  linearDamping: number;
  angularDamping: number;
  collisionGroup: number;
}

export interface RevoluteJointDef {
  bodyA: number;
  bodyB: number;
  anchorX: number;
  anchorY: number;
  enableLimit: boolean;
  lowerAngle: number;
  upperAngle: number;
  enableMotor: boolean;
  motorSpeed: number;
  maxMotorTorque: number;
  collideConnected: boolean;
}

/**
 * Not specified in PLAN.md section 4 (only referenced by name). Anchors are in
 * world coordinates at creation time. length <= 0 means "use the initial
 * anchor distance". frequencyHz = 0 means a rigid rod, > 0 a soft spring.
 */
export interface DistanceJointDef {
  bodyA: number;
  bodyB: number;
  anchorAX: number;
  anchorAY: number;
  anchorBX: number;
  anchorBY: number;
  length: number;
  frequencyHz: number;
  dampingRatio: number;
  collideConnected: boolean;
}

export interface WorldStats {
  bodies: number;
  contacts: number;
  stepMs: number;
  kineticEnergy: number;
  potentialEnergy: number;
}

export interface PhysicsWorld {
  setGravity(x: number, y: number): void;
  addBody(def: BodyDef): number;
  removeBody(id: number): void;
  setBodyTransform(id: number, x: number, y: number, angle: number): void;
  setBodyVelocity(id: number, vx: number, vy: number, w: number): void;
  addRevoluteJoint(def: RevoluteJointDef): number;
  updateRevoluteJoint(id: number, patch: Partial<RevoluteJointDef>): void;
  addDistanceJoint(def: DistanceJointDef): number;
  removeJoint(id: number): void;
  createMouseJoint(bodyId: number, x: number, y: number, maxForce: number): number;
  setMouseTarget(jointId: number, x: number, y: number): void;
  queryPoint(x: number, y: number): number;
  step(frameDt: number): void;
  /** Float32Array view: [x, y, angle] per body, in getBodyIds() order. Zero-copy into WASM memory. */
  getBodyStates(): Float32Array;
  getBodyIds(): Int32Array;
  /** [ax, ay, bx, by] per joint. */
  getJointAnchors(): Float32Array;
  /** [px, py, nx, ny] per contact point. */
  getContacts(): Float32Array;
  getStats(): WorldStats;
  destroy(): void;
}

export function defaultBodyDef(partial: Partial<BodyDef> & Pick<BodyDef, 'shape'>): BodyDef {
  return {
    type: 'dynamic',
    x: 0,
    y: 0,
    angle: 0,
    density: 1,
    friction: 0.5,
    restitution: 0,
    linearDamping: 0,
    angularDamping: 0,
    collisionGroup: 0,
    ...partial,
  };
}

export function defaultRevoluteDef(
  partial: Partial<RevoluteJointDef> & Pick<RevoluteJointDef, 'bodyA' | 'bodyB' | 'anchorX' | 'anchorY'>,
): RevoluteJointDef {
  return {
    enableLimit: false,
    lowerAngle: 0,
    upperAngle: 0,
    enableMotor: false,
    motorSpeed: 0,
    maxMotorTorque: 0,
    collideConnected: false,
    ...partial,
  };
}

import type {
  BodyDef,
  DistanceJointDef,
  PhysicsWorld,
  RevoluteJointDef,
  WorldStats,
} from './types';

/**
 * Pure-TS stand-in for the WASM engine: semi-implicit Euler + sequential
 * impulses for revolute / distance / mouse joints. No collision detection.
 */

const FIXED_DT = 1 / 120;
const MAX_SUBSTEPS = 16;
const VELOCITY_ITERATIONS = 12;
const BAUMGARTE = 0.2;

interface Body {
  id: number;
  def: BodyDef;
  x: number;
  y: number;
  angle: number;
  vx: number;
  vy: number;
  w: number;
  mass: number;
  inertia: number;
  invMass: number;
  invI: number;
}

interface RevoluteJoint {
  kind: 'revolute';
  id: number;
  a: Body;
  b: Body;
  laX: number;
  laY: number;
  lbX: number;
  lbY: number;
  refAngle: number;
  def: RevoluteJointDef;
  impulseX: number;
  impulseY: number;
  motorImpulse: number;
  lowerImpulse: number;
  upperImpulse: number;
  // per-step
  rax: number;
  ray: number;
  rbx: number;
  rby: number;
  k11: number;
  k12: number;
  k22: number;
  axialMass: number;
}

interface DistanceJoint {
  kind: 'distance';
  id: number;
  a: Body;
  b: Body;
  laX: number;
  laY: number;
  lbX: number;
  lbY: number;
  length: number;
  frequencyHz: number;
  dampingRatio: number;
  impulse: number;
  rax: number;
  ray: number;
  rbx: number;
  rby: number;
  ux: number;
  uy: number;
  softMass: number;
  gamma: number;
  bias: number;
}

interface MouseJoint {
  kind: 'mouse';
  id: number;
  b: Body;
  lbX: number;
  lbY: number;
  targetX: number;
  targetY: number;
  maxForce: number;
  impulseX: number;
  impulseY: number;
  rbx: number;
  rby: number;
  gamma: number;
  beta: number;
  k11: number;
  k12: number;
  k22: number;
}

type Joint = RevoluteJoint | DistanceJoint | MouseJoint;

function polygonMass(vertices: { x: number; y: number }[], density: number): { mass: number; inertia: number } {
  let area = 0;
  let inertia = 0;
  for (let i = 0; i < vertices.length; i++) {
    const p = vertices[i];
    const q = vertices[(i + 1) % vertices.length];
    const cross = p.x * q.y - p.y * q.x;
    area += 0.5 * cross;
    inertia += (cross / 12) * (p.x * p.x + p.x * q.x + q.x * q.x + p.y * p.y + p.y * q.y + q.y * q.y);
  }
  return { mass: Math.abs(area) * density, inertia: Math.abs(inertia) * density };
}

function massOf(def: BodyDef): { mass: number; inertia: number } {
  const shape = def.shape;
  if (shape.kind === 'circle') {
    const mass = Math.PI * shape.radius * shape.radius * def.density;
    return { mass, inertia: 0.5 * mass * shape.radius * shape.radius };
  }
  if (shape.kind === 'box') {
    const mass = shape.w * shape.h * def.density;
    return { mass, inertia: (mass * (shape.w * shape.w + shape.h * shape.h)) / 12 };
  }
  return polygonMass(shape.vertices, def.density);
}

function pointInBody(body: Body, px: number, py: number): boolean {
  const c = Math.cos(body.angle);
  const s = Math.sin(body.angle);
  const dx = px - body.x;
  const dy = py - body.y;
  const lx = c * dx + s * dy;
  const ly = -s * dx + c * dy;
  const shape = body.def.shape;
  if (shape.kind === 'circle') {
    return lx * lx + ly * ly <= shape.radius * shape.radius;
  }
  if (shape.kind === 'box') {
    return Math.abs(lx) <= shape.w / 2 && Math.abs(ly) <= shape.h / 2;
  }
  const v = shape.vertices;
  let sign = 0;
  for (let i = 0; i < v.length; i++) {
    const p = v[i];
    const q = v[(i + 1) % v.length];
    const cross = (q.x - p.x) * (ly - p.y) - (q.y - p.y) * (lx - p.x);
    if (cross !== 0) {
      if (sign !== 0 && Math.sign(cross) !== sign) {
        return false;
      }
      sign = Math.sign(cross);
    }
  }
  return true;
}

export class MockEngine implements PhysicsWorld {
  private bodies = new Map<number, Body>();
  private joints = new Map<number, Joint>();
  private nextBodyId = 0;
  private nextJointId = 0;
  private gx = 0;
  private gy = -9.81;
  private accumulator = 0;
  private stepMs = 0;
  private statesBuf = new Float32Array(0);
  private idsBuf = new Int32Array(0);
  private anchorsBuf = new Float32Array(0);
  private emptyContacts = new Float32Array(0);

  setGravity(x: number, y: number): void {
    this.gx = x;
    this.gy = y;
  }

  addBody(def: BodyDef): number {
    const id = this.nextBodyId++;
    const m = massOf(def);
    const dynamic = def.type === 'dynamic' && m.mass > 0;
    let invMass = 0;
    let invI = 0;
    if (dynamic) {
      invMass = 1 / m.mass;
    }
    if (dynamic && m.inertia > 0) {
      invI = 1 / m.inertia;
    }
    this.bodies.set(id, {
      id,
      def,
      x: def.x,
      y: def.y,
      angle: def.angle,
      vx: 0,
      vy: 0,
      w: 0,
      mass: m.mass,
      inertia: m.inertia,
      invMass,
      invI,
    });
    return id;
  }

  removeBody(id: number): void {
    this.bodies.delete(id);
    for (const [jid, j] of this.joints) {
      let attached = false;
      if (j.kind === 'mouse') {
        attached = j.b.id === id;
      } else {
        attached = j.a.id === id || j.b.id === id;
      }
      if (attached) {
        this.joints.delete(jid);
      }
    }
  }

  setBodyTransform(id: number, x: number, y: number, angle: number): void {
    const b = this.bodies.get(id);
    if (b) {
      b.x = x;
      b.y = y;
      b.angle = angle;
    }
  }

  setBodyVelocity(id: number, vx: number, vy: number, w: number): void {
    const b = this.bodies.get(id);
    if (b) {
      b.vx = vx;
      b.vy = vy;
      b.w = w;
    }
  }

  private toLocal(b: Body, wx: number, wy: number): { x: number; y: number } {
    const c = Math.cos(b.angle);
    const s = Math.sin(b.angle);
    const dx = wx - b.x;
    const dy = wy - b.y;
    return { x: c * dx + s * dy, y: -s * dx + c * dy };
  }

  addRevoluteJoint(def: RevoluteJointDef): number {
    const a = this.bodies.get(def.bodyA);
    const b = this.bodies.get(def.bodyB);
    if (!a || !b) {
      return -1;
    }
    const id = this.nextJointId++;
    const la = this.toLocal(a, def.anchorX, def.anchorY);
    const lb = this.toLocal(b, def.anchorX, def.anchorY);
    this.joints.set(id, {
      kind: 'revolute',
      id,
      a,
      b,
      laX: la.x,
      laY: la.y,
      lbX: lb.x,
      lbY: lb.y,
      refAngle: b.angle - a.angle,
      def: { ...def },
      impulseX: 0,
      impulseY: 0,
      motorImpulse: 0,
      lowerImpulse: 0,
      upperImpulse: 0,
      rax: 0,
      ray: 0,
      rbx: 0,
      rby: 0,
      k11: 0,
      k12: 0,
      k22: 0,
      axialMass: 0,
    });
    return id;
  }

  updateRevoluteJoint(id: number, patch: Partial<RevoluteJointDef>): void {
    const j = this.joints.get(id);
    if (j && j.kind === 'revolute') {
      j.def = { ...j.def, ...patch };
    }
  }

  addDistanceJoint(def: DistanceJointDef): number {
    const a = this.bodies.get(def.bodyA);
    const b = this.bodies.get(def.bodyB);
    if (!a || !b) {
      return -1;
    }
    const id = this.nextJointId++;
    const la = this.toLocal(a, def.anchorAX, def.anchorAY);
    const lb = this.toLocal(b, def.anchorBX, def.anchorBY);
    let length = def.length;
    if (length <= 0) {
      length = Math.hypot(def.anchorBX - def.anchorAX, def.anchorBY - def.anchorAY);
    }
    this.joints.set(id, {
      kind: 'distance',
      id,
      a,
      b,
      laX: la.x,
      laY: la.y,
      lbX: lb.x,
      lbY: lb.y,
      length: Math.max(length, 1e-4),
      frequencyHz: def.frequencyHz,
      dampingRatio: def.dampingRatio,
      impulse: 0,
      rax: 0,
      ray: 0,
      rbx: 0,
      rby: 0,
      ux: 1,
      uy: 0,
      softMass: 0,
      gamma: 0,
      bias: 0,
    });
    return id;
  }

  removeJoint(id: number): void {
    this.joints.delete(id);
  }

  createMouseJoint(bodyId: number, x: number, y: number, maxForce: number): number {
    const b = this.bodies.get(bodyId);
    if (!b) {
      return -1;
    }
    const id = this.nextJointId++;
    const lb = this.toLocal(b, x, y);
    this.joints.set(id, {
      kind: 'mouse',
      id,
      b,
      lbX: lb.x,
      lbY: lb.y,
      targetX: x,
      targetY: y,
      maxForce,
      impulseX: 0,
      impulseY: 0,
      rbx: 0,
      rby: 0,
      gamma: 0,
      beta: 0,
      k11: 0,
      k12: 0,
      k22: 0,
    });
    return id;
  }

  setMouseTarget(jointId: number, x: number, y: number): void {
    const j = this.joints.get(jointId);
    if (j && j.kind === 'mouse') {
      j.targetX = x;
      j.targetY = y;
    }
  }

  queryPoint(x: number, y: number): number {
    let found = -1;
    for (const b of this.bodies.values()) {
      if (pointInBody(b, x, y)) {
        found = b.id;
        if (b.invMass > 0) {
          return b.id;
        }
      }
    }
    return found;
  }

  step(frameDt: number): void {
    const t0 = performance.now();
    this.accumulator += Math.max(0, frameDt);
    let n = 0;
    while (this.accumulator >= FIXED_DT && n < MAX_SUBSTEPS) {
      this.substep(FIXED_DT);
      this.accumulator -= FIXED_DT;
      n++;
    }
    if (n === MAX_SUBSTEPS) {
      this.accumulator = 0;
    }
    this.stepMs = performance.now() - t0;
  }

  private substep(dt: number): void {
    for (const b of this.bodies.values()) {
      if (b.invMass > 0) {
        b.vx += this.gx * dt;
        b.vy += this.gy * dt;
        b.vx /= 1 + dt * b.def.linearDamping;
        b.vy /= 1 + dt * b.def.linearDamping;
        b.w /= 1 + dt * b.def.angularDamping;
      }
    }
    const joints = Array.from(this.joints.values());
    for (const j of joints) {
      this.prepare(j, dt);
    }
    for (const j of joints) {
      this.warmStart(j);
    }
    for (let it = 0; it < VELOCITY_ITERATIONS; it++) {
      for (const j of joints) {
        this.solve(j, dt);
      }
    }
    for (const b of this.bodies.values()) {
      if (b.def.type !== 'static') {
        b.x += b.vx * dt;
        b.y += b.vy * dt;
        b.angle += b.w * dt;
      }
    }
  }

  private prepare(j: Joint, dt: number): void {
    if (j.kind === 'revolute') {
      const { a, b } = j;
      const ca = Math.cos(a.angle);
      const sa = Math.sin(a.angle);
      const cb = Math.cos(b.angle);
      const sb = Math.sin(b.angle);
      j.rax = ca * j.laX - sa * j.laY;
      j.ray = sa * j.laX + ca * j.laY;
      j.rbx = cb * j.lbX - sb * j.lbY;
      j.rby = sb * j.lbX + cb * j.lbY;
      const mA = a.invMass;
      const mB = b.invMass;
      const iA = a.invI;
      const iB = b.invI;
      j.k11 = mA + mB + iA * j.ray * j.ray + iB * j.rby * j.rby;
      j.k12 = -iA * j.rax * j.ray - iB * j.rbx * j.rby;
      j.k22 = mA + mB + iA * j.rax * j.rax + iB * j.rbx * j.rbx;
      const iSum = iA + iB;
      j.axialMass = 0;
      if (iSum > 0) {
        j.axialMass = 1 / iSum;
      }
      if (!j.def.enableMotor) {
        j.motorImpulse = 0;
      }
      if (!j.def.enableLimit) {
        j.lowerImpulse = 0;
        j.upperImpulse = 0;
      }
    } else if (j.kind === 'distance') {
      const { a, b } = j;
      const ca = Math.cos(a.angle);
      const sa = Math.sin(a.angle);
      const cb = Math.cos(b.angle);
      const sb = Math.sin(b.angle);
      j.rax = ca * j.laX - sa * j.laY;
      j.ray = sa * j.laX + ca * j.laY;
      j.rbx = cb * j.lbX - sb * j.lbY;
      j.rby = sb * j.lbX + cb * j.lbY;
      let dx = b.x + j.rbx - a.x - j.rax;
      let dy = b.y + j.rby - a.y - j.ray;
      const len = Math.hypot(dx, dy);
      if (len > 1e-6) {
        dx /= len;
        dy /= len;
      } else {
        dx = 1;
        dy = 0;
      }
      j.ux = dx;
      j.uy = dy;
      const crA = j.rax * dy - j.ray * dx;
      const crB = j.rbx * dy - j.rby * dx;
      let invMass = a.invMass + b.invMass + a.invI * crA * crA + b.invI * crB * crB;
      const C = len - j.length;
      j.gamma = 0;
      j.bias = (BAUMGARTE / dt) * C;
      if (j.frequencyHz > 0 && invMass > 0) {
        const m = 1 / invMass;
        const omega = 2 * Math.PI * j.frequencyHz;
        const d = 2 * m * j.dampingRatio * omega;
        const k = m * omega * omega;
        const g = dt * (d + dt * k);
        if (g > 0) {
          j.gamma = 1 / g;
        }
        j.bias = C * dt * k * j.gamma;
        invMass += j.gamma;
      }
      j.softMass = 0;
      if (invMass > 0) {
        j.softMass = 1 / invMass;
      }
    } else {
      const b = j.b;
      const cb = Math.cos(b.angle);
      const sb = Math.sin(b.angle);
      j.rbx = cb * j.lbX - sb * j.lbY;
      j.rby = sb * j.lbX + cb * j.lbY;
      const m = b.mass;
      const omega = 2 * Math.PI * 5;
      const zeta = 0.7;
      const d = 2 * m * zeta * omega;
      const k = m * omega * omega;
      const g = dt * (d + dt * k);
      j.gamma = 0;
      if (g > 0) {
        j.gamma = 1 / g;
      }
      j.beta = dt * k * j.gamma;
      j.k11 = b.invMass + b.invI * j.rby * j.rby + j.gamma;
      j.k12 = -b.invI * j.rbx * j.rby;
      j.k22 = b.invMass + b.invI * j.rbx * j.rbx + j.gamma;
      b.w *= 0.98;
    }
  }

  private applyImpulse(a: Body, b: Body | null, rax: number, ray: number, rbx: number, rby: number, px: number, py: number): void {
    a.vx -= a.invMass * px;
    a.vy -= a.invMass * py;
    a.w -= a.invI * (rax * py - ray * px);
    if (b) {
      b.vx += b.invMass * px;
      b.vy += b.invMass * py;
      b.w += b.invI * (rbx * py - rby * px);
    }
  }

  private warmStart(j: Joint): void {
    if (j.kind === 'revolute') {
      const axial = j.motorImpulse + j.lowerImpulse - j.upperImpulse;
      j.a.w -= j.a.invI * axial;
      j.b.w += j.b.invI * axial;
      this.applyImpulse(j.a, j.b, j.rax, j.ray, j.rbx, j.rby, j.impulseX, j.impulseY);
    } else if (j.kind === 'distance') {
      this.applyImpulse(j.a, j.b, j.rax, j.ray, j.rbx, j.rby, j.impulse * j.ux, j.impulse * j.uy);
    } else {
      const b = j.b;
      b.vx += b.invMass * j.impulseX;
      b.vy += b.invMass * j.impulseY;
      b.w += b.invI * (j.rbx * j.impulseY - j.rby * j.impulseX);
    }
  }

  private solve(j: Joint, dt: number): void {
    if (j.kind === 'revolute') {
      this.solveRevolute(j, dt);
    } else if (j.kind === 'distance') {
      const { a, b } = j;
      const vax = a.vx - a.w * j.ray;
      const vay = a.vy + a.w * j.rax;
      const vbx = b.vx - b.w * j.rby;
      const vby = b.vy + b.w * j.rbx;
      const cdot = j.ux * (vbx - vax) + j.uy * (vby - vay);
      const imp = -j.softMass * (cdot + j.bias + j.gamma * j.impulse);
      j.impulse += imp;
      this.applyImpulse(a, b, j.rax, j.ray, j.rbx, j.rby, imp * j.ux, imp * j.uy);
    } else {
      const b = j.b;
      const cdx = b.vx - b.w * j.rby;
      const cdy = b.vy + b.w * j.rbx;
      const cx = b.x + j.rbx - j.targetX;
      const cy = b.y + j.rby - j.targetY;
      const rx = cdx + j.beta * cx + j.gamma * j.impulseX;
      const ry = cdy + j.beta * cy + j.gamma * j.impulseY;
      const det = j.k11 * j.k22 - j.k12 * j.k12;
      if (det === 0) {
        return;
      }
      const ix = -(j.k22 * rx - j.k12 * ry) / det;
      const iy = -(-j.k12 * rx + j.k11 * ry) / det;
      const oldX = j.impulseX;
      const oldY = j.impulseY;
      j.impulseX += ix;
      j.impulseY += iy;
      const maxImpulse = dt * j.maxForce;
      const len = Math.hypot(j.impulseX, j.impulseY);
      if (len > maxImpulse && len > 0) {
        j.impulseX *= maxImpulse / len;
        j.impulseY *= maxImpulse / len;
      }
      const px = j.impulseX - oldX;
      const py = j.impulseY - oldY;
      b.vx += b.invMass * px;
      b.vy += b.invMass * py;
      b.w += b.invI * (j.rbx * py - j.rby * px);
    }
  }

  private solveRevolute(j: RevoluteJoint, dt: number): void {
    const { a, b, def } = j;
    if (def.enableMotor && j.axialMass > 0) {
      const cdot = b.w - a.w - def.motorSpeed;
      let imp = -j.axialMass * cdot;
      const maxImp = dt * def.maxMotorTorque;
      const old = j.motorImpulse;
      j.motorImpulse = Math.max(-maxImp, Math.min(maxImp, old + imp));
      imp = j.motorImpulse - old;
      a.w -= a.invI * imp;
      b.w += b.invI * imp;
    }
    if (def.enableLimit && j.axialMass > 0) {
      const angle = b.angle - a.angle - j.refAngle;
      {
        const C = angle - def.lowerAngle;
        let slack = C / dt;
        if (C < 0) {
          slack = (BAUMGARTE * C) / dt;
        }
        const cdot = b.w - a.w;
        let imp = -j.axialMass * (cdot + slack);
        const old = j.lowerImpulse;
        j.lowerImpulse = Math.max(old + imp, 0);
        imp = j.lowerImpulse - old;
        a.w -= a.invI * imp;
        b.w += b.invI * imp;
      }
      {
        const C = def.upperAngle - angle;
        let slack = C / dt;
        if (C < 0) {
          slack = (BAUMGARTE * C) / dt;
        }
        const cdot = a.w - b.w;
        let imp = -j.axialMass * (cdot + slack);
        const old = j.upperImpulse;
        j.upperImpulse = Math.max(old + imp, 0);
        imp = j.upperImpulse - old;
        a.w += a.invI * imp;
        b.w -= b.invI * imp;
      }
    }
    const cdx = b.vx - b.w * j.rby - (a.vx - a.w * j.ray);
    const cdy = b.vy + b.w * j.rbx - (a.vy + a.w * j.rax);
    const cx = b.x + j.rbx - a.x - j.rax;
    const cy = b.y + j.rby - a.y - j.ray;
    const rx = cdx + (BAUMGARTE / dt) * cx;
    const ry = cdy + (BAUMGARTE / dt) * cy;
    const det = j.k11 * j.k22 - j.k12 * j.k12;
    if (det === 0) {
      return;
    }
    const ix = -(j.k22 * rx - j.k12 * ry) / det;
    const iy = -(-j.k12 * rx + j.k11 * ry) / det;
    j.impulseX += ix;
    j.impulseY += iy;
    this.applyImpulse(a, b, j.rax, j.ray, j.rbx, j.rby, ix, iy);
  }

  getBodyStates(): Float32Array {
    const n = this.bodies.size;
    if (this.statesBuf.length !== n * 3) {
      this.statesBuf = new Float32Array(n * 3);
    }
    let i = 0;
    for (const b of this.bodies.values()) {
      this.statesBuf[i++] = b.x;
      this.statesBuf[i++] = b.y;
      this.statesBuf[i++] = b.angle;
    }
    return this.statesBuf;
  }

  getBodyIds(): Int32Array {
    const n = this.bodies.size;
    if (this.idsBuf.length !== n) {
      this.idsBuf = new Int32Array(n);
    }
    let i = 0;
    for (const id of this.bodies.keys()) {
      this.idsBuf[i++] = id;
    }
    return this.idsBuf;
  }

  getJointAnchors(): Float32Array {
    const n = this.joints.size;
    if (this.anchorsBuf.length !== n * 4) {
      this.anchorsBuf = new Float32Array(n * 4);
    }
    let i = 0;
    for (const j of this.joints.values()) {
      if (j.kind === 'revolute' || j.kind === 'distance') {
        const ca = Math.cos(j.a.angle);
        const sa = Math.sin(j.a.angle);
        const cb = Math.cos(j.b.angle);
        const sb = Math.sin(j.b.angle);
        this.anchorsBuf[i++] = j.a.x + ca * j.laX - sa * j.laY;
        this.anchorsBuf[i++] = j.a.y + sa * j.laX + ca * j.laY;
        this.anchorsBuf[i++] = j.b.x + cb * j.lbX - sb * j.lbY;
        this.anchorsBuf[i++] = j.b.y + sb * j.lbX + cb * j.lbY;
      } else {
        const cb = Math.cos(j.b.angle);
        const sb = Math.sin(j.b.angle);
        this.anchorsBuf[i++] = j.b.x + cb * j.lbX - sb * j.lbY;
        this.anchorsBuf[i++] = j.b.y + sb * j.lbX + cb * j.lbY;
        this.anchorsBuf[i++] = j.targetX;
        this.anchorsBuf[i++] = j.targetY;
      }
    }
    return this.anchorsBuf;
  }

  getContacts(): Float32Array {
    return this.emptyContacts;
  }

  getStats(): WorldStats {
    let ke = 0;
    let pe = 0;
    for (const b of this.bodies.values()) {
      if (b.invMass > 0) {
        ke += 0.5 * b.mass * (b.vx * b.vx + b.vy * b.vy) + 0.5 * b.inertia * b.w * b.w;
        pe -= b.mass * (this.gx * b.x + this.gy * b.y);
      }
    }
    return { bodies: this.bodies.size, contacts: 0, stepMs: this.stepMs, kineticEnergy: ke, potentialEnergy: pe };
  }

  destroy(): void {
    this.bodies.clear();
    this.joints.clear();
  }
}

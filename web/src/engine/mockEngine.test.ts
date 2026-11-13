import { describe, expect, it } from 'vitest';
import { MockEngine } from './mockEngine';
import { defaultBodyDef, defaultRevoluteDef } from './types';

function pendulum(L = 1, angle0 = 0) {
  const w = new MockEngine();
  const pin = w.addBody(defaultBodyDef({ type: 'static', x: 0, y: 5, shape: { kind: 'circle', radius: 0.05 } }));
  // Bar hanging from the pin, rotated by angle0 from straight down.
  const a = -Math.PI / 2 + angle0;
  const bob = w.addBody(
    defaultBodyDef({
      x: (L / 2) * Math.cos(a),
      y: 5 + (L / 2) * Math.sin(a),
      angle: a,
      shape: { kind: 'box', w: L, h: 0.05 },
    }),
  );
  w.addRevoluteJoint(defaultRevoluteDef({ bodyA: pin, bodyB: bob, anchorX: 0, anchorY: 5 }));
  return { w, bob };
}

function totalEnergy(w: MockEngine): number {
  const s = w.getStats();
  return s.kineticEnergy + s.potentialEnergy;
}

describe('MockEngine', () => {
  it('swings a pendulum and conserves energy roughly', () => {
    const { w } = pendulum(1, 0.6);
    const e0 = totalEnergy(w);
    let minAngle = Infinity;
    let maxAngle = -Infinity;
    let maxDrift = 0;
    for (let i = 0; i < 60 * 10; i++) {
      w.step(1 / 60);
      const st = w.getBodyStates();
      const angle = st[5] + Math.PI / 2;
      minAngle = Math.min(minAngle, angle);
      maxAngle = Math.max(maxAngle, angle);
      maxDrift = Math.max(maxDrift, Math.abs(totalEnergy(w) - e0));
    }
    expect(maxAngle).toBeGreaterThan(0.5);
    expect(minAngle).toBeLessThan(-0.5);
    // Allow 5% of the energy exchanged between potential and kinetic.
    const m = 1 * 0.05;
    const swing = m * 9.81 * 0.5 * (1 - Math.cos(0.6));
    expect(maxDrift).toBeLessThan(0.05 * swing);
  });

  it('keeps the revolute anchor attached', () => {
    const { w } = pendulum(1, 1);
    for (let i = 0; i < 600; i++) {
      w.step(1 / 60);
    }
    const a = w.getJointAnchors();
    expect(Math.hypot(a[0] - a[2], a[1] - a[3])).toBeLessThan(0.01);
  });

  it('has a pendulum period near 2*pi*sqrt(I/(m g d))', () => {
    const { w } = pendulum(1, 0.1);
    // Rod about end: T = 2 pi sqrt(2L / 3g)
    const expected = 2 * Math.PI * Math.sqrt((2 * 1) / (3 * 9.81));
    let prev = w.getBodyStates()[2] + 0;
    const crossings: number[] = [];
    let t = 0;
    for (let i = 0; i < 60 * 6; i++) {
      w.step(1 / 60);
      t += 1 / 60;
      const st = w.getBodyStates();
      const ang = st[5] + Math.PI / 2;
      if (prev < 0 && ang >= 0) {
        crossings.push(t);
      }
      prev = ang;
    }
    expect(crossings.length).toBeGreaterThanOrEqual(2);
    const period = crossings[1] - crossings[0];
    expect(Math.abs(period - expected) / expected).toBeLessThan(0.03);
  });

  it('respects joint limits', () => {
    const { w } = pendulum(1, 1.2);
    // Limits are relative to the initial relative angle.
    const a0 = w.getBodyStates()[5];
    w.updateRevoluteJoint(0, { enableLimit: true, lowerAngle: -0.3, upperAngle: 0.3 });
    let maxA = -Infinity;
    for (let i = 0; i < 600; i++) {
      w.step(1 / 60);
      maxA = Math.max(maxA, Math.abs(w.getBodyStates()[5] - a0));
    }
    expect(maxA).toBeLessThan(0.45);
  });

  it('motor reaches target speed', () => {
    const { w } = pendulum(1, 0);
    w.setGravity(0, 0);
    w.updateRevoluteJoint(0, { enableMotor: true, motorSpeed: 2, maxMotorTorque: 100 });
    for (let i = 0; i < 60; i++) {
      w.step(1 / 60);
    }
    const a0 = w.getBodyStates()[5];
    w.step(0.1);
    const a1 = w.getBodyStates()[5];
    expect((a1 - a0) / 0.1).toBeCloseTo(2, 1);
  });

  it('queryPoint finds bodies and mouse joint pulls a body', () => {
    const w = new MockEngine();
    w.setGravity(0, 0);
    const id = w.addBody(defaultBodyDef({ x: 0, y: 0, shape: { kind: 'box', w: 1, h: 1 } }));
    expect(w.queryPoint(0.2, 0.2)).toBe(id);
    expect(w.queryPoint(3, 3)).toBe(-1);
    const j = w.createMouseJoint(id, 0, 0, 1000);
    w.setMouseTarget(j, 2, 0);
    for (let i = 0; i < 120; i++) {
      w.step(1 / 60);
    }
    expect(w.getBodyStates()[0]).toBeGreaterThan(1.5);
  });
});

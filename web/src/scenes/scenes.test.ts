import { describe, expect, it } from 'vitest';
import { MockEngine } from '../engine/mockEngine';
import { bodyMass, defaultBodyDef } from '../engine/types';
import { ARM_PIVOT_Y, buildArm } from './arm';
import { GALLERY } from './gallery';
import { defaultState } from './sceneState';

function finiteAfterSteps(w: MockEngine, n: number): boolean {
  for (let i = 0; i < n; i++) {
    w.step(1 / 60);
  }
  return Array.from(w.getBodyStates()).every((v) => Number.isFinite(v));
}

describe('scenes on the mock engine', () => {
  it('builds and runs the arm', () => {
    const w = new MockEngine();
    const b = buildArm(w, defaultState().links);
    expect(b.revoluteIds).toHaveLength(3);
    expect(finiteAfterSteps(w, 300)).toBe(true);
  });

  it.each(GALLERY.map((g) => [g.id, g] as const))('builds and runs %s', (_id, g) => {
    const w = new MockEngine();
    g.build(w);
    expect(finiteAfterSteps(w, 300)).toBe(true);
  });
});

describe('rest pose for the energy chart (kg*m summed over bodies hanging at rest)', () => {
  it('double pendulum: both bars hanging straight down from the pivot', () => {
    const w = new MockEngine();
    const g = GALLERY.find((x) => x.id === 'double-pendulum');
    const m = bodyMass(defaultBodyDef({ shape: { kind: 'box', w: 1.5, h: 0.1 } }));
    expect(g?.build(w).restHeightMass).toBeCloseTo(m * (3 - 0.75) + m * (3 - 2.25), 9);
  });

  it('cradle: five balls hanging at rod length below the bar', () => {
    const w = new MockEngine();
    const g = GALLERY.find((x) => x.id === 'cradle');
    const m = bodyMass(defaultBodyDef({ shape: { kind: 'circle', radius: 0.5 } }));
    expect(g?.build(w).restHeightMass).toBeCloseTo(5 * m * 1.5, 9);
  });

  it('arm: links hang straight down from the pivot', () => {
    const links = defaultState().links;
    const w = new MockEngine();
    let expected = 0;
    let drop = 0;
    for (const l of links) {
      const m = bodyMass(defaultBodyDef({ density: l.density, shape: { kind: 'box', w: l.length, h: l.width } }));
      expected += m * (ARM_PIVOT_Y - drop - l.length / 2);
      drop += l.length;
    }
    expect(buildArm(w, links).restHeightMass).toBeCloseTo(expected, 9);
  });

  it('scenes without a known rest pose leave it undefined', () => {
    const w = new MockEngine();
    expect(GALLERY.find((x) => x.id === 'bridge')?.build(w).restHeightMass).toBeUndefined();
  });
});

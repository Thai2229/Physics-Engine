import { describe, expect, it } from 'vitest';
import { MockEngine } from '../engine/mockEngine';
import { buildArm } from './arm';
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

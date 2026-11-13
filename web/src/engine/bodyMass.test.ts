import { describe, expect, it } from 'vitest';
import { bodyMass, defaultBodyDef } from './types';

describe('bodyMass', () => {
  it('computes box, circle and polygon mass as area times density', () => {
    expect(bodyMass(defaultBodyDef({ density: 2, shape: { kind: 'box', w: 2, h: 3 } }))).toBeCloseTo(12);
    expect(bodyMass(defaultBodyDef({ density: 1, shape: { kind: 'circle', radius: 0.5 } }))).toBeCloseTo(Math.PI / 4);
    const tri = [
      { x: 0, y: 0 },
      { x: 2, y: 0 },
      { x: 0, y: 2 },
    ];
    expect(bodyMass(defaultBodyDef({ density: 1, shape: { kind: 'polygon', vertices: tri } }))).toBeCloseTo(2);
  });
});

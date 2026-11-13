import { describe, expect, it } from 'vitest';
import { HAND_BASE_ACCEL, HAND_BASE_SPEED, HAND_MIN_MASS, handMaxForce, handMaxSpeed } from './hand';

describe('hand strength', () => {
  it('scales the pull force with body mass and strength', () => {
    expect(handMaxForce(2, 1)).toBeCloseTo(2 * HAND_BASE_ACCEL);
    expect(handMaxForce(2, 3)).toBeCloseTo(2 * HAND_BASE_ACCEL * 3);
  });

  it('never treats a body as lighter than the minimum mass', () => {
    expect(handMaxForce(0, 1)).toBeCloseTo(HAND_MIN_MASS * HAND_BASE_ACCEL);
  });

  it('scales the hand speed cap with strength', () => {
    expect(handMaxSpeed(1)).toBeCloseTo(HAND_BASE_SPEED);
    expect(handMaxSpeed(4)).toBeCloseTo(HAND_BASE_SPEED * 4);
  });
});

/** The hand can accelerate a grabbed body by at most this much (m/s^2) at strength 1x, whatever it weighs. */
export const HAND_BASE_ACCEL = 60;
/** A hand-held body cannot move faster than this (m/s) at strength 1x. */
export const HAND_BASE_SPEED = 5;
export const HAND_MIN_MASS = 0.05;

export function handMaxForce(mass: number, strength: number): number {
  return Math.max(mass, HAND_MIN_MASS) * HAND_BASE_ACCEL * strength;
}

export function handMaxSpeed(strength: number): number {
  return HAND_BASE_SPEED * strength;
}

import { describe, expect, it } from 'vitest';
import { defaultState, LIMITS, MAX_LINKS, parseState, sanitizeState, serializeState } from './sceneState';

describe('scene hash', () => {
  it('round-trips a state', () => {
    const s = defaultState();
    s.tab = 'bridge';
    s.gravity = 3.5;
    s.handStrength = 2.5;
    s.links[0].motorEnabled = true;
    s.links[0].limitEnabled = true;
    expect(parseState('#' + serializeState(s))).toEqual(s);
    expect(parseState(serializeState(s))).toEqual(s);
  });

  it('falls back to defaults on garbage', () => {
    expect(parseState('')).toEqual(defaultState());
    expect(parseState('#s=%E0%A4%A')).toEqual(defaultState());
    expect(parseState('#s=' + encodeURIComponent('{not json'))).toEqual(defaultState());
    expect(parseState('#s=' + encodeURIComponent('42'))).toEqual(defaultState());
    expect(parseState('#other=1')).toEqual(defaultState());
  });

  it('clamps out-of-range values and rejects wrong types', () => {
    const s = sanitizeState({
      tab: 'nope',
      gravity: 1e9,
      timeScale: -5,
      handStrength: 1e9,
      links: [{ length: 100, width: 'x', density: NaN, lowerDeg: 90, upperDeg: -90, motorSpeed: 99, maxTorque: -1, motorEnabled: 'yes' }],
    });
    expect(s.tab).toBe('arm');
    expect(s.gravity).toBe(LIMITS.gravity.max);
    expect(s.timeScale).toBe(LIMITS.timeScale.min);
    expect(s.handStrength).toBe(LIMITS.handStrength.max);
    expect(sanitizeState({ handStrength: 'strong' }).handStrength).toBe(1);
    const l = s.links[0];
    expect(l.length).toBe(LIMITS.length.max);
    expect(l.width).toBeGreaterThanOrEqual(LIMITS.width.min);
    expect(Number.isFinite(l.density)).toBe(true);
    expect(l.lowerDeg).toBe(-90);
    expect(l.upperDeg).toBe(90);
    expect(l.motorSpeed).toBe(LIMITS.motorSpeed.max);
    expect(l.maxTorque).toBe(LIMITS.maxTorque.min);
    expect(l.motorEnabled).toBe(false);
  });

  it('limits link count and keeps defaults for empty list', () => {
    const many = sanitizeState({ links: Array.from({ length: 50 }, () => ({})) });
    expect(many.links).toHaveLength(MAX_LINKS);
    expect(sanitizeState({ links: [] }).links).toEqual(defaultState().links);
  });
});

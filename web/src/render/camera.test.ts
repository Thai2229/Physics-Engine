import { describe, expect, it } from 'vitest';
import { clampFrameDt, MAX_FRAME_DT, MAX_SCALE, panBy, screenToWorld, worldToScreen, zoomAt } from './camera';

const vp = { width: 800, height: 600 };
const cam = { x: 2, y: 1, scale: 50 };

describe('camera', () => {
  it('maps the camera centre to the viewport centre and flips y', () => {
    expect(worldToScreen(cam, vp, 2, 1)).toEqual({ x: 400, y: 300 });
    const up = worldToScreen(cam, vp, 2, 2);
    expect(up.y).toBe(250);
  });

  it('round-trips world <-> screen', () => {
    const s = worldToScreen(cam, vp, -3.2, 4.7);
    const w = screenToWorld(cam, vp, s.x, s.y);
    expect(w.x).toBeCloseTo(-3.2, 9);
    expect(w.y).toBeCloseTo(4.7, 9);
  });

  it('zoom keeps the point under the cursor fixed and clamps scale', () => {
    const before = screenToWorld(cam, vp, 123, 456);
    const z = zoomAt(cam, vp, 123, 456, 2);
    expect(z.scale).toBe(100);
    const after = screenToWorld(z, vp, 123, 456);
    expect(after.x).toBeCloseTo(before.x, 9);
    expect(after.y).toBeCloseTo(before.y, 9);
    expect(zoomAt(cam, vp, 0, 0, 1e6).scale).toBe(MAX_SCALE);
  });

  it('pan moves content with the pointer', () => {
    const p = panBy(cam, 50, 50);
    const s = worldToScreen(p, vp, 2, 1);
    expect(s.x).toBeCloseTo(450, 9);
    expect(s.y).toBeCloseTo(350, 9);
  });

  it('clamps frame dt', () => {
    expect(clampFrameDt(16)).toBeCloseTo(0.016);
    expect(clampFrameDt(5000)).toBe(MAX_FRAME_DT);
    expect(clampFrameDt(-1)).toBe(0);
    expect(clampFrameDt(NaN)).toBe(0);
  });
});

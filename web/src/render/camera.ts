/** World (meters, y-up) <-> screen (CSS pixels, y-down) camera. */
export interface Camera {
  /** World point at the viewport centre. */
  x: number;
  y: number;
  /** Pixels per meter. */
  scale: number;
}

export interface Viewport {
  width: number;
  height: number;
}

export const MIN_SCALE = 5;
export const MAX_SCALE = 500;

export function worldToScreen(cam: Camera, vp: Viewport, wx: number, wy: number): { x: number; y: number } {
  return {
    x: vp.width / 2 + (wx - cam.x) * cam.scale,
    y: vp.height / 2 - (wy - cam.y) * cam.scale,
  };
}

export function screenToWorld(cam: Camera, vp: Viewport, sx: number, sy: number): { x: number; y: number } {
  return {
    x: cam.x + (sx - vp.width / 2) / cam.scale,
    y: cam.y - (sy - vp.height / 2) / cam.scale,
  };
}

/** Zoom by `factor` keeping the world point under (sx, sy) fixed. */
export function zoomAt(cam: Camera, vp: Viewport, sx: number, sy: number, factor: number): Camera {
  const scale = Math.min(MAX_SCALE, Math.max(MIN_SCALE, cam.scale * factor));
  const before = screenToWorld(cam, vp, sx, sy);
  const next = { x: cam.x, y: cam.y, scale };
  const after = screenToWorld(next, vp, sx, sy);
  next.x += before.x - after.x;
  next.y += before.y - after.y;
  return next;
}

/** Pan by a screen-space drag delta (content follows the pointer). */
export function panBy(cam: Camera, dsx: number, dsy: number): Camera {
  return { x: cam.x - dsx / cam.scale, y: cam.y + dsy / cam.scale, scale: cam.scale };
}

/** Largest real frame time (seconds) fed to the engine. */
export const MAX_FRAME_DT = 1 / 20;

export function clampFrameDt(ms: number): number {
  if (!Number.isFinite(ms) || ms < 0) {
    return 0;
  }
  return Math.min(ms / 1000, MAX_FRAME_DT);
}

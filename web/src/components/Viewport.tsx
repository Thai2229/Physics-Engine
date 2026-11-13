'use client';

import { useEffect, useRef } from 'react';
import type { MutableRefObject } from 'react';
import type { EngineHandle } from '../engine/loadEngine';
import { handMaxForce, handMaxSpeed } from '../engine/hand';
import { bodyMass } from '../engine/types';
import type { PhysicsWorld } from '../engine/types';
import { clampFrameDt, panBy, screenToWorld, zoomAt } from '../render/camera';
import type { Camera } from '../render/camera';
import { Renderer } from '../render/renderer';
import type { DrawArgs, Overlays } from '../render/renderer';
import { recordSample, resetTelemetry } from '../render/telemetry';
import type { Telemetry } from '../render/telemetry';
import type { SceneBuild } from '../scenes/types';

const MAX_TRAIL_POINTS = 400;

export interface ViewportProps {
  handle: EngineHandle;
  /** World is rebuilt when this changes (or resetNonce does). */
  buildKey: string;
  build: (world: PhysicsWorld) => SceneBuild;
  /** Cheap live updates (no rebuild) applied when this changes. */
  liveKey: string;
  live?: (world: PhysicsWorld, build: SceneBuild) => void;
  /** Camera is reset when this changes (or resetNonce does). */
  cameraKey: string;
  gravity: number;
  timeScale: number;
  handStrength: number;
  playing: boolean;
  resetNonce: number;
  stepNonce: number;
  overlays: Overlays;
  telemetry: MutableRefObject<Telemetry>;
}

interface Sim {
  world: PhysicsWorld;
  build: SceneBuild;
  trail: number[];
}

type Drag =
  | { mode: 'mouse'; jointId: number; bodyId: number; localX: number; localY: number; targetX: number; targetY: number }
  | { mode: 'pan'; x: number; y: number };

export function Viewport(props: ViewportProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const rendererRef = useRef<Renderer | null>(null);
  const sim = useRef<Sim | null>(null);
  const camera = useRef<Camera>({ x: 0, y: 0, scale: 50 });
  const cameraKeyRef = useRef<string | null>(null);
  const resetRef = useRef(-1);
  const drag = useRef<Drag | null>(null);
  const latest = useRef(props);

  useEffect(() => {
    latest.current = props;
  });

  // (Re)build the world.
  useEffect(() => {
    const p = latest.current;
    const world = p.handle.createWorld();
    world.setGravity(0, -p.gravity);
    world.setMouseMaxSpeed(handMaxSpeed(p.handStrength));
    const build = p.build(world);
    sim.current = { world, build, trail: [] };
    drag.current = null;
    resetTelemetry(p.telemetry.current);
    rendererRef.current?.resetHistory();
    if (cameraKeyRef.current !== p.cameraKey || resetRef.current !== p.resetNonce) {
      camera.current = { ...build.camera };
      cameraKeyRef.current = p.cameraKey;
      resetRef.current = p.resetNonce;
    }
    recordSample(p.telemetry.current, world.getStats(), restPotential(build, p.gravity));
    return () => {
      sim.current = null;
      world.destroy();
    };
  }, [props.handle, props.buildKey, props.resetNonce, props.cameraKey]);

  useEffect(() => {
    sim.current?.world.setGravity(0, -props.gravity);
  }, [props.gravity, props.buildKey, props.resetNonce]);

  useEffect(() => {
    sim.current?.world.setMouseMaxSpeed(handMaxSpeed(props.handStrength));
  }, [props.handStrength, props.buildKey, props.resetNonce]);

  useEffect(() => {
    const s = sim.current;
    const live = latest.current.live;
    if (s && live) {
      live(s.world, s.build);
    }
  }, [props.liveKey, props.buildKey, props.resetNonce]);

  // Single-step.
  useEffect(() => {
    if (props.stepNonce === 0) {
      return;
    }
    const s = sim.current;
    if (s) {
      advance(s, 1 / 60, latest.current.telemetry.current, latest.current.gravity);
    }
  }, [props.stepNonce]);

  // Render loop + resize + wheel.
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) {
      return;
    }
    const renderer = new Renderer(canvas);
    rendererRef.current = renderer;
    const parent = canvas.parentElement;
    const resize = () => {
      const w = parent?.clientWidth ?? 600;
      const h = parent?.clientHeight ?? 400;
      renderer.resize(w, h, window.devicePixelRatio || 1);
    };
    resize();
    const ro = new ResizeObserver(resize);
    if (parent) {
      ro.observe(parent);
    }
    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      const rect = canvas.getBoundingClientRect();
      const factor = Math.exp(-e.deltaY * 0.0015);
      camera.current = zoomAt(camera.current, renderer.viewport, e.clientX - rect.left, e.clientY - rect.top, factor);
    };
    canvas.addEventListener('wheel', onWheel, { passive: false });

    let last = performance.now();
    let raf = 0;
    const frame = (now: number) => {
      raf = requestAnimationFrame(frame);
      const dtReal = clampFrameDt(now - last);
      last = now;
      const s = sim.current;
      const p = latest.current;
      if (!s) {
        return;
      }
      let simDt = 0;
      if (p.playing) {
        simDt = dtReal * p.timeScale;
        advance(s, simDt, p.telemetry.current, p.gravity);
      }
      let dragLine: DrawArgs['dragLine'];
      const d = drag.current;
      if (d && d.mode === 'mouse') {
        const ids = s.world.getBodyIds();
        const idx = ids.indexOf(d.bodyId);
        if (idx >= 0) {
          const st = s.world.getBodyStates();
          const c = Math.cos(st[idx * 3 + 2]);
          const sn = Math.sin(st[idx * 3 + 2]);
          dragLine = {
            ax: st[idx * 3] + c * d.localX - sn * d.localY,
            ay: st[idx * 3 + 1] + sn * d.localX + c * d.localY,
            bx: d.targetX,
            by: d.targetY,
          };
        }
      }
      renderer.draw({
        dragLine,
        world: s.world,
        shapes: s.build.shapes,
        camera: camera.current,
        overlays: p.overlays,
        trail: s.trail,
        simDt,
      });
    };
    raf = requestAnimationFrame(frame);
    return () => {
      cancelAnimationFrame(raf);
      ro.disconnect();
      canvas.removeEventListener('wheel', onWheel);
      rendererRef.current = null;
    };
  }, []);

  const toWorld = (e: React.PointerEvent) => {
    const rect = e.currentTarget.getBoundingClientRect();
    const r = rendererRef.current;
    if (!r) {
      return null;
    }
    return screenToWorld(camera.current, r.viewport, e.clientX - rect.left, e.clientY - rect.top);
  };

  const onPointerDown = (e: React.PointerEvent<HTMLCanvasElement>) => {
    const s = sim.current;
    const w = toWorld(e);
    if (!s || !w) {
      return;
    }
    e.currentTarget.setPointerCapture(e.pointerId);
    const id = s.world.queryPoint(w.x, w.y);
    const def = s.build.shapes.get(id);
    if (id >= 0 && def && def.type === 'dynamic') {
      const ids = s.world.getBodyIds();
      const idx = ids.indexOf(id);
      const st = s.world.getBodyStates();
      const dx = w.x - st[idx * 3];
      const dy = w.y - st[idx * 3 + 1];
      const c = Math.cos(st[idx * 3 + 2]);
      const sn = Math.sin(st[idx * 3 + 2]);
      drag.current = {
        mode: 'mouse',
        jointId: s.world.createMouseJoint(id, w.x, w.y, handMaxForce(bodyMass(def), latest.current.handStrength)),
        bodyId: id,
        localX: c * dx + sn * dy,
        localY: -sn * dx + c * dy,
        targetX: w.x,
        targetY: w.y,
      };
    } else {
      drag.current = { mode: 'pan', x: e.clientX, y: e.clientY };
    }
  };

  const onPointerMove = (e: React.PointerEvent<HTMLCanvasElement>) => {
    const d = drag.current;
    if (!d) {
      return;
    }
    if (d.mode === 'mouse') {
      const w = toWorld(e);
      if (w && sim.current) {
        sim.current.world.setMouseTarget(d.jointId, w.x, w.y);
        d.targetX = w.x;
        d.targetY = w.y;
      }
    } else {
      camera.current = panBy(camera.current, e.clientX - d.x, e.clientY - d.y);
      d.x = e.clientX;
      d.y = e.clientY;
    }
  };

  const endDrag = () => {
    const d = drag.current;
    if (d && d.mode === 'mouse' && sim.current) {
      sim.current.world.removeJoint(d.jointId);
    }
    drag.current = null;
  };

  return (
    <canvas
      ref={canvasRef}
      className="viewport-canvas"
      onPointerDown={onPointerDown}
      onPointerMove={onPointerMove}
      onPointerUp={endDrag}
      onPointerCancel={endDrag}
    />
  );
}

/** Potential energy (J) the scene keeps at rest under the current gravity; 0 when its rest pose is unknown. */
function restPotential(build: SceneBuild, gravity: number): number {
  return (build.restHeightMass ?? 0) * gravity;
}

function advance(s: Sim, dt: number, telemetry: Telemetry, gravity: number): void {
  s.world.step(dt);
  telemetry.simTime += dt;
  recordSample(telemetry, s.world.getStats(), restPotential(s.build, gravity));
  const trail = s.build.trail;
  if (trail) {
    const ids = s.world.getBodyIds();
    const idx = ids.indexOf(trail.bodyId);
    if (idx >= 0) {
      const st = s.world.getBodyStates();
      const c = Math.cos(st[idx * 3 + 2]);
      const sn = Math.sin(st[idx * 3 + 2]);
      s.trail.push(
        st[idx * 3] + c * trail.localX - sn * trail.localY,
        st[idx * 3 + 1] + sn * trail.localX + c * trail.localY,
      );
      if (s.trail.length > MAX_TRAIL_POINTS * 2) {
        s.trail.splice(0, 2);
      }
    }
  }
}

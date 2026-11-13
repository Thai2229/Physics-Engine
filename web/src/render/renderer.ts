import type { BodyDef, PhysicsWorld } from '../engine/types';
import { worldToScreen } from './camera';
import type { Camera, Viewport } from './camera';

export interface Overlays {
  jointAnchors: boolean;
  contacts: boolean;
  velocities: boolean;
  centerOfMass: boolean;
}

export const DEFAULT_OVERLAYS: Overlays = {
  jointAnchors: true,
  contacts: true,
  velocities: false,
  centerOfMass: false,
};

export interface DrawArgs {
  world: PhysicsWorld;
  shapes: Map<number, BodyDef>;
  camera: Camera;
  overlays: Overlays;
  trail: number[];
  /** Sim time advanced since last draw; 0 while paused (velocity overlay keeps last values). */
  simDt: number;
}

const FILL: Record<BodyDef['type'], string> = {
  static: '#475569',
  dynamic: '#3b82f6',
  kinematic: '#a855f7',
};

const STROKE = '#0f172a';

export class Renderer {
  private ctx: CanvasRenderingContext2D;
  private vp: Viewport = { width: 1, height: 1 };
  private dpr = 1;
  private prev = new Map<number, [number, number]>();
  private vel = new Map<number, [number, number]>();

  constructor(private canvas: HTMLCanvasElement) {
    const ctx = canvas.getContext('2d');
    if (!ctx) {
      throw new Error('2D canvas not supported');
    }
    this.ctx = ctx;
  }

  get viewport(): Viewport {
    return this.vp;
  }

  resize(width: number, height: number, dpr: number): void {
    this.vp = { width, height };
    this.dpr = dpr;
    this.canvas.width = Math.max(1, Math.floor(width * dpr));
    this.canvas.height = Math.max(1, Math.floor(height * dpr));
  }

  resetHistory(): void {
    this.prev.clear();
    this.vel.clear();
  }

  draw(args: DrawArgs): void {
    const { ctx, vp } = this;
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    ctx.fillStyle = '#f8fafc';
    ctx.fillRect(0, 0, vp.width, vp.height);
    this.drawGrid(args.camera);
    this.drawBodies(args);
    this.drawTrail(args);
    if (args.overlays.jointAnchors) {
      this.drawJoints(args);
    }
    if (args.overlays.contacts) {
      this.drawContacts(args);
    }
  }

  private drawGrid(cam: Camera): void {
    const { ctx, vp } = this;
    let step = 1;
    if (cam.scale < 20) {
      step = 5;
    }
    const tl = { x: cam.x - vp.width / 2 / cam.scale, y: cam.y + vp.height / 2 / cam.scale };
    const br = { x: cam.x + vp.width / 2 / cam.scale, y: cam.y - vp.height / 2 / cam.scale };
    ctx.lineWidth = 1;
    ctx.strokeStyle = '#e2e8f0';
    ctx.beginPath();
    for (let gx = Math.ceil(tl.x / step) * step; gx <= br.x; gx += step) {
      const s = worldToScreen(cam, vp, gx, 0);
      ctx.moveTo(Math.round(s.x) + 0.5, 0);
      ctx.lineTo(Math.round(s.x) + 0.5, vp.height);
    }
    for (let gy = Math.ceil(br.y / step) * step; gy <= tl.y; gy += step) {
      const s = worldToScreen(cam, vp, 0, gy);
      ctx.moveTo(0, Math.round(s.y) + 0.5);
      ctx.lineTo(vp.width, Math.round(s.y) + 0.5);
    }
    ctx.stroke();
    const o = worldToScreen(cam, vp, 0, 0);
    ctx.strokeStyle = '#94a3b8';
    ctx.beginPath();
    ctx.moveTo(0, o.y);
    ctx.lineTo(vp.width, o.y);
    ctx.moveTo(o.x, 0);
    ctx.lineTo(o.x, vp.height);
    ctx.stroke();
  }

  private drawBodies(args: DrawArgs): void {
    const { ctx, vp } = this;
    const { world, shapes, camera, overlays, simDt } = args;
    const states = world.getBodyStates();
    const ids = world.getBodyIds();
    const s = camera.scale;
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = STROKE;
    for (let i = 0; i < ids.length; i++) {
      const id = ids[i];
      const x = states[i * 3];
      const y = states[i * 3 + 1];
      const angle = states[i * 3 + 2];
      if (simDt > 0) {
        const p = this.prev.get(id);
        if (p) {
          this.vel.set(id, [(x - p[0]) / simDt, (y - p[1]) / simDt]);
        }
        this.prev.set(id, [x, y]);
      }
      const def = shapes.get(id);
      if (!def) {
        continue;
      }
      const c = worldToScreen(camera, vp, x, y);
      ctx.save();
      ctx.translate(c.x, c.y);
      ctx.rotate(-angle);
      ctx.fillStyle = FILL[def.type];
      ctx.globalAlpha = 0.85;
      const shape = def.shape;
      ctx.beginPath();
      if (shape.kind === 'circle') {
        ctx.arc(0, 0, shape.radius * s, 0, Math.PI * 2);
      } else if (shape.kind === 'box') {
        ctx.rect((-shape.w / 2) * s, (-shape.h / 2) * s, shape.w * s, shape.h * s);
      } else {
        shape.vertices.forEach((v, k) => {
          if (k === 0) {
            ctx.moveTo(v.x * s, -v.y * s);
          } else {
            ctx.lineTo(v.x * s, -v.y * s);
          }
        });
        ctx.closePath();
      }
      ctx.fill();
      ctx.globalAlpha = 1;
      ctx.stroke();
      if (shape.kind === 'circle') {
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(shape.radius * s, 0);
        ctx.stroke();
      }
      ctx.restore();
      if (overlays.centerOfMass && def.type === 'dynamic') {
        this.cross(c.x, c.y, 5, '#dc2626');
      }
      if (overlays.velocities && def.type !== 'static') {
        const v = this.vel.get(id);
        if (v) {
          this.arrow(c.x, c.y, c.x + v[0] * s * 0.25, c.y - v[1] * s * 0.25, '#16a34a');
        }
      }
    }
  }

  private drawTrail(args: DrawArgs): void {
    const { ctx, vp } = this;
    const t = args.trail;
    if (t.length < 4) {
      return;
    }
    ctx.lineWidth = 2;
    for (let i = 2; i < t.length; i += 2) {
      const alpha = i / t.length;
      ctx.strokeStyle = `rgba(234, 88, 12, ${alpha.toFixed(3)})`;
      const a = worldToScreen(args.camera, vp, t[i - 2], t[i - 1]);
      const b = worldToScreen(args.camera, vp, t[i], t[i + 1]);
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.stroke();
    }
  }

  private drawJoints(args: DrawArgs): void {
    const { ctx, vp } = this;
    const a = args.world.getJointAnchors();
    ctx.lineWidth = 1.5;
    for (let i = 0; i + 3 < a.length; i += 4) {
      const p = worldToScreen(args.camera, vp, a[i], a[i + 1]);
      const q = worldToScreen(args.camera, vp, a[i + 2], a[i + 3]);
      if (Math.hypot(p.x - q.x, p.y - q.y) > 1) {
        ctx.strokeStyle = '#f59e0b';
        ctx.beginPath();
        ctx.moveTo(p.x, p.y);
        ctx.lineTo(q.x, q.y);
        ctx.stroke();
      }
      ctx.fillStyle = '#fff';
      ctx.strokeStyle = '#b45309';
      ctx.beginPath();
      ctx.arc(p.x, p.y, 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    }
  }

  private drawContacts(args: DrawArgs): void {
    const { ctx, vp } = this;
    const c = args.world.getContacts();
    for (let i = 0; i + 3 < c.length; i += 4) {
      const p = worldToScreen(args.camera, vp, c[i], c[i + 1]);
      ctx.fillStyle = '#ef4444';
      ctx.beginPath();
      ctx.arc(p.x, p.y, 3, 0, Math.PI * 2);
      ctx.fill();
      this.arrow(p.x, p.y, p.x + c[i + 2] * 16, p.y - c[i + 3] * 16, '#ef4444');
    }
  }

  private cross(x: number, y: number, r: number, color: string): void {
    const { ctx } = this;
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(x - r, y);
    ctx.lineTo(x + r, y);
    ctx.moveTo(x, y - r);
    ctx.lineTo(x, y + r);
    ctx.stroke();
  }

  private arrow(x0: number, y0: number, x1: number, y1: number, color: string): void {
    const { ctx } = this;
    const len = Math.hypot(x1 - x0, y1 - y0);
    if (len < 2) {
      return;
    }
    const ang = Math.atan2(y1 - y0, x1 - x0);
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(x0, y0);
    ctx.lineTo(x1, y1);
    ctx.lineTo(x1 - 6 * Math.cos(ang - 0.4), y1 - 6 * Math.sin(ang - 0.4));
    ctx.moveTo(x1, y1);
    ctx.lineTo(x1 - 6 * Math.cos(ang + 0.4), y1 - 6 * Math.sin(ang + 0.4));
    ctx.stroke();
  }
}

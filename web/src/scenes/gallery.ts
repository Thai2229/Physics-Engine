import { defaultBodyDef, defaultRevoluteDef } from '../engine/types';
import type { PhysicsWorld } from '../engine/types';
import { SceneBuilder } from './types';
import type { GalleryScene, SceneBuild } from './types';

function doublePendulum(world: PhysicsWorld): SceneBuild {
  const b = new SceneBuilder(world);
  const L = 1.5;
  const pivotY = 3;
  const pin = b.body(defaultBodyDef({ type: 'static', x: 0, y: pivotY, shape: { kind: 'circle', radius: 0.08 } }));
  const a1 = 2.2; // initial angle of bar 1 from +x axis (radians)
  const a2 = 2.9;
  const x1 = (L / 2) * Math.cos(a1);
  const y1 = pivotY + (L / 2) * Math.sin(a1);
  const jx = L * Math.cos(a1);
  const jy = pivotY + L * Math.sin(a1);
  const bar1 = b.body(defaultBodyDef({ x: x1, y: y1, angle: a1, shape: { kind: 'box', w: L, h: 0.1 }, collisionGroup: -1 }));
  const bar2 = b.body(
    defaultBodyDef({
      x: jx + (L / 2) * Math.cos(a2),
      y: jy + (L / 2) * Math.sin(a2),
      angle: a2,
      shape: { kind: 'box', w: L, h: 0.1 },
      collisionGroup: -1,
    }),
  );
  b.revoluteIds.push(world.addRevoluteJoint(defaultRevoluteDef({ bodyA: pin, bodyB: bar1, anchorX: 0, anchorY: pivotY })));
  b.revoluteIds.push(world.addRevoluteJoint(defaultRevoluteDef({ bodyA: bar1, bodyB: bar2, anchorX: jx, anchorY: jy })));
  return b.finish({ x: 0, y: 1.5, scale: 80 }, { bodyId: bar2, localX: L / 2, localY: 0 });
}

function bridge(world: PhysicsWorld): SceneBuild {
  const b = new SceneBuilder(world);
  const planks = 14;
  const span = 12;
  const pw = span / planks;
  const y = 2;
  const left = b.body(defaultBodyDef({ type: 'static', x: -span / 2 - 0.25, y: y - 0.5, shape: { kind: 'box', w: 0.5, h: 2 } }));
  const right = b.body(defaultBodyDef({ type: 'static', x: span / 2 + 0.25, y: y - 0.5, shape: { kind: 'box', w: 0.5, h: 2 } }));
  let prev = left;
  for (let i = 0; i < planks; i++) {
    const x = -span / 2 + pw * (i + 0.5);
    const id = b.body(defaultBodyDef({ x, y, shape: { kind: 'box', w: pw * 0.96, h: 0.16 }, collisionGroup: -1, angularDamping: 0.1, linearDamping: 0.2 }));
    b.revoluteIds.push(
      world.addRevoluteJoint(defaultRevoluteDef({ bodyA: prev, bodyB: id, anchorX: x - pw / 2, anchorY: y })),
    );
    prev = id;
  }
  b.revoluteIds.push(world.addRevoluteJoint(defaultRevoluteDef({ bodyA: prev, bodyB: right, anchorX: span / 2, anchorY: y })));
  return b.finish({ x: 0, y: 0.5, scale: 50 });
}

function cradle(world: PhysicsWorld): SceneBuild {
  const b = new SceneBuilder(world);
  const n = 5;
  const r = 0.5;
  const len = 3;
  const top = 4.5;
  // Real cradles leave a hair of space between balls; exactly touching balls make the contact solver inject energy.
  const gap = 0.02;
  const bar = b.body(defaultBodyDef({ type: 'static', x: 0, y: top, shape: { kind: 'box', w: n * 2 * r + 1, h: 0.15 } }));
  for (let i = 0; i < n; i++) {
    const px = (i - (n - 1) / 2) * (2 * r + gap);
    let x = px;
    let y = top - len;
    if (i === 0) {
      const swing = 0.8;
      x = px - len * Math.sin(swing);
      y = top - len * Math.cos(swing);
    }
    const id = b.body(
      defaultBodyDef({ x, y, shape: { kind: 'circle', radius: r }, restitution: 1, friction: 0, collisionGroup: 0, linearDamping: 0.15 }),
    );
    world.addDistanceJoint({
      bodyA: bar,
      bodyB: id,
      anchorAX: px,
      anchorAY: top,
      anchorBX: x,
      anchorBY: y,
      length: len,
      frequencyHz: 0,
      dampingRatio: 0,
      collideConnected: false,
    });
  }
  return b.finish({ x: 0, y: 2.5, scale: 70 });
}

function stack(world: PhysicsWorld): SceneBuild {
  const b = new SceneBuilder(world);
  b.body(defaultBodyDef({ type: 'static', x: 0, y: -0.25, shape: { kind: 'box', w: 20, h: 0.5 } }));
  const rows = 8;
  for (let row = 0; row < rows; row++) {
    const count = rows - row;
    for (let i = 0; i < count; i++) {
      b.body(
        defaultBodyDef({
          x: (i - (count - 1) / 2) * 0.82,
          y: 0.2 + row * 0.4,
          shape: { kind: 'box', w: 0.8, h: 0.4 },
          friction: 0.6,
        }),
      );
    }
  }
  return b.finish({ x: 0, y: 2, scale: 60 });
}

export const GALLERY: GalleryScene[] = [
  {
    id: 'double-pendulum',
    title: 'Double pendulum',
    description: 'Chaotic motion from two revolute joints. The trail follows the tip of the second bar.',
    needsCollisions: false,
    build: doublePendulum,
  },
  {
    id: 'bridge',
    title: 'Rope bridge',
    description: 'A chain of planks pinned between two posts. Drag planks with the mouse.',
    needsCollisions: false,
    build: bridge,
  },
  {
    id: 'cradle',
    title: "Newton's cradle",
    description: 'Five restitution-1 balls on rigid rods. Momentum transfer needs ball-ball collisions.',
    needsCollisions: true,
    build: cradle,
  },
  {
    id: 'stack',
    title: 'Box pyramid',
    description: 'A 36-box pyramid testing solver stability. Needs contacts and ground collision.',
    needsCollisions: true,
    build: stack,
  },
];

export function findGalleryScene(id: string): GalleryScene | undefined {
  return GALLERY.find((s) => s.id === id);
}

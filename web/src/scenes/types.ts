import type { BodyDef, PhysicsWorld } from '../engine/types';
import type { Camera } from '../render/camera';

export interface SceneBuild {
  /** Shape/type info per body id (the engine API does not expose shapes). */
  shapes: Map<number, BodyDef>;
  /** Revolute joint ids, for live updates. */
  revoluteIds: number[];
  camera: Camera;
  /** Draws a trail of a body-local point. */
  trail?: { bodyId: number; localX: number; localY: number };
}

export class SceneBuilder {
  readonly shapes = new Map<number, BodyDef>();
  readonly revoluteIds: number[] = [];

  constructor(readonly world: PhysicsWorld) {}

  body(def: BodyDef): number {
    const id = this.world.addBody(def);
    this.shapes.set(id, def);
    return id;
  }

  finish(camera: Camera, trail?: SceneBuild['trail']): SceneBuild {
    return { shapes: this.shapes, revoluteIds: this.revoluteIds, camera, trail };
  }
}

export interface GalleryScene {
  id: string;
  title: string;
  description: string;
  /** True when the scene is only meaningful with a collision-capable engine. */
  needsCollisions: boolean;
  build(world: PhysicsWorld): SceneBuild;
}

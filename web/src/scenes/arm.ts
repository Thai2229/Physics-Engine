import { defaultBodyDef, defaultRevoluteDef } from '../engine/types';
import type { PhysicsWorld } from '../engine/types';
import type { LinkParams } from './sceneState';
import { SceneBuilder } from './types';
import type { SceneBuild } from './types';

export const ARM_PIVOT_Y = 2.5;
const DEG = Math.PI / 180;

export function jointPatch(link: LinkParams) {
  return {
    enableLimit: link.limitEnabled,
    lowerAngle: link.lowerDeg * DEG,
    upperAngle: link.upperDeg * DEG,
    enableMotor: link.motorEnabled,
    motorSpeed: link.motorSpeed,
    maxMotorTorque: link.maxTorque,
  };
}

/** Key of parameters that need a world rebuild when changed. */
export function armStructureKey(links: LinkParams[]): string {
  return JSON.stringify(links.map((l) => [l.length, l.width, l.density]));
}

/** Key of parameters that can be applied live via updateRevoluteJoint. */
export function armJointKey(links: LinkParams[]): string {
  return JSON.stringify(links.map(jointPatch));
}

export function buildArm(world: PhysicsWorld, links: LinkParams[]): SceneBuild {
  const b = new SceneBuilder(world);
  const base = b.body(
    defaultBodyDef({ type: 'static', x: 0, y: ARM_PIVOT_Y, shape: { kind: 'box', w: 0.6, h: 0.5 }, collisionGroup: -1 }),
  );
  let prev = base;
  let px = 0;
  for (const link of links) {
    const id = b.body(
      defaultBodyDef({
        x: px + link.length / 2,
        y: ARM_PIVOT_Y,
        shape: { kind: 'box', w: link.length, h: link.width },
        density: link.density,
        angularDamping: 0.05,
        collisionGroup: -1,
      }),
    );
    const jid = world.addRevoluteJoint(
      defaultRevoluteDef({ bodyA: prev, bodyB: id, anchorX: px, anchorY: ARM_PIVOT_Y, ...jointPatch(link) }),
    );
    b.revoluteIds.push(jid);
    prev = id;
    px += link.length;
  }
  return b.finish({ x: 0, y: 0.5, scale: 55 });
}

export function applyArmJoints(world: PhysicsWorld, build: SceneBuild, links: LinkParams[]): void {
  links.forEach((link, i) => {
    const jid = build.revoluteIds[i];
    if (jid !== undefined) {
      world.updateRevoluteJoint(jid, jointPatch(link));
    }
  });
}

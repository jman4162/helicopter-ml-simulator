import { describe, it, expect } from 'vitest';
import { defaultParams, initialState, step, HeliState } from '../physics/heli';
import { HoverController } from './autopilot';
import { fromAxisAngle, multiply } from '../math/quaternion';
import { length, vec3 } from '../math/vec3';

const P = defaultParams();
const DT = 1 / 50; // control rate

const horizontalError = (s: HeliState, target: { x: number; y: number; z: number }) =>
  Math.hypot(s.position.x - target.x, s.position.y - target.y, s.position.z - target.z);

describe('HoverController (nonlinear closed loop)', () => {
  it('drives an attitude + position perturbation back to the setpoint', () => {
    const ctrl = new HoverController(P, DT);
    const target = { x: 0, y: 0, z: -6 };
    ctrl.setSetpoint(target);

    // Start tilted 25° and displaced.
    let s: HeliState = {
      ...initialState(P),
      position: vec3(2, -1.5, -6),
      orientation: multiply(initialState(P).orientation, fromAxisAngle(vec3(0.44, 0.3, 0))),
      velBody: vec3(1, 0.5, 0),
    };

    for (let i = 0; i < 600; i++) {
      const u = ctrl.control(s);
      s = step(P, s, u, DT);
    }

    expect(horizontalError(s, target)).toBeLessThan(0.1);
    expect(length(s.velBody)).toBeLessThan(0.1);
    expect(length(s.rateBody)).toBeLessThan(0.1);
  });

  it('recovers from a wind impulse and returns to hover', () => {
    const ctrl = new HoverController(P, DT);
    const target = { x: 0, y: 0, z: -6 };
    ctrl.setSetpoint(target);
    let s: HeliState = { ...initialState(P), position: vec3(0, 0, -6) };

    // Settle, then hit it with a 0.3 s lateral + vertical gust.
    let maxExcursion = 0;
    for (let i = 0; i < 800; i++) {
      const u = ctrl.control(s);
      const gust = i >= 100 && i < 115 ? { lin: vec3(6, 4, -3) } : undefined;
      s = step(P, s, u, DT, gust);
      if (i > 115) maxExcursion = Math.max(maxExcursion, horizontalError(s, target));
    }

    expect(maxExcursion).toBeLessThan(5); // gust pushes it, but bounded
    expect(horizontalError(s, target)).toBeLessThan(0.1); // and it returns
  });

  it('holds a commanded heading offset', () => {
    const ctrl = new HoverController(P, DT);
    ctrl.setSetpoint({ x: 0, y: 0, z: -6 }, Math.PI / 2); // face east
    let s: HeliState = { ...initialState(P), position: vec3(0, 0, -6) };
    for (let i = 0; i < 600; i++) s = step(P, s, ctrl.control(s), DT);
    // Yaw should approach +90°.
    const yaw = Math.atan2(
      2 * (s.orientation.w * s.orientation.z + s.orientation.x * s.orientation.y),
      1 - 2 * (s.orientation.y * s.orientation.y + s.orientation.z * s.orientation.z),
    );
    expect(Math.abs(yaw - Math.PI / 2)).toBeLessThan(0.05);
  });
});

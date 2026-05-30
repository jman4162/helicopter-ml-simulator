import { describe, it, expect } from 'vitest';
import { defaultParams, initialState, step, HeliState } from '../physics/heli';
import { TrajectoryController } from './autopilot';
import { buildManeuver } from './maneuvers';
import { boxminus } from './linearize';
import { rotate } from '../math/quaternion';
import { vec3 } from '../math/vec3';

const P = defaultParams();
const DT = 1 / 50;

const start = (): HeliState => ({ ...initialState(P), position: vec3(0, 0, -8) });

const trackingErrorNorm = (s: HeliState, ref: HeliState): number => {
  const e = boxminus(s, ref);
  return Math.hypot(...e);
};

describe('maneuver tracking (TVLQR)', () => {
  it('forward flight: reaches the target and tracks tightly despite a start offset', () => {
    const s0 = start();
    const ref = buildManeuver('forward', P, s0, DT);
    const ctrl = new TrajectoryController(P, ref.states, ref.controls, DT);

    // Perturb the actual start so the controller has to work.
    let s: HeliState = { ...s0, position: vec3(1, 1, -8) };
    let maxErr = 0;
    for (let t = 0; t < ref.controls.length; t++) {
      s = step(P, s, ctrl.control(s, t), DT);
      maxErr = Math.max(maxErr, trackingErrorNorm(s, ref.states[t + 1]));
    }
    const finalRef = ref.states[ref.states.length - 1];
    expect(finalRef.position.x).toBeGreaterThan(12); // the maneuver really moves forward
    expect(trackingErrorNorm(s, finalRef)).toBeLessThan(0.3); // converged onto it
    expect(maxErr).toBeLessThan(2.5); // bounded throughout
  });

  it('flip: the reference inverts, and the controller stays bounded under a gust', () => {
    const s0 = start();
    const ref = buildManeuver('flip', P, s0, DT);

    // The reference must actually go inverted (body-up points down in NED).
    const inverted = ref.states.some((st) => rotate(st.orientation, vec3(0, 0, -1)).z > 0.5);
    expect(inverted).toBe(true);

    const ctrl = new TrajectoryController(P, ref.states, ref.controls, DT);
    let s: HeliState = s0;
    let maxErr = 0;
    for (let t = 0; t < ref.controls.length; t++) {
      const gust = t >= 20 && t < 28 ? { lin: vec3(3, 0, 0) } : undefined;
      s = step(P, s, ctrl.control(s, t), DT, gust);
      maxErr = Math.max(maxErr, trackingErrorNorm(s, ref.states[t + 1]));
    }
    expect(Number.isFinite(maxErr)).toBe(true);
    expect(maxErr).toBeLessThan(4); // tracks the aggressive maneuver without blowing up
  });
});

import {
  Control,
  HeliParams,
  HeliState,
  clampControl,
  hoverControl,
  initialState,
} from '../physics/heli';
import { Matrix, zeros } from '../math/matrix';
import { Quat } from '../math/quaternion';
import { fromAxisAngle } from '../math/quaternion';
import { vec3 } from '../math/vec3';
import { boxminus, CTRL_DIM, ERR_DIM, linearizeDiscrete } from './linearize';
import { dlqr, feedback, tvlqr } from './lqr';

const diag = (values: number[]): Matrix => {
  const m = zeros(values.length, values.length);
  values.forEach((v, i) => (m[i][i] = v));
  return m;
};

const addControl = (a: Control, d: number[]): Control => ({
  u1: a.u1 + d[0],
  u2: a.u2 + d[1],
  u3: a.u3 + d[2],
  u4: a.u4 + d[3],
});

/**
 * LQR weights over the 12-D error state [pos(3), attitude(3), vel(3), rate(3)]
 * and 4 controls. Attitude is weighted hardest — it is the fast inner loop that
 * everything else depends on; position is the slow outer loop.
 */
export interface Weights {
  pos: number;
  att: number;
  vel: number;
  rate: number;
  ctrl: number;
}

export const defaultWeights: Weights = { pos: 3, att: 8, vel: 1.5, rate: 0.4, ctrl: 0.6 };

const Qof = (w: Weights): Matrix =>
  diag([w.pos, w.pos, w.pos, w.att, w.att, w.att, w.vel, w.vel, w.vel, w.rate, w.rate, w.rate]);

const Rof = (w: Weights): Matrix => diag([w.ctrl, w.ctrl, w.ctrl, w.ctrl]);

/** Yaw (heading) of a level-ish body->world NED quaternion. */
const yawOf = (q: Quat): number =>
  Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));

/** A level orientation with the given heading (rotation about world-down). */
const levelOrientation = (yaw: number): Quat => fromAxisAngle(vec3(0, 0, yaw));

/**
 * Hold a hover setpoint with an infinite-horizon LQR. The gain is computed once
 * by linearizing the discrete dynamics at the level hover trim; since the
 * dynamics are yaw-invariant when level, the same gain holds for any heading.
 */
export class HoverController {
  private readonly K: Matrix;
  private readonly uRef: Control;
  private ref: HeliState;

  constructor(
    private readonly params: HeliParams,
    dt: number,
    weights: Weights = defaultWeights,
  ) {
    this.uRef = hoverControl(params);
    this.ref = initialState(params);
    const { A, B } = linearizeDiscrete(params, this.ref, this.uRef, dt);
    this.K = dlqr(A, B, Qof(weights), Rof(weights)).K;
  }

  /** Aim to hover at `position` (world NED) holding heading `yaw` (default level/north). */
  setSetpoint(position: { x: number; y: number; z: number }, yaw = 0): void {
    this.ref = {
      position: vec3(position.x, position.y, position.z),
      orientation: levelOrientation(yaw),
      velBody: vec3(0, 0, 0),
      rateBody: vec3(0, 0, 0),
      rotorSpeed: this.params.rotorTarget,
    };
  }

  /** Engage from the current state: hold the current position and heading. */
  engageFrom(state: HeliState): void {
    this.setSetpoint(state.position, yawOf(state.orientation));
  }

  control(state: HeliState): Control {
    const err = boxminus(state, this.ref);
    return clampControl(addControl(this.uRef, feedback(this.K, err)));
  }

  get setpoint(): HeliState {
    return this.ref;
  }
}

/**
 * Track a (feasible) reference trajectory with time-varying LQR. The reference
 * is a state+control sequence; gains are precomputed by a backward Riccati pass
 * over the per-step linearizations. Holds the final reference state after the end.
 */
export class TrajectoryController {
  private readonly Ks: Matrix[];
  constructor(
    params: HeliParams,
    private readonly refStates: HeliState[],
    private readonly refControls: Control[],
    dt: number,
    weights: Weights = defaultWeights,
  ) {
    const N = refControls.length;
    const As: Matrix[] = new Array(N);
    const Bs: Matrix[] = new Array(N);
    for (let t = 0; t < N; t++) {
      const { A, B } = linearizeDiscrete(params, refStates[t], refControls[t], dt);
      As[t] = A;
      Bs[t] = B;
    }
    const Qf = Qof({ ...weights, pos: weights.pos * 4, att: weights.att * 2 });
    this.Ks = tvlqr(As, Bs, Qof(weights), Rof(weights), Qf);
  }

  get length(): number {
    return this.refControls.length;
  }

  /** Control at trajectory step `t` (clamped to the last step when past the end). */
  control(state: HeliState, t: number): Control {
    const i = Math.min(Math.max(t, 0), this.refControls.length - 1);
    const err = boxminus(state, this.refStates[i]);
    return clampControl(addControl(this.refControls[i], feedback(this.Ks[i], err)));
  }

  referenceState(t: number): HeliState {
    return this.refStates[Math.min(Math.max(t, 0), this.refStates.length - 1)];
  }
}

export { ERR_DIM, CTRL_DIM };

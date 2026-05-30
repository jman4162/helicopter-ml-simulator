import { Control, G, HeliParams, HeliState, hoverControl, initialState, step } from '../physics/heli';
import { rotateInverse } from '../math/quaternion';
import { Matrix, matMul, matVec, solve, transpose } from '../math/matrix';
import { makeRng } from './synthetic';

/**
 * System identification (Abbeel, Coates & Ng §3.3). The A/B/C/D coefficients of
 * the dynamics model (Eq. 1) appear *linearly* in the accelerations, so they can
 * be recovered from flight logs by linear least squares: move the known
 * kinematic terms (Coriolis, gravity-in-body, gyroscopic coupling) to the
 * left-hand side, then regress the residual acceleration onto the relevant
 * state/control regressors. This closes the loop — the same model the simulator
 * integrates can be re-learned from data it generates.
 */

export interface FlightLog {
  states: HeliState[];
  controls: Control[];
  dt: number;
}

/**
 * Fly a rich, multi-frequency excitation on all four controls around hover, so
 * every axis (and thus every coefficient) is well excited. Returns the log.
 */
export const generateExcitation = (
  params: HeliParams,
  dt: number,
  seconds: number,
  seed = 1,
): FlightLog => {
  const rng = makeRng(seed);
  const ph = [rng(), rng(), rng(), rng()].map((r) => r * Math.PI * 2);
  const hover = hoverControl(params).u4;
  const steps = Math.round(seconds / dt);

  const states: HeliState[] = [initialState(params)];
  const controls: Control[] = [];
  for (let k = 0; k < steps; k++) {
    const t = k * dt;
    const c: Control = {
      u1: 0.55 * Math.sin(2 * Math.PI * 0.7 * t + ph[0]) + 0.3 * Math.sin(2 * Math.PI * 1.9 * t),
      u2: 0.55 * Math.sin(2 * Math.PI * 0.5 * t + ph[1]) + 0.3 * Math.sin(2 * Math.PI * 1.3 * t),
      u3: 0.5 * Math.sin(2 * Math.PI * 0.9 * t + ph[2]),
      u4: hover + 0.3 * Math.sin(2 * Math.PI * 0.4 * t + ph[3]),
    };
    controls.push(c);
    states.push(step(params, states[k], c, dt));
  }
  return { states, controls, dt };
};

/** Least squares: solve minₜ ‖Φθ − y‖² via the normal equations ΦᵀΦ θ = Φᵀy. */
const lstsq = (Phi: Matrix, y: number[]): number[] => {
  const Pt = transpose(Phi);
  return solve(matMul(Pt, Phi), matVec(Pt, y));
};

export interface SysIdResult {
  params: HeliParams; // fitted parameters (kinematic structure copied from `prior`)
  residual: number; // RMS one-step acceleration residual of the fit
}

/**
 * Identify the linear coefficients from a flight log. The gyroscopic-coupling
 * inertia ratios and rotor governor are treated as known structure (copied from
 * `prior`); everything in A/B/C/D and D0 is fit. Optional `sensorNoise` adds
 * Gaussian noise to the logged states first, mimicking real measurements.
 */
export const identify = (log: FlightLog, prior: HeliParams, sensorNoise = 0, seed = 99): SysIdResult => {
  const rng = makeRng(seed);
  const gz = (s: HeliState) => rotateInverse(s.orientation, { x: 0, y: 0, z: G });
  const noise = () => (sensorNoise > 0 ? (Math.sqrt(-2 * Math.log(Math.max(rng(), 1e-12))) * Math.cos(2 * Math.PI * rng())) * sensorNoise : 0);

  // Optionally corrupt the logged velocities/rates with measurement noise.
  const vel = log.states.map((s) => ({ x: s.velBody.x + noise(), y: s.velBody.y + noise(), z: s.velBody.z + noise() }));
  const rate = log.states.map((s) => ({ x: s.rateBody.x + noise(), y: s.rateBody.y + noise(), z: s.rateBody.z + noise() }));

  const dt = log.dt;
  // Regression rows per equation, keyed by the coefficients they solve for.
  const rows: Record<string, { phi: number[]; y: number }[]> = {
    u: [], v: [], w: [], p: [], q: [], r: [],
  };

  // Central differences for the body accelerations (accurate to O(dt²)).
  for (let t = 1; t < log.states.length - 1; t++) {
    const s = log.states[t];
    const c = log.controls[t];
    const du = { x: (vel[t + 1].x - vel[t - 1].x) / (2 * dt), y: (vel[t + 1].y - vel[t - 1].y) / (2 * dt), z: (vel[t + 1].z - vel[t - 1].z) / (2 * dt) };
    const dw = { x: (rate[t + 1].x - rate[t - 1].x) / (2 * dt), y: (rate[t + 1].y - rate[t - 1].y) / (2 * dt), z: (rate[t + 1].z - rate[t - 1].z) / (2 * dt) };
    const u = vel[t].x, v = vel[t].y, w = vel[t].z;
    const p = rate[t].x, q = rate[t].y, r = rate[t].z;
    const g = gz(s);

    // Known kinematic terms moved to the LHS.
    rows.u.push({ phi: [u], y: du.x - (v * r - w * q) - g.x }); // Ax
    rows.v.push({ phi: [v, 1], y: du.y - (w * p - u * r) - g.y }); // Ay, D0
    rows.w.push({ phi: [w, c.u4, 1], y: du.z - (u * q - v * p) - g.z }); // Az, C4, D4
    rows.p.push({ phi: [p, c.u1, 1], y: dw.x - q * r * ((prior.Iyy - prior.Izz) / prior.Ixx) }); // Bx, C1, D1
    rows.q.push({ phi: [q, c.u2, 1], y: dw.y - p * r * ((prior.Izz - prior.Ixx) / prior.Iyy) }); // By, C2, D2
    rows.r.push({ phi: [r, c.u3, 1], y: dw.z - p * q * ((prior.Ixx - prior.Iyy) / prior.Izz) }); // Bz, C3, D3
  }

  const fit = (key: string) => {
    const data = rows[key];
    const theta = lstsq(data.map((d) => d.phi), data.map((d) => d.y));
    let ss = 0;
    for (const d of data) {
      const pred = d.phi.reduce((acc, x, i) => acc + x * theta[i], 0);
      ss += (pred - d.y) ** 2;
    }
    return { theta, rms: Math.sqrt(ss / data.length) };
  };

  const fu = fit('u'), fv = fit('v'), fw = fit('w'), fp = fit('p'), fq = fit('q'), fr = fit('r');
  const residual = Math.sqrt([fu, fv, fw, fp, fq, fr].reduce((s, f) => s + f.rms ** 2, 0) / 6);

  const params: HeliParams = {
    ...prior,
    Ax: fu.theta[0],
    Ay: fv.theta[0], D0: fv.theta[1],
    Az: fw.theta[0], C4: fw.theta[1], D4: fw.theta[2],
    Bx: fp.theta[0], C1: fp.theta[1], D1: fp.theta[2],
    By: fq.theta[0], C2: fq.theta[1], D2: fq.theta[2],
    Bz: fr.theta[0], C3: fr.theta[1], D3: fr.theta[2],
  };
  return { params, residual };
};

/**
 * Simulation-accuracy criterion (§3.3): roll the fitted model forward from the
 * log's first state under the logged controls, and measure how far its predicted
 * positions drift from the true flight (multi-step open-loop prediction RMSE).
 */
export const predictionError = (
  log: FlightLog,
  fitted: HeliParams,
): { rmse: number; predicted: HeliState[] } => {
  const predicted: HeliState[] = [log.states[0]];
  let s = log.states[0];
  let se = 0;
  for (let k = 0; k < log.controls.length; k++) {
    s = step(fitted, s, log.controls[k], log.dt);
    predicted.push(s);
    const a = s.position;
    const b = log.states[k + 1].position;
    se += (a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2;
  }
  return { rmse: Math.sqrt(se / log.controls.length), predicted };
};

/** Open-loop rollout of a model under a control sequence from a start state. */
export const rollout = (params: HeliParams, start: HeliState, controls: Control[], dt: number): HeliState[] => {
  const states = [start];
  for (let k = 0; k < controls.length; k++) states.push(step(params, states[k], controls[k], dt));
  return states;
};

/**
 * A gentle, low-amplitude excitation that swoops rather than tumbles — a
 * readable held-out test flight for visually comparing the true and fitted
 * models (they should trace the same path if identification succeeded).
 */
export const gentleTestFlight = (params: HeliParams, dt: number, seconds: number, seed = 7): Control[] => {
  const rng = makeRng(seed);
  const ph = [rng(), rng(), rng(), rng()].map((r) => r * Math.PI * 2);
  const hover = hoverControl(params).u4;
  const steps = Math.round(seconds / dt);
  return Array.from({ length: steps }, (_, k) => {
    const t = k * dt;
    return {
      u1: 0.3 * Math.sin(2 * Math.PI * 0.22 * t + ph[0]) + 0.12 * Math.sin(2 * Math.PI * 0.6 * t),
      u2: 0.3 * Math.sin(2 * Math.PI * 0.18 * t + ph[1]) + 0.12 * Math.sin(2 * Math.PI * 0.5 * t),
      u3: 0.16 * Math.sin(2 * Math.PI * 0.4 * t + ph[2]),
      u4: hover + 0.14 * Math.sin(2 * Math.PI * 0.16 * t + ph[3]),
    };
  });
};

/** Max relative error across the fitted vs true linear coefficients (for reporting). */
export const coefficientError = (fitted: HeliParams, truth: HeliParams): number => {
  const keys: (keyof HeliParams)[] = ['Ax', 'Ay', 'Az', 'Bx', 'By', 'Bz', 'C1', 'C2', 'C3', 'C4'];
  let worst = 0;
  for (const k of keys) {
    const denom = Math.abs(truth[k] as number) || 1;
    worst = Math.max(worst, Math.abs((fitted[k] as number) - (truth[k] as number)) / denom);
  }
  return worst;
};

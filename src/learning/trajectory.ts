import { sampleAt, Traj } from './synthetic';

/**
 * Apprenticeship trajectory learning (Abbeel, Coates & Ng §4), in the paper's
 * linear-Gaussian spirit: the hidden "intended" trajectory is inferred from
 * several noisy, time-warped demonstrations by alternating
 *   1. time-alignment of each demo to the current estimate (dynamic time warping
 *      with 1–3 step transitions, the paper's Eq. 4 / Needleman–Wunsch DP), and
 *   2. a Kalman (RTS) smoother that fuses all aligned observations into a clean
 *      trajectory, using a generic constant-velocity "dynamics" prior (the paper
 *      notes the dynamics model used for trajectory learning need not be accurate).
 * This is the EM loop: alignment is the discrete latent variable, smoothing is
 * the continuous E-step. With known ground truth we can watch it converge.
 */

export interface LearnOptions {
  iterations: number;
  /** Hidden-timeline length; default ≈ 2× the average demo length (paper). */
  hiddenLength?: number;
  /** Process-noise (acceleration) variance of the smoothing prior — smaller = smoother. */
  processVar: number;
  /** Per-sample observation variance. */
  obsVar: number;
}

export const defaultLearnOptions: LearnOptions = {
  iterations: 12,
  processVar: 0.02,
  obsVar: 0.1,
};

const sqDist = (a: number[], b: number[]): number => {
  let s = 0;
  for (let d = 0; d < a.length; d++) {
    const e = a[d] - b[d];
    s += e * e;
  }
  return s;
};

const resample = (traj: Traj, T: number): Traj =>
  Array.from({ length: T }, (_, t) => sampleAt(traj, (t * (traj.length - 1)) / (T - 1)));

/**
 * Dynamic time warping: map each demo frame j to a hidden index τ[j], monotonic,
 * advancing 1–3 hidden steps per demo frame, anchored at both ends. Cost is the
 * squared distance from the demo frame to the current estimate at that index.
 */
export const dtwAlign = (demo: Traj, estimate: Traj): number[] => {
  const N = demo.length;
  const T = estimate.length;
  const INF = Infinity;
  const dp = Array.from({ length: N }, () => new Array<number>(T).fill(INF));
  const back = Array.from({ length: N }, () => new Array<number>(T).fill(-1));

  dp[0][0] = sqDist(demo[0], estimate[0]); // anchor start at hidden 0
  for (let j = 1; j < N; j++) {
    for (let t = j; t < T; t++) {
      let best = INF;
      let arg = -1;
      for (let step = 1; step <= 3; step++) {
        const pt = t - step;
        if (pt < 0) break;
        if (dp[j - 1][pt] < best) {
          best = dp[j - 1][pt];
          arg = pt;
        }
      }
      if (arg >= 0) {
        dp[j][t] = best + sqDist(demo[j], estimate[t]);
        back[j][t] = arg;
      }
    }
  }

  // Prefer the exact end anchor (T-1); fall back to the best-scoring last frame.
  let endT = T - 1;
  if (!Number.isFinite(dp[N - 1][endT])) {
    let best = INF;
    for (let t = 0; t < T; t++)
      if (dp[N - 1][t] < best) {
        best = dp[N - 1][t];
        endT = t;
      }
  }

  const tau = new Array<number>(N);
  let t = endT;
  for (let j = N - 1; j >= 0; j--) {
    tau[j] = t;
    t = back[j][t];
    if (t < 0) t = 0;
  }
  return tau;
};

interface Obs {
  y: number;
  R: number;
}

/**
 * Scalar constant-velocity RTS smoother. `obs[t]` is the fused observation at
 * hidden time t (or null if no demo aligned there). Returns smoothed positions.
 */
const rtsSmoothScalar = (obs: (Obs | null)[], q: number): number[] => {
  const T = obs.length;
  // White-acceleration process noise (dt = 1): Q = q·[[1/3,1/2],[1/2,1]].
  const Q = [
    [q / 3, q / 2],
    [q / 2, q],
  ];
  // Filtered (xf,Pf) and one-step-predicted (xp,Pp) moments.
  const xf: number[][] = [];
  const Pf: number[][][] = [];
  const xp: number[][] = [];
  const Pp: number[][][] = [];

  let x = [obs.find((o) => o)?.y ?? 0, 0];
  let P = [
    [1e3, 0],
    [0, 1e3],
  ];

  for (let t = 0; t < T; t++) {
    if (t > 0) {
      // Predict: x = F x, P = F P Fᵀ + Q  (F = [[1,1],[0,1]]).
      x = [x[0] + x[1], x[1]];
      const p00 = P[0][0] + P[0][1] + P[1][0] + P[1][1];
      const p01 = P[0][1] + P[1][1];
      const p10 = P[1][0] + P[1][1];
      const p11 = P[1][1];
      P = [
        [p00 + Q[0][0], p01 + Q[0][1]],
        [p10 + Q[1][0], p11 + Q[1][1]],
      ];
    }
    xp[t] = [...x];
    Pp[t] = [P[0].slice(), P[1].slice()];

    const o = obs[t];
    if (o) {
      // Update with scalar measurement (H = [1,0]).
      const S = P[0][0] + o.R;
      const k0 = P[0][0] / S;
      const k1 = P[1][0] / S;
      const innov = o.y - x[0];
      x = [x[0] + k0 * innov, x[1] + k1 * innov];
      const p00 = P[0][0] - k0 * P[0][0];
      const p01 = P[0][1] - k0 * P[0][1];
      const p10 = P[1][0] - k1 * P[0][0];
      const p11 = P[1][1] - k1 * P[0][1];
      P = [
        [p00, p01],
        [p10, p11],
      ];
    }
    xf[t] = [...x];
    Pf[t] = [P[0].slice(), P[1].slice()];
  }

  // RTS backward pass.
  const xs: number[][] = new Array(T);
  xs[T - 1] = xf[T - 1];
  let smoothedNext = xf[T - 1];
  for (let t = T - 2; t >= 0; t--) {
    const Ppn = Pp[t + 1];
    // C = Pf[t] Fᵀ (Pp[t+1])⁻¹ ; Fᵀ = [[1,0],[1,1]].
    const PfFt = [
      [Pf[t][0][0] + Pf[t][0][1], Pf[t][0][1]],
      [Pf[t][1][0] + Pf[t][1][1], Pf[t][1][1]],
    ];
    const det = Ppn[0][0] * Ppn[1][1] - Ppn[0][1] * Ppn[1][0];
    const inv = [
      [Ppn[1][1] / det, -Ppn[0][1] / det],
      [-Ppn[1][0] / det, Ppn[0][0] / det],
    ];
    const C = [
      [PfFt[0][0] * inv[0][0] + PfFt[0][1] * inv[1][0], PfFt[0][0] * inv[0][1] + PfFt[0][1] * inv[1][1]],
      [PfFt[1][0] * inv[0][0] + PfFt[1][1] * inv[1][0], PfFt[1][0] * inv[0][1] + PfFt[1][1] * inv[1][1]],
    ];
    const dx0 = smoothedNext[0] - xp[t + 1][0];
    const dx1 = smoothedNext[1] - xp[t + 1][1];
    const cur = [xf[t][0] + C[0][0] * dx0 + C[0][1] * dx1, xf[t][1] + C[1][0] * dx0 + C[1][1] * dx1];
    xs[t] = cur;
    smoothedNext = cur;
  }
  return xs.map((s) => s[0]);
};

export interface LearnResult {
  estimate: Traj; // recovered intended trajectory (hiddenLength × dim)
  initial: Traj; // naive time-unaligned average (the baseline EM improves on)
  snapshots: Traj[]; // estimate after each step (snapshots[0] = initial), for animation
  alignments: number[][]; // τ per demo
  history: { iteration: number; rmse?: number }[];
  hiddenLength: number;
}

/** Naive baseline: resample every demo to length T and average — ignores time
 *  warp, so it blurs the path. EM exists precisely to beat this. */
const naiveAverage = (demos: Traj[], T: number, dim: number): Traj => {
  const out = Array.from({ length: T }, () => new Array<number>(dim).fill(0));
  for (const demo of demos) {
    const r = resample(demo, T);
    for (let t = 0; t < T; t++) for (let d = 0; d < dim; d++) out[t][d] += r[t][d] / demos.length;
  }
  return out;
};

/** RMSE between two equal-length trajectories. */
export const rmse = (a: Traj, b: Traj): number => {
  let s = 0;
  let n = 0;
  for (let t = 0; t < a.length; t++)
    for (let d = 0; d < a[t].length; d++) {
      const e = a[t][d] - b[t][d];
      s += e * e;
      n++;
    }
  return Math.sqrt(s / n);
};

/**
 * Path-recovery RMSE: how close two trajectories are as *paths*, independent of
 * time parameterization (the learned hidden timeline need not match the ground
 * truth's). Uses classic DTW (either sequence may "wait"), so it measures pure
 * shape similarity regardless of length or local speed — unlike the forward-only
 * `dtwAlign` used inside the learning. This is the honest "did we recover the
 * trajectory?" metric.
 */
export const pathRmse = (a: Traj, b: Traj): number => {
  const N = a.length;
  const M = b.length;
  const D = Array.from({ length: N }, () => new Array<number>(M).fill(Infinity));
  D[0][0] = sqDist(a[0], b[0]);
  for (let i = 1; i < N; i++) D[i][0] = D[i - 1][0] + sqDist(a[i], b[0]);
  for (let j = 1; j < M; j++) D[0][j] = D[0][j - 1] + sqDist(a[0], b[j]);
  for (let i = 1; i < N; i++)
    for (let j = 1; j < M; j++)
      D[i][j] = sqDist(a[i], b[j]) + Math.min(D[i - 1][j], D[i][j - 1], D[i - 1][j - 1]);

  // Backtrack the optimal warping path, averaging matched squared distances.
  let i = N - 1;
  let j = M - 1;
  let sum = 0;
  let count = 0;
  while (i > 0 || j > 0) {
    sum += sqDist(a[i], b[j]);
    count++;
    if (i === 0) j--;
    else if (j === 0) i--;
    else {
      const m = Math.min(D[i - 1][j], D[i][j - 1], D[i - 1][j - 1]);
      if (m === D[i - 1][j - 1]) {
        i--;
        j--;
      } else if (m === D[i - 1][j]) i--;
      else j--;
    }
  }
  sum += sqDist(a[0], b[0]);
  count++;
  return Math.sqrt(sum / count);
};

/**
 * Run the EM loop. If `truth` is provided, each iteration records the RMSE of the
 * estimate against it (resampled to the hidden length) so convergence is visible.
 */
export const learnTrajectory = (
  demos: Traj[],
  opts: LearnOptions = defaultLearnOptions,
  truth?: Traj,
): LearnResult => {
  const dim = demos[0][0].length;
  const avgLen = demos.reduce((s, d) => s + d.length, 0) / demos.length;
  const T = opts.hiddenLength ?? Math.round(2 * avgLen);

  const initial = naiveAverage(demos, T, dim);
  let estimate: Traj = initial;
  const snapshots: Traj[] = [initial];
  const history: { iteration: number; rmse?: number }[] = [
    { iteration: 0, rmse: truth ? pathRmse(estimate, truth) : undefined }, // naive baseline
  ];
  let alignments: number[][] = [];

  for (let it = 1; it <= opts.iterations; it++) {
    // E-step (discrete): align every demo to the current estimate.
    alignments = demos.map((demo) => dtwAlign(demo, estimate));

    // Gather fused observations per hidden time, per dimension.
    const obs: (Obs | null)[][] = Array.from({ length: dim }, () => new Array(T).fill(null));
    const sums = Array.from({ length: T }, () => new Array<number>(dim).fill(0));
    const counts = new Array<number>(T).fill(0);
    demos.forEach((demo, k) => {
      const tau = alignments[k];
      for (let j = 0; j < demo.length; j++) {
        const t = tau[j];
        counts[t]++;
        for (let d = 0; d < dim; d++) sums[t][d] += demo[j][d];
      }
    });
    for (let t = 0; t < T; t++) {
      if (counts[t] === 0) continue;
      for (let d = 0; d < dim; d++) {
        obs[d][t] = { y: sums[t][d] / counts[t], R: opts.obsVar / counts[t] };
      }
    }

    // E-step (continuous): smooth each dimension into the new estimate.
    const cols = Array.from({ length: dim }, (_, d) => rtsSmoothScalar(obs[d], opts.processVar));
    estimate = Array.from({ length: T }, (_, t) => cols.map((c) => c[t]));

    snapshots.push(estimate);
    history.push({ iteration: it, rmse: truth ? pathRmse(estimate, truth) : undefined });
  }

  return { estimate, initial, snapshots, alignments, history, hiddenLength: T };
};

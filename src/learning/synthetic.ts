/**
 * Synthetic "expert" demonstrations for the apprenticeship-learning demo.
 *
 * The paper learns an intended trajectory from real, imperfect human RC-pilot
 * flights. We don't have a pilot, so we manufacture the same situation: take an
 * ideal trajectory (the hidden ground truth the algorithm must recover) and
 * produce M corrupted copies, each with
 *   - a non-uniform time warp (the pilot flies parts faster/slower, and demos
 *     differ in length) — this is the hard part the paper's time-alignment solves;
 *   - Gaussian per-sample measurement noise;
 *   - a slow positional drift (random walk) — the pilot can't hold an exact
 *     position, "demonstration drift" in the paper.
 * Because we know the ground truth, we can score how well learning recovers it.
 */

export type Traj = number[][]; // [frame][dim]

/** Deterministic, seedable PRNG (mulberry32) so demos/tests are reproducible. */
export const makeRng = (seed: number): (() => number) => {
  let a = seed >>> 0;
  return () => {
    a |= 0;
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
};

/** Standard-normal sample via Box–Muller, driven by a uniform rng. */
const gauss = (rng: () => number): number => {
  const u = Math.max(rng(), 1e-12);
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * rng());
};

/** Linear interpolation of a trajectory at a fractional frame index. */
export const sampleAt = (traj: Traj, f: number): number[] => {
  const n = traj.length;
  const x = Math.min(Math.max(f, 0), n - 1);
  const i = Math.floor(x);
  const frac = x - i;
  if (i >= n - 1) return [...traj[n - 1]];
  return traj[i].map((v, d) => v + (traj[i + 1][d] - v) * frac);
};

export interface DemoOptions {
  count: number; // number of demonstrations
  posNoise: number; // std of per-sample Gaussian noise
  driftStep: number; // std of the per-step random-walk drift
  lengthJitter: number; // fractional spread in demo length
  speedVar: number; // amount of non-uniform time warp (0 = uniform)
  seed: number;
}

export const defaultDemoOptions: DemoOptions = {
  count: 8,
  posNoise: 0.35,
  driftStep: 0.008,
  lengthJitter: 0.18,
  speedVar: 0.55,
  seed: 1,
};

/** A smooth, strictly-positive speed profile (sum of a few random sinusoids). */
const speedProfile = (rng: () => number, amount: number): ((u: number) => number) => {
  const waves = Array.from({ length: 3 }, () => ({
    amp: amount * (0.4 + 0.6 * rng()),
    freq: 1 + Math.floor(rng() * 3),
    phase: rng() * Math.PI * 2,
  }));
  return (u: number) => {
    let s = 1;
    for (const w of waves) s += w.amp * Math.sin(w.freq * 2 * Math.PI * u + w.phase);
    return Math.max(0.15, s); // keep speed positive
  };
};

/**
 * Generate demonstrations from an ideal trajectory. Returns the demos (each of
 * possibly different length) and the ground-truth `truth` they were warped from.
 */
export const generateDemos = (
  ideal: Traj,
  opts: DemoOptions = defaultDemoOptions,
): { demos: Traj[]; truth: Traj } => {
  const rng = makeRng(opts.seed);
  const T0 = ideal.length;
  const dim = ideal[0].length;
  const demos: Traj[] = [];

  for (let k = 0; k < opts.count; k++) {
    const len = Math.max(8, Math.round(T0 * (1 + (rng() - 0.5) * 2 * opts.lengthJitter)));
    const speed = speedProfile(rng, opts.speedVar);

    // Build a monotonic time warp by integrating the speed profile.
    const cum: number[] = [0];
    for (let j = 1; j < len; j++) cum.push(cum[j - 1] + speed(j / len));
    const total = cum[len - 1];

    const drift = new Array<number>(dim).fill(0);
    const demo: Traj = [];
    for (let j = 0; j < len; j++) {
      const phase = (cum[j] / total) * (T0 - 1);
      const base = sampleAt(ideal, phase);
      for (let d = 0; d < dim; d++) drift[d] += gauss(rng) * opts.driftStep;
      demo.push(base.map((v, d) => v + drift[d] + gauss(rng) * opts.posNoise));
    }
    demos.push(demo);
  }

  return { demos, truth: ideal.map((p) => [...p]) };
};

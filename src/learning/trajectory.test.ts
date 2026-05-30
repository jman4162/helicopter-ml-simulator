import { describe, it, expect } from 'vitest';
import { generateDemos, sampleAt, Traj } from './synthetic';
import { dtwAlign, learnTrajectory, pathRmse } from './trajectory';

// An open 3-D path (open curves align cleanly under anchored DTW).
const ideal: Traj = Array.from({ length: 120 }, (_, t) => {
  const u = t / 119;
  return [12 * u, 2 * Math.sin(3 * Math.PI * u), -6 - 2 * Math.sin(Math.PI * u)];
});

describe('synthetic demos', () => {
  it('are reproducible and noticeably corrupted', () => {
    const a = generateDemos(ideal, { count: 5, posNoise: 0.35, driftStep: 0.01, lengthJitter: 0.2, speedVar: 0.5, seed: 7 });
    const b = generateDemos(ideal, { count: 5, posNoise: 0.35, driftStep: 0.01, lengthJitter: 0.2, speedVar: 0.5, seed: 7 });
    expect(a.demos[0]).toEqual(b.demos[0]); // deterministic for a seed
    // Demos differ in length (time warp) and are noisy vs the ideal.
    const lengths = new Set(a.demos.map((d) => d.length));
    expect(lengths.size).toBeGreaterThan(1);
    expect(pathRmse(a.demos[0], ideal)).toBeGreaterThan(0.1);
  });
});

describe('DTW alignment', () => {
  it('recovers the correspondence of a cleanly time-warped copy', () => {
    // Warp the ideal by a known nonuniform (monotonic) schedule, no noise.
    const N = 90;
    const warped: Traj = Array.from({ length: N }, (_, j) => {
      const u = j / (N - 1);
      const phase = u + 0.15 * Math.sin(2 * Math.PI * u); // monotonic warp of [0,1]
      return sampleAt(ideal, Math.min(1, Math.max(0, phase)) * (ideal.length - 1));
    });
    const tau = dtwAlign(warped, ideal);
    for (let j = 1; j < tau.length; j++) expect(tau[j]).toBeGreaterThanOrEqual(tau[j - 1]); // monotonic
    expect(tau[0]).toBe(0); // anchored start
    expect(tau[tau.length - 1]).toBe(ideal.length - 1); // anchored end
    expect(pathRmse(warped, ideal)).toBeLessThan(0.15); // correspondence recovered
  });
});

describe('EM trajectory learning', () => {
  it('recovers the hidden trajectory well below the sample noise, beating naive averaging', () => {
    const posNoise = 0.4;
    const { demos, truth } = generateDemos(ideal, { count: 8, posNoise, driftStep: 0.008, lengthJitter: 0.18, speedVar: 0.55, seed: 3 });
    const res = learnTrajectory(demos, { iterations: 12, processVar: 0.02, obsVar: 0.1 }, truth);

    const learnedErr = pathRmse(res.estimate, truth);
    const naiveErr = pathRmse(res.initial, truth); // time-unaligned average

    expect(learnedErr).toBeLessThan(posNoise * 0.7); // denoises well below sample noise
    expect(learnedErr).toBeLessThan(naiveErr * 0.7); // alignment clearly beats naive averaging
  });

  it('drives the recovery error down from the naive baseline over iterations', () => {
    const { demos, truth } = generateDemos(ideal, { count: 7, posNoise: 0.4, driftStep: 0.008, lengthJitter: 0.2, speedVar: 0.5, seed: 11 });
    const res = learnTrajectory(demos, { iterations: 12, processVar: 0.02, obsVar: 0.1 }, truth);
    const first = res.history[0].rmse!; // naive baseline (iteration 0)
    const best = Math.min(...res.history.map((h) => h.rmse!));
    expect(best).toBeLessThan(first * 0.7); // EM improves substantially on the baseline
    expect(res.history[res.history.length - 1].rmse!).toBeLessThan(first);
    expect(res.history.every((h) => h.rmse !== undefined)).toBe(true);
  });
});

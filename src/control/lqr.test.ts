import { describe, it, expect } from 'vitest';
import { dlqr, tvlqr, feedback } from './lqr';
import { identity, matVec } from '../math/matrix';

describe('dlqr (infinite-horizon)', () => {
  it('matches the analytic scalar solution (golden ratio)', () => {
    // A=B=Q=R=1 ⇒ ARE root p = (1+√5)/2, K = p/(1+p).
    const { K, P } = dlqr([[1]], [[1]], [[1]], [[1]]);
    const golden = (1 + Math.sqrt(5)) / 2;
    expect(P[0][0]).toBeCloseTo(golden, 6);
    expect(K[0][0]).toBeCloseTo(golden / (1 + golden), 6);
  });

  it('stabilizes a double integrator (closed loop decays to zero)', () => {
    const dt = 0.1;
    const A = [
      [1, dt],
      [0, 1],
    ];
    const B = [[0], [dt]];
    const { K } = dlqr(A, B, identity(2), [[0.1]]);
    let x = [1, 0];
    for (let i = 0; i < 400; i++) {
      const u = feedback(K, x); // = -Kx
      x = [A[0][0] * x[0] + A[0][1] * x[1] + B[0][0] * u[0], A[1][0] * x[0] + A[1][1] * x[1] + B[1][0] * u[0]];
    }
    expect(Math.hypot(x[0], x[1])).toBeLessThan(1e-3);
  });
});

describe('tvlqr (finite-horizon)', () => {
  it('returns one gain per step and stabilizes a time-invariant system', () => {
    const dt = 0.1;
    const A = [
      [1, dt],
      [0, 1],
    ];
    const B = [[0], [dt]];
    const N = 200;
    const As = Array.from({ length: N }, () => A);
    const Bs = Array.from({ length: N }, () => B);
    const Ks = tvlqr(As, Bs, identity(2), [[0.1]], identity(2));
    expect(Ks.length).toBe(N);

    let x = [1, 0.5];
    for (let t = 0; t < N; t++) {
      const u = matVec(Ks[t], x).map((v) => -v);
      x = [A[0][0] * x[0] + A[0][1] * x[1] + B[0][0] * u[0], A[1][0] * x[0] + A[1][1] * x[1] + B[1][0] * u[0]];
    }
    expect(Math.hypot(x[0], x[1])).toBeLessThan(1e-2);
  });
});

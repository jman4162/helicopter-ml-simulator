import { Matrix, inverse, matAdd, matMul, matSub, matVec, symmetrize, transpose } from '../math/matrix';

/**
 * Discrete LQR. For the system δ_{t+1} = A δ_t + B δu_t with stage cost
 * δᵀQδ + δuᵀRδu, the optimal feedback is δu = −K δ. This is the linear-quadratic
 * machinery the paper's controller is built on (§6.1); here applied around a
 * (feasible) reference, which is the tracking form of Gauss–Newton LQR.
 */

const frob = (a: Matrix): number => {
  let s = 0;
  for (const row of a) for (const v of row) s += v * v;
  return Math.sqrt(s);
};

/** One Riccati backup: returns the gain K and the updated cost-to-go P. */
const riccatiStep = (A: Matrix, B: Matrix, Q: Matrix, R: Matrix, P: Matrix): { K: Matrix; P: Matrix } => {
  const At = transpose(A);
  const Bt = transpose(B);
  const BtP = matMul(Bt, P);
  const S = matAdd(R, matMul(BtP, B)); // R + BᵀPB
  const K = matMul(inverse(S), matMul(BtP, A)); // (R+BᵀPB)⁻¹ BᵀPA
  // P⁺ = Q + AᵀPA − AᵀPB K
  const AtP = matMul(At, P);
  const Pnext = symmetrize(matSub(matAdd(Q, matMul(AtP, A)), matMul(matMul(AtP, B), K)));
  return { K, P: Pnext };
};

/**
 * Infinite-horizon LQR: iterate the Riccati recursion to convergence. Used for
 * holding a steady setpoint (e.g. hover) — gives a constant gain K with crisp
 * disturbance rejection.
 */
export const dlqr = (
  A: Matrix,
  B: Matrix,
  Q: Matrix,
  R: Matrix,
  maxIters = 1000,
  tol = 1e-9,
): { K: Matrix; P: Matrix } => {
  let P = Q;
  let K: Matrix = [];
  for (let i = 0; i < maxIters; i++) {
    const next = riccatiStep(A, B, Q, R, P);
    const change = frob(matSub(next.P, P)) / (frob(P) + 1e-12);
    K = next.K;
    P = next.P;
    if (change < tol) break;
  }
  return { K, P };
};

/**
 * Finite-horizon time-varying LQR over a reference trajectory. `As[t]`, `Bs[t]`
 * are the per-step linearizations (length N); returns the per-step gains `Ks[t]`
 * via a single backward Riccati pass with terminal cost `Qf`.
 */
export const tvlqr = (As: Matrix[], Bs: Matrix[], Q: Matrix, R: Matrix, Qf: Matrix): Matrix[] => {
  const N = As.length;
  const Ks = new Array<Matrix>(N);
  let P = Qf;
  for (let t = N - 1; t >= 0; t--) {
    const { K, P: Pnext } = riccatiStep(As[t], Bs[t], Q, R, P);
    Ks[t] = K;
    P = Pnext;
  }
  return Ks;
};

/** Feedback control delta δu = −K·δ for an error vector δ. */
export const feedback = (K: Matrix, err: number[]): number[] => matVec(K, err).map((v) => -v);

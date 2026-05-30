/** Generic state vector for integration. */
export type State = number[];

/** Derivative function dy/dt = f(y). (Autonomous within a single step.) */
export type Deriv = (y: State) => State;

const axpy = (a: State, s: number, b: State): State => {
  const out = new Array<number>(a.length);
  for (let i = 0; i < a.length; i++) out[i] = a[i] + s * b[i];
  return out;
};

/**
 * Classic 4th-order Runge–Kutta step. Operates on a flat numeric state vector so
 * it can integrate the helicopter dynamics regardless of how state is structured.
 * Quaternion re-normalization (if any) is the caller's responsibility.
 */
export const rk4 = (f: Deriv, y: State, dt: number): State => {
  const k1 = f(y);
  const k2 = f(axpy(y, dt / 2, k1));
  const k3 = f(axpy(y, dt / 2, k2));
  const k4 = f(axpy(y, dt, k3));
  const out = new Array<number>(y.length);
  for (let i = 0; i < y.length; i++) {
    out[i] = y[i] + (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
  }
  return out;
};

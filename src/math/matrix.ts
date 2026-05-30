/**
 * Small dense matrix helpers (row-major number[][]). Sized for the control and
 * learning math (a few dozen rows at most), so clarity is favored over raw speed.
 */
export type Matrix = number[][];
export type Vector = number[];

export const zeros = (rows: number, cols: number): Matrix =>
  Array.from({ length: rows }, () => new Array<number>(cols).fill(0));

export const identity = (n: number): Matrix => {
  const m = zeros(n, n);
  for (let i = 0; i < n; i++) m[i][i] = 1;
  return m;
};

export const transpose = (a: Matrix): Matrix => {
  const rows = a.length;
  const cols = a[0].length;
  const out = zeros(cols, rows);
  for (let i = 0; i < rows; i++) for (let j = 0; j < cols; j++) out[j][i] = a[i][j];
  return out;
};

export const matAdd = (a: Matrix, b: Matrix): Matrix =>
  a.map((row, i) => row.map((v, j) => v + b[i][j]));

export const matSub = (a: Matrix, b: Matrix): Matrix =>
  a.map((row, i) => row.map((v, j) => v - b[i][j]));

export const matScale = (a: Matrix, s: number): Matrix => a.map((row) => row.map((v) => v * s));

/** Symmetrize: ½(A + Aᵀ). Keeps Riccati iterates numerically symmetric. */
export const symmetrize = (a: Matrix): Matrix => {
  const n = a.length;
  const out = zeros(n, n);
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) out[i][j] = 0.5 * (a[i][j] + a[j][i]);
  return out;
};

export const matMul = (a: Matrix, b: Matrix): Matrix => {
  const n = a.length;
  const m = b[0].length;
  const k = b.length;
  const out = zeros(n, m);
  for (let i = 0; i < n; i++) {
    for (let p = 0; p < k; p++) {
      const aip = a[i][p];
      if (aip === 0) continue;
      for (let j = 0; j < m; j++) out[i][j] += aip * b[p][j];
    }
  }
  return out;
};

export const matVec = (a: Matrix, x: Vector): Vector => {
  const out = new Array<number>(a.length).fill(0);
  for (let i = 0; i < a.length; i++) {
    let s = 0;
    for (let j = 0; j < x.length; j++) s += a[i][j] * x[j];
    out[i] = s;
  }
  return out;
};

/** Solve A x = b via Gaussian elimination with partial pivoting. */
export const solve = (A: Matrix, b: Vector): Vector => {
  const n = A.length;
  // Augmented copy.
  const M = A.map((row, i) => [...row, b[i]]);
  for (let col = 0; col < n; col++) {
    // Pivot.
    let pivot = col;
    for (let r = col + 1; r < n; r++) if (Math.abs(M[r][col]) > Math.abs(M[pivot][col])) pivot = r;
    if (Math.abs(M[pivot][col]) < 1e-12) throw new Error('matrix is singular');
    [M[col], M[pivot]] = [M[pivot], M[col]];
    // Eliminate.
    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const factor = M[r][col] / M[col][col];
      for (let c = col; c <= n; c++) M[r][c] -= factor * M[col][c];
    }
  }
  return M.map((row, i) => row[n] / M[i][i]);
};

export const inverse = (A: Matrix): Matrix => {
  const n = A.length;
  const I = identity(n);
  const cols = I[0].map((_, j) => solve(A, I.map((row) => row[j])));
  // cols[j] is the j-th column of the inverse.
  const out = zeros(n, n);
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) out[i][j] = cols[j][i];
  return out;
};

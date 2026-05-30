/**
 * Minimal 3-vector. Plain object so it is cheap to allocate and easy to read in
 * the physics equations, where component names (x/y/z) matter for clarity.
 */
export interface Vec3 {
  x: number;
  y: number;
  z: number;
}

export const vec3 = (x = 0, y = 0, z = 0): Vec3 => ({ x, y, z });

export const add = (a: Vec3, b: Vec3): Vec3 => ({ x: a.x + b.x, y: a.y + b.y, z: a.z + b.z });

export const sub = (a: Vec3, b: Vec3): Vec3 => ({ x: a.x - b.x, y: a.y - b.y, z: a.z - b.z });

export const scale = (a: Vec3, s: number): Vec3 => ({ x: a.x * s, y: a.y * s, z: a.z * s });

export const dot = (a: Vec3, b: Vec3): number => a.x * b.x + a.y * b.y + a.z * b.z;

export const cross = (a: Vec3, b: Vec3): Vec3 => ({
  x: a.y * b.z - a.z * b.y,
  y: a.z * b.x - a.x * b.z,
  z: a.x * b.y - a.y * b.x,
});

export const length = (a: Vec3): number => Math.sqrt(dot(a, a));

export const normalize = (a: Vec3): Vec3 => {
  const len = length(a);
  return len > 1e-12 ? scale(a, 1 / len) : vec3(0, 0, 0);
};

export const toArray = (a: Vec3): [number, number, number] => [a.x, a.y, a.z];

export const fromArray = (a: ArrayLike<number>, offset = 0): Vec3 => ({
  x: a[offset],
  y: a[offset + 1],
  z: a[offset + 2],
});

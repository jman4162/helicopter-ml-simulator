import { Traj } from './synthetic';

/**
 * A recognizable "airshow" ground-truth trajectory (NED world coords): a
 * figure-eight in the horizontal plane with an altitude wave. This is the hidden
 * intended trajectory the learning demo tries to recover from noisy demos.
 */
export const airshowPath = (samples = 160): Traj =>
  Array.from({ length: samples }, (_, t) => {
    const u = t / (samples - 1);
    const a = 2 * Math.PI * u;
    return [10 * Math.sin(a), 7 * Math.sin(2 * a), -11 - 3 * Math.sin(a)];
  });

/** Convert a [x,y,z][] trajectory to {x,y,z}[] for the viz layer. */
export const toXYZ = (traj: Traj): { x: number; y: number; z: number }[] =>
  traj.map((p) => ({ x: p[0], y: p[1], z: p[2] }));

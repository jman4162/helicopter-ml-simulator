# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

An interactive, educational **helicopter flight simulator** implementing Abbeel, Coates & Ng,
*Autonomous Helicopter Aerobatics through Apprenticeship Learning* (IJRR 2010, the included
`AbbeelCoatesNg_IJRR2010.pdf`). Web-first: TypeScript + Vite + Three.js, everything runs in the
browser. The project is built in phases; **Phases 1–3 (physics + manual flight; autonomous
LQR/MPC control; apprenticeship trajectory learning) are complete**, Phases 4–5 (system ID,
lessons) upcoming.

## Commands

- `npm run dev` — Vite dev server at http://localhost:5173 (fly it: W/S collective, arrows cyclic, A/D yaw, R reset, C camera).
- `npm test` — run the Vitest suite once. `npm run test:watch` for watch mode.
- `npm run typecheck` — `tsc --noEmit` (strict).
- `npm run build` — typecheck + production bundle to `dist/`.
- Run a single test file: `npx vitest run src/physics/heli.test.ts`. Single test: add `-t "<name pattern>"`.

## Architecture

Layered, with a strict dependency direction: **math → physics → {viz, ui} → main**. Lower layers
never import upward.

- `src/math/` — pure, dependency-free numerics: `vec3`, `quaternion` (body→world, axis-angle),
  `matrix` (dense solve/inverse, for upcoming control/learning), `integrate` (generic RK4 over a
  flat `number[]` state). All unit-tested against analytical results.
- `src/physics/heli.ts` — **the core**. Rigid-body dynamics of paper Eq. 1 in a body NED frame
  (x fwd, y right, z down). `accelerations()` is a verbatim transcription of the equations;
  `step()` integrates a 14-D flat state with RK4 and re-normalizes the quaternion. Coefficients in
  `defaultParams()` are hand-picked plausible values (Phase 4 will fit them from data via system
  ID). `hoverCollective()` gives the trim that cancels gravity.
- `src/viz/` — Three.js. `coords.ts` holds the **single** NED→Three.js frame conversion (a proper
  rotation `M`, det +1, so no mirroring) — all frame mapping goes through here. `scene.ts` builds
  the stylized helicopter in **body NED coordinates** and lets the root group's transform map it to
  screen, so geometry reads the same as the math.
- `src/control/` — autonomous control. `linearize.ts`: the 12-D error/tangent state
  (`boxplus`/`boxminus`, axis-angle attitude error) and finite-difference A/B Jacobians of the
  discrete dynamics. `lqr.ts`: `dlqr` (infinite-horizon Riccati, for setpoint hold) and `tvlqr`
  (finite-horizon backward pass, for trajectory tracking) + `feedback`. `autopilot.ts`:
  `HoverController` (constant-gain hover/setpoint hold) and `TrajectoryController` (TVLQR maneuver
  tracking). `maneuvers.ts`: feasible-by-construction references (model rollouts) for forward
  flight, square, in-place flip, loop.
- `src/learning/` — apprenticeship trajectory learning (paper §4), linear-Gaussian form.
  `synthetic.ts`: seeded generator turning an ideal trajectory into noisy, time-warped, drifting
  demos. `trajectory.ts`: `dtwAlign` (forward 1–3 step DTW for demo→hidden, Eq. 4), a
  constant-velocity RTS Kalman smoother, the `learnTrajectory` EM loop (records per-iteration
  snapshots + RMSE history), `pathRmse` (classic DTW, the path-similarity metric — note
  `dtwAlign` is NOT a valid metric for similar-length sequences). `airshow.ts`: the figure-eight
  ground-truth path. Dimension-general; the demo runs on 3-D position.
- `src/ui/` — `input.ts` (keyboard momentary cyclic/yaw + held collective; RC-style gamepad;
  autopilot keys H/1-4/G/M, learning key L), `hud.ts` (flight overlay + learning panel;
  quaternion→Euler), `styles.css`.
- `src/main.ts` — fixed-timestep (1/100 s) accumulator sim loop with an autopilot state machine
  (manual / hover-hold / maneuver), wind-gust injector, soft ground floor, and a separate
  apprenticeship-learning mode (L) that animates EM convergence; render once per rAF.

## Conventions that matter

- **Frames**: physics is NED (z down, gravity `+z`); altitude is `-position.z`. Three.js is y-up.
  Never mix them outside `src/viz/coords.ts`. When adding anything visual, build geometry in body
  NED coords and let the group transform handle the rest.
- **Quaternion convention**: `Quat` rotates body→world. `rotate()` = body→world, `rotateInverse()`
  = world→body. Always re-normalize after integrating.
- **Tests are the spec for the physics/math.** When changing dynamics, keep the sanity tests
  meaningful (hover trim is a fixed point, free-fall sign, gyroscopic-coupling sign, rate steady
  states). The strongest future test (Phase 3) is recovering a known ground-truth trajectory from
  synthetic noisy/time-warped demos.

## Paper section → code map (for upcoming phases)

- §3 / Eq. 1 → `physics/heli.ts` (done). §3.3 parameter ID → Phase 4 `learning/sysid`.
- §4 + Appendix A trajectory learning (EM, EKF smoother, dynamic-time-warping) → Phase 3
  `learning/trajectoryLearning`.
- §6 Gauss–Newton LQR / receding-horizon MPC, error state, α-homotopy → Phase 2 `control/`.

The full phased plan lives at `~/.claude/plans/please-make-an-implementation-sleepy-porcupine.md`.

## Verifying changes end-to-end

Beyond `npm test`, a headless smoke test pattern works for the browser app: launch system Chrome
via `puppeteer-core` with `--use-gl=angle --use-angle=swiftshader --enable-unsafe-swiftshader`
(plain WebGL fails headless), load the dev server, drive keys, assert the HUD text updates, and
screenshot. (WebGL needs the software-rasterizer flags above to init headless.)

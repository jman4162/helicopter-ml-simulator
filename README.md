# Helicopter Apprenticeship-Learning Simulator

An interactive, educational simulator built from Abbeel, Coates & Ng,
*[Autonomous Helicopter Aerobatics through Apprenticeship Learning](AbbeelCoatesNg_IJRR2010.pdf)*
(IJRR 2010). It aims to make the paper's three pillars — **physics**, **machine learning**, and
**control** — tangible and visual, all running live in the browser.

> **Status:** Complete (all five phases) — faithful rigid-body flight dynamics with a stylized 3D
> view and manual flight; an autonomous LQR autopilot (hover-hold, aerobatic maneuvers, wind
> recovery); apprenticeship trajectory learning (recover a clean intended path from noisy,
> time-warped demos); system identification (re-learn the dynamics from flight data); and an
> educational layer — a guided tour with equations, an autonomous airshow, and live telemetry.

## Quick start

```bash
npm install
npm run dev      # http://localhost:5173
```

### Fly it

| Key | Action |
| --- | --- |
| `W` / `S` | collective (climb / descend) |
| `↑` / `↓` | longitudinal cyclic (pitch forward / back) |
| `←` / `→` | lateral cyclic (roll left / right) |
| `A` / `D` | tail rotor (yaw left / right) |
| `R` | reset · `C` | cycle camera |

A connected gamepad maps to the two sticks (mode-2 RC layout).

### Let it fly itself (autopilot)

| Key | Action |
| --- | --- |
| `H` | hold a hover at the current position |
| `1` / `2` | forward flight / square pattern |
| `3` / `4` | in-place flip / loop |
| `G` | inject a wind gust (watch the LQR recover) |
| `L` | apprenticeship-learning demo (watch EM recover an intended path from noisy demos) |
| `I` | system-identification demo (fit the dynamics from flight data; true vs predicted) |
| `O` | autonomous airshow (chains maneuvers) |
| `T` | guided tour (`N`/`B` to step through the lessons) |
| `M` | back to manual (or just touch a flight key) |

The autopilot is a Gauss–Newton LQR (the paper's control approach): an infinite-horizon LQR holds
hover, and a time-varying LQR tracks aerobatic maneuvers around feasible reference trajectories.

## What's modeled

The flight dynamics are Eq. 1 of the paper: body-frame linear and angular accelerations with
gyroscopic coupling, gravity resolved into the body frame, and four controls (two cyclic, tail
rotor, collective). Integrated with RK4 over a unit quaternion attitude. The default coefficients
are plausible aerobatic-class values; a later phase fits them from flight data (system ID).

## Development

```bash
npm test         # Vitest unit + sanity tests
npm run typecheck
npm run build
```

See [`CLAUDE.md`](CLAUDE.md) for architecture, conventions, and the paper-section → code map.

## Roadmap

1. ✅ **Physics + manual flight** — rigid-body dynamics, 3D viz, HUD.
2. ✅ **Control** — Gauss–Newton LQR autopilot; hover-hold, aerobatic maneuvers, wind recovery.
3. ✅ **Apprenticeship learning** — infer an intended trajectory from noisy, time-warped demos
   (EM + dynamic time warping); the visual centerpiece.
4. ✅ **System identification** — fit the dynamics coefficients from logged flight data.
5. ✅ **Educational layer** — a guided tour with equations, an autonomous airshow, telemetry plot.

New here? Press **`T`** for the guided tour — it walks through all three pillars in order.

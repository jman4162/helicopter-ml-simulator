# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project status

This is a **greenfield repository** — as of this writing it contains no source code, build
configuration, tests, or README. The only file is a reference research paper. There are
therefore no build, lint, run, or test commands yet; establish them as the codebase is created
and record them here.

## Intent

Based on the directory name (`helicopter-ml-simulator`) and the included paper, the goal is to
build a machine-learning–driven helicopter flight simulator / controller, grounded in:

> Abbeel, Coates & Ng, *Autonomous Helicopter Aerobatics through Apprenticeship Learning*,
> International Journal of Robotics Research, 2010 (`AbbeelCoatesNg_IJRR2010.pdf`).

Key concepts from the paper relevant to implementation:

- **Apprenticeship learning** — learn the target trajectory and the dynamics model from
  (suboptimal, repeated) expert demonstrations rather than hand-specifying them.
- **Trajectory learning** — an EM algorithm (extended Kalman smoother in the E-step + dynamic
  programming for time-alignment) infers the unobserved intended target trajectory from multiple
  demonstrations.
- **Dynamics modeling** — a "rigid-body" state representation (position, velocity, orientation,
  angular rate, main-rotor speed); unknown parameters fit to flight data; note the paper's
  observation that the rigid-body model alone underfits aggressive aerobatics.
- **Control** — receding-horizon (model-predictive) variation of LQR for the non-linear system.

When implementing, confirm design decisions against the paper rather than assuming.

## Notes for future sessions

- This repo is **not** a git repository yet — initialize version control before substantial work.
- Update this file with real commands and architecture once the project structure exists; remove
  the "greenfield" framing at that point.

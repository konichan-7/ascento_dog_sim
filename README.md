# Ascento Dog Simulation

This repository is the start of a simulation and control stack for a four-wheeled robot using an Ascento-style one-DoF leg linkage.

The current milestone contains:

- an explicit linkage convention and project rules in `AGENTS.md`;
- analytical forward and inverse kinematics for one leg;
- a free-floating, contact-free MuJoCo model with a true closed-loop constraint;
- numerical tests that compare analytical points with MuJoCo sites.

## Quick start

```bash
uv sync --dev
uv run pytest
uv run verify-leg-kinematics
uv run view-single-leg
```

The viewer command requires a desktop OpenGL session. The test and verification commands run headlessly.

See `docs/leg_kinematics.md` for the coordinate convention, equations, assumptions, and validation criteria.

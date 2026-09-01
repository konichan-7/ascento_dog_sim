# Single-leg validation record

Date: 2026-09-01

## Configuration

- Python: 3.12 (managed by `uv`)
- MuJoCo: locked by `uv.lock`
- Analytical sweep: 101 evenly spaced hip angles from -65 degrees to -15 degrees
- MuJoCo root: free joint
- Gravity and contact: disabled for linkage-isolation testing

## Reproduction commands

```bash
uv sync --dev
uv run pytest -q
uv run verify-leg-kinematics --samples 101
```

## Results

```text
94 passed
samples: 101
hip range: [-65.000, -15.000] deg
wheel z range: [-0.433653, -0.111206] m
wheel x excursion: 1.352260e-02 m
max bar residual: 8.604228e-16 m
max IK(FK(q)) error: 4.440892e-15 rad
max MuJoCo point error: 2.581179e-11 m
max MuJoCo loop error: 5.117076e-11 m
```

The wheel center travels 0.322447 m vertically while its total x excursion is 0.013523 m over the provisional working interval. This confirms the intended near-vertical, rather than perfectly vertical, path.

The dynamic smoke test commands the hip actuator by +15 degrees for 1000 MuJoCo steps with the base still free. It checks finite state, actuator convergence, a transient loop error below 1 mm, and a final loop error below 1 micrometer.

## Interpretation and limitations

These results validate linkage geometry, branch selection, inverse/forward consistency, MJCF transform mapping, and equality-loop closure. They do not validate mass distribution, torque demand, spring behavior, actuator bandwidth, impact, tire contact, or whole-robot stability. The current visual geom density, link thicknesses, damping, armature, actuator gain, force limit, and wheel radius are explicit placeholders until CAD or measured parameters are supplied.

# Repository architecture

The project is divided into three product layers and one validation layer.

## Documentation

`docs/` records conventions, derivations, parameter provenance, assumptions, and validation results. A model is not considered stable if its conventions exist only in code.

## MuJoCo simulation

`models/mujoco/` contains MJCF assets. Python adapters in `src/ascento_dog/simulation/` map analytical state into MuJoCo joint state and expose named validation points.

The first model is deliberately contact-free and has a free root body. It isolates closed-loop linkage geometry from tire contact, suspension, body coupling, and balancing control.

## Control

`src/ascento_dog/control/` is reserved for control code. Control will be added only after the corresponding plant and state convention pass the verification gates in `AGENTS.md`. Controllers must import the kinematics package instead of reimplementing linkage equations.

Planned order:

1. single-leg position/force mapping;
2. single-leg actuator and spring identification;
3. four-leg chassis model;
4. wheel-ground contact and wheel-speed control;
5. posture/height control;
6. whole-body dynamic control.

## Validation

`tests/` checks analytical closure, inverse/forward round trips, MJCF compilation, equality closure, and point-by-point agreement between MuJoCo and the analytical oracle.

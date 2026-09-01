# Single-leg kinematics

## Scope

This document defines the geometry used by the first floating single-leg model. The model validates the one-degree-of-freedom linkage before ground contact, body coupling, springs, or control are introduced.

The attached Ascento paper motivates the near-linear wheel trajectory and the closed-loop leg design, but it does not publish the five dimensions used here. Those values come from the user's supplied table and schematic.

## Geometry and units

All equations use meters and radians.

| Segment | Points | Length (m) |
| --- | --- | ---: |
| `L1` | `DE` | 0.23500 |
| `L2` | `AD` | 0.23800 |
| `L3` | `BC` | 0.24400 |
| `L4` | `AB` | 0.10889 |
| `L23` | `CD` | 0.05700 |

`A` is the origin of the body-fixed sagittal frame. +x points forward and +z points upward. The fixed frame segment `AB` is at `gamma = 45 deg`. The hip input `q` is the counter-clockwise angle from +x to `AD` and is negative in the pictured working posture.

`C`, `D`, and `E` are collinear, with `D` between `C` and `E`. The selected assembly mode satisfies

```text
cross(D - B, C - D) > 0.
```

This selects the branch whose wheel center remains close to the vertical line through `A`. The provisional, nonsingular working interval is `-65 deg <= q <= -15 deg`.

## Forward kinematics

The fixed and driven points are

```text
A = [0, 0]
B = L4 [cos(gamma), sin(gamma)]
D = L2 [cos(q), sin(q)].
```

Point `C` is the selected intersection of two circles:

```text
||C - B|| = L3
||C - D|| = L23.
```

The wheel center follows from collinearity:

```text
E = D + (L1 / L23) (D - C).
```

The passive absolute link angles used to initialize MuJoCo are

```text
psi = atan2(Cz - Dz, Cx - Dx)
phi = atan2(Cz - Bz, Cx - Bx),
```

where `psi` is the `D -> C` angle and `phi` is the `B -> C` angle.

## Inverse kinematics

The linkage has one DoF, so its Cartesian workspace is a curve rather than an area. For a wheel target `E` on that curve, candidate points `D` are the intersections of

```text
||D - A|| = L2
||D - E|| = L1.
```

For every candidate,

```text
C = D + (L23 / L1) (D - E)
```

is reconstructed and checked against `||C - B|| = L3`, the working assembly sign, and the hip limits. The valid solution is `q = atan2(Dz, Dx)`. Off-curve Cartesian targets are rejected explicitly. Height-only IK is solved on the one-dimensional path with a bracketed bisection search.

## MuJoCo mapping

The MJCF uses a free root body and zero gravity, so the leg is floating and has no ground contact. Three hinge bodies form an open kinematic tree. A named `connect` equality constraint joins the two sites representing `C`, closing the four-bar linkage.

The MJCF reference pose is `q0 = -40 deg`. Its hinge coordinates are deviations from that reference:

```text
hip_drive = q - q0
inner_passive = (psi - q) - (psi0 - q0)
pin_passive = phi - phi0.
```

The density and shape parameters in the first MJCF are visualization-only placeholders. They must not be used as identified dynamics.

## Reproduction

```bash
uv sync --dev
uv run pytest
uv run verify-leg-kinematics --samples 101
```

The verification reports maximum bar-length residual, FK/IK hip-angle error, analytical-to-MuJoCo point error, and equality-loop closure error.

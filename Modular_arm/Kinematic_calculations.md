# Custom Robotic Arm - Kinematic Calculations

## Purpose

This file contains the engineering calculations derived from the requirements in
`Workspace_constraints`. Requirements and design decisions remain in that file; equations,
numerical evaluations, candidate comparisons and calculation results are recorded here.

## Calculation status

| Topic / item | Plan / execution description | Status |
|---|---|---|
| Kinematic convention | Define the joint coordinate frames and DH convention. | Complete - standard DH selected |
| Initial DH table | Build the symbolic table for J1-J5. | Complete for ideal intersecting-axis model |
| Forward kinematics | Derive the base-to-TCP transformation. | Complete for ideal intersecting-axis model |
| Critical-point inverse kinematics | Solve CP1 and CP2 for elbow-up and elbow-down configurations. | Preliminary planar calculation complete |
| CP1-to-CP2 continuity test | Test every 1 mm radial position from R = 200 mm to R = 500 mm at Z = 0 with a vertically downward tool. | Complete for joint limits; collision validation remains |
| R = 600 mm boundary test | Test a horizontal-forward tool at R = 600 mm and provisional Z = 180 mm. | Preliminary planar calculation complete |
| CP3 maximum-height test | Test R = 300 mm, Z = 650 mm with a horizontal-forward tool. | Preliminary planar calculation complete |
| Workspace validation | Sample both required tool orientations across the complete R-Z domain and apply commanded joint limits. | Complete at 1 mm resolution; collision validation remains |
| Link-length iteration | Compare feasible L1/L2 candidates and select a kinematic candidate. | Not started |
| Static torque | Calculate gravity loads using the selected geometry and mass model. | Not started |
| Dynamic torque | Add acceleration, inertia, friction and service margins. | Not started |

## Confirmed calculation inputs

| Input | Value | Source/status |
|---|---:|---|
| Arm joint sequence | Yaw-pitch-pitch-pitch-roll | Confirmed |
| J1/J2 axis relationship | Intersecting axes at the shoulder centre; zero intentional horizontal offset | Confirmed |
| J4/J5 axis relationship | Intersecting axes at a common wrist centre; zero intentional offset | Confirmed |
| Shoulder height, H | 180 mm | Confirmed |
| Tool offset, Lt | 100 mm to TCP | Confirmed |
| Candidate upper-arm length, L1 | 280 mm | Provisional |
| Candidate forearm length, L2 | 260 mm | Provisional |
| Required maximum radial TCP reach | 600 mm | Confirmed |
| Required vertical TCP range | 0-650 mm | Provisional |
| CP1 | R = 500 mm, Z = 0 mm | Confirmed coordinates |
| CP2 | R = 200 mm, Z = 0 mm | Confirmed coordinates |
| Object payload | 0.750 kg, excluding gripper/interface | Confirmed |
| Initial complete end-effector load | 1.250 kg | Provisional |

## Preliminary planar continuity calculation: CP2 to CP1

### Question

Determine whether confirming CP2 at R = 200 mm and CP1 at R = 500 mm also makes
every intermediate radial point reachable at Z = 0 mm with the gripper pointing vertically
downward.

### Model used

For a downward-pointing tool, the wrist centre is 100 mm above the TCP. Therefore:

```text
R_w = R_TCP
Z_w = Z_TCP + Lt = 100 mm
Delta Z from J2 = Z_w - H = 100 - 180 = -80 mm
```

The wrist distance from J2 is:

```text
d(R) = sqrt(R^2 + (-80)^2)
```

Across R = 200-500 mm:

```text
d_min = 215.41 mm
d_max = 506.36 mm
```

The two-link geometric reach interval is:

```text
|L1 - L2| <= d <= L1 + L2
20 mm <= d <= 540 mm
```

The complete requested radial interval is therefore geometrically inside the two-link
annulus.

For each radial sample, the elbow solutions were calculated using:

```text
cos(q3) = (d^2 - L1^2 - L2^2) / (2 L1 L2)

q3 = +/- acos(cos(q3))

q2 = atan2(Delta Z, R)
     - atan2(L2 sin(q3), L1 + L2 cos(q3))

q4 = phi - q2 - q3
```

where `phi = -90 degrees` represents the vertically downward tool direction.

### Numerical interval test

The interval was sampled at every integer radius from 200 mm through 500 mm, inclusive,
using the commanded limits for J2, J3 and J4.

| IK branch | J2 range | J3 range | J4 range | Valid samples |
|---|---:|---:|---:|---:|
| Negative-elbow branch | 10.47 to 40.01 deg | -133.16 to -40.69 deg | -59.78 to 3.27 deg | 301 / 301 |
| Positive-elbow branch | -83.50 to -28.65 deg | 40.69 to 133.16 deg | -139.67 to -102.04 deg | 170 / 301 |

The positive-elbow branch first becomes valid at approximately R = 331 mm because J4
exceeds its -130 degree commanded limit at smaller radii.

### Result

The current candidate geometry provides a continuous, joint-limit-valid negative-elbow
solution for every radial TCP position from R = 200 mm to R = 500 mm at Z = 0 mm with a
vertically downward gripper.

This is a kinematic joint-limit result, not yet a complete physical guarantee. Final access
also requires CAD collision checks, base keep-out checks, link thickness, gripper/payload
clearance, cable routing and singularity margin validation.

## Preliminary R = 600 mm horizontal boundary calculation

### Boundary definition

```text
TCP radial position: R = 600 mm
TCP vertical position: Z = 180 mm (provisional)
Tool orientation: horizontal and forward
Tool offset: Lt = 100 mm
```

Because the tool points horizontally forward, the wrist target is:

```text
R_w = 600 - 100 = 500 mm
Z_w = 180 mm
Delta Z from J2 = 180 - 180 = 0 mm
d = 500 mm
```

This is inside the two-link reach interval of 20-540 mm.

| IK branch | J2 | J3 | J4 | Joint-limit result |
|---|---:|---:|---:|---|
| Negative-elbow | 21.34 deg | -44.42 deg | 23.07 deg | Pass |
| Positive-elbow | -21.34 deg | 44.42 deg | -23.07 deg | Pass |

Both elbow branches reach this provisional boundary pose within the commanded J2-J4
limits. The configuration is also away from the straight-arm singularity because the elbow
magnitude is approximately 44.42 degrees rather than zero.

## Preliminary CP3 maximum-height calculation

### Boundary definition

```text
TCP radial position: R = 300 mm
TCP vertical position: Z = 650 mm
Tool orientation: horizontal and forward
Tool offset: Lt = 100 mm
```

The horizontal tool offset places the wrist target at:

```text
R_w = 300 - 100 = 200 mm
Z_w = 650 mm
Delta Z from J2 = 650 - 180 = 470 mm
d = sqrt(200^2 + 470^2) = 510.78 mm
```

The required wrist distance is inside the two-link reach interval of 20-540 mm.

| IK branch | J2 | J3 | J4 | Joint-limit result |
|---|---:|---:|---:|---|
| Negative-elbow | 85.17 deg | -37.89 deg | -47.27 deg | Pass |
| Positive-elbow | 48.73 deg | 37.89 deg | -86.62 deg | Pass |

Both elbow branches reach CP3 within the commanded J2-J4 limits. The negative-elbow
branch places J2 closer to its +110 degree upper limit, while the positive-elbow branch has
more shoulder-angle margin.

## Standard DH model and forward kinematics

### Convention and frame assumptions

This model uses standard Denavit-Hartenberg transformations in the exact order:

```text
A_i = Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)
```

The model assumes:

- J1 and J2 intersect at the shoulder centre, H = 180 mm above the bench.
- J2, J3 and J4 are parallel pitch axes.
- J4 and J5 intersect at a common wrist centre.
- J5 is the tool-roll axis and points along the tool centreline.
- The TCP is Lt = 100 mm along the J5/tool axis.
- G1 operates the gripper and is not part of the serial arm-pose transformation T_0^5.

### Initial five-row DH table

The table follows the convention written above; it does not use the alternative
`a_(i-1), alpha_(i-1)` column labelling.

| i | theta_i | d_i | a_i | alpha_i | Joint/function |
|---:|---:|---:|---:|---:|---|
| 1 | q1 | H | 0 | +90 deg | Base yaw |
| 2 | q2 | 0 | L1 | 0 deg | Shoulder pitch |
| 3 | q3 | 0 | L2 | 0 deg | Elbow pitch |
| 4 | q4 + 90 deg | 0 | 0 | +90 deg | Wrist pitch and frame alignment to roll axis |
| 5 | q5 | Lt | 0 | 0 deg | Wrist roll and TCP offset |

The +90 degree term in row 4 is a coordinate-frame offset. It does not mean that the
physical wrist is commanded to q4 = 90 degrees when the wrist is straight.

### Compact position equations

Define:

```text
q23  = q2 + q3
phi  = q2 + q3 + q4

rho = L1 cos(q2) + L2 cos(q23) + Lt cos(phi)
```

Then the TCP position is:

```text
x = rho cos(q1)
y = rho sin(q1)
z = H + L1 sin(q2) + L2 sin(q23) + Lt sin(phi)
```

J5 does not change TCP position because it rolls the tool about its own centreline.

### Symbolic base-to-TCP transformation

Use the abbreviations:

```text
c1 = cos(q1)       s1 = sin(q1)
c5 = cos(q5)       s5 = sin(q5)
cp = cos(phi)      sp = sin(phi)
```

The complete transformation is:

```text
T_0^5 =

[ -c1*sp*c5 + s1*s5,   c1*sp*s5 + s1*c5,   c1*cp,   rho*c1 ]
[ -s1*sp*c5 - c1*s5,   s1*sp*s5 - c1*c5,   s1*cp,   rho*s1 ]
[           cp*c5,              -cp*s5,        sp,         z ]
[               0,                    0,         0,         1 ]
```

The third rotation-matrix column is the physical tool-axis direction:

```text
tool_axis = [cos(q1) cos(phi), sin(q1) cos(phi), sin(phi)]^T
```

Therefore:

- `phi = 0 deg` gives a horizontal-forward tool.
- `phi = -90 deg` gives a vertically downward tool.
- q5 changes tool roll without changing the tool-axis direction or TCP position.

### Checks

At the zero pose `q1=q2=q3=q4=q5=0`:

```text
TCP = [L1 + L2 + Lt, 0, H]
    = [640, 0, 180] mm
```

A numerical cross-check was also performed at:

```text
[q1, q2, q3, q4, q5] = [30, 20, -50, 40, 25] deg
```

Both the DH matrix product and independent geometric equations returned:

```text
TCP = [508.150204, 293.380657, 163.130458] mm
```

The maximum absolute position difference was approximately 1.14e-13 mm, which is
numerical round-off. The DH table and compact position equations are therefore internally
consistent.

## Reachable workspace maps

### Method

Two orientation-constrained radial-height maps were calculated:

```text
R domain: 200-600 mm
Z domain: 0-650 mm
Initial visual/data grid: 5 mm
Refined boundary grid: 1 mm
Downward orientation: phi = -90 deg
Horizontal-forward orientation: phi = 0 deg
```

At every point, both planar IK branches were calculated and tested against:

```text
J2: -90 to +110 deg
J3: -140 to +140 deg
J4: -130 to +130 deg
```

The state categories are:

| Code/category | Meaning |
|---|---|
| Geometrically unreachable | Wrist target lies outside the two-link annulus. |
| Joint-limit excluded | Link geometry reaches the target, but neither complete J2/J3/J4 solution passes. |
| Negative-elbow only | Only the negative-q3 branch passes all commanded limits. |
| Positive-elbow only | Only the positive-q3 branch passes all commanded limits. |
| Both branches | Both complete IK branches pass all commanded limits. |

The 1 mm grid contains 401 x 651 = 261,051 points per orientation.

### Map 1 - vertically downward tool

For phi = -90 degrees:

| Refined classification | Point count |
|---|---:|
| Geometrically unreachable | 110,023 |
| Joint-limit excluded | 4,585 |
| Negative-elbow only | 120,128 |
| Positive-elbow only | 0 |
| Both branches | 26,315 |
| Total joint-limit reachable | 146,443 (56.10 percent) |

Important refined boundaries:

| Height | Reachable radial interval |
|---:|---:|
| Z = 0 mm | R = 200-534 mm |
| Z = 180 mm | R = 200-530 mm |
| Z = 650 mm | No reachable points |

Across the complete analysed radial domain, the downward map reaches a maximum height of
Z = 527 mm. Some downward poses are reachable as far as R = 540 mm at other heights, but
the bench-level limit is R = 534 mm.

Critical-point results:

| Point | Classification | Result |
|---|---|---|
| CP1: R = 500 mm, Z = 0 mm | Both elbow branches | Pass |
| CP2: R = 200 mm, Z = 0 mm | Negative-elbow branch only | Pass |

Files:

- Visual: `workspace_map_downward.png`, `analysis_artifacts/workspace_map_downward.svg`
- Coarse data: `analysis_artifacts/workspace_map_downward_5mm.csv`
- Refined envelope: `analysis_artifacts/workspace_envelope_downward_1mm.csv`

### Map 2 - horizontal-forward tool

For phi = 0 degrees:

| Refined classification | Point count |
|---|---:|
| Geometrically unreachable | 24,292 |
| Joint-limit excluded | 20,669 |
| Negative-elbow only | 7,638 |
| Positive-elbow only | 12,873 |
| Both branches | 195,579 |
| Total joint-limit reachable | 216,090 (82.78 percent) |

Important refined boundaries:

| Height | Reachable radial interval |
|---:|---:|
| Z = 0 mm | R = 213-600 mm |
| Z = 180 mm | R = 286-600 mm |
| Z = 650 mm | R = 200-365 mm |

The horizontal map contains reachable points across the complete analysed R = 200-600 mm
and Z = 0-650 mm bounds, but not every R-Z combination is reachable.

Critical-point results:

| Point | Classification | Result |
|---|---|---|
| CP3: R = 300 mm, Z = 650 mm | Both elbow branches | Pass |
| R600 boundary: R = 600 mm, Z = 180 mm | Both elbow branches | Pass |

Files:

- Visual: `workspace_map_horizontal.png`, `analysis_artifacts/workspace_map_horizontal.svg`
- Coarse data: `analysis_artifacts/workspace_map_horizontal_5mm.csv`
- Refined envelope: `analysis_artifacts/workspace_envelope_horizontal_1mm.csv`

### Interpretation and limitations

These maps verify planar kinematics and commanded joint limits. J1 sweeps each valid radial
point around the base through its +/-150 degree range, while J5 changes roll without moving
the TCP. The percentages refer to points in the sampled R-Z rectangle and are not three-
dimensional workspace-volume percentages.

The maps do not yet account for physical link thickness, base keep-out clearance,
self-collision, gripper/payload dimensions, cable routing or external obstacles. Those checks
require the mechanical CAD geometry.

The reproducible calculation is in `analysis_artifacts/workspace_map_analysis.ps1`;
aggregate machine-readable results are in
`analysis_artifacts/workspace_map_summary.json`.

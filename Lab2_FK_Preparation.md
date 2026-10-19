# CEG4158 Lab 2: Forward Kinematics Preparation

This is a working table transcribed from the manufacturer's Classical DH table (Kinova Gen3 Lite User Guide, Table 50, printed page 135) into the Lab 2 guide's column order. The Lab 2 guide still asks you to inspect the frame axes and validate the model against `Gen3Lite fk` and `j.EndEffectorPose`. Confirm joint-positive directions and the base/tool frame placement in the simulator before treating the model as validated.

## Frame sketch notes

Annotate the robot's two home views (Lab 2 guide, Figure 2) with frames `R0` through `R6`:

- Put each `z(i-1)` axis on the physical rotation axis of revolute joint `i`.
- For each adjacent pair of axes, put `x(i-1)` along their common normal, directed from the previous joint axis toward the next. If the two axes intersect, choose the perpendicular consistent with the right-hand frame and record the choice.
- Complete each frame with `y = z cross x` so it is right-handed.
- Mark the positive rotation arrow on every joint only after checking the sign in CoppeliaSim. The twist signs and angle offsets below are provisional until that check.
- `R0` and `R6` must correspond to the guide's base and tool reference frames. Do not add the 52 mm scene plate height when comparing base-relative poses; `EndEffectorPose` is reported relative to the robot base.

For the candidate table below, the joint-axis sketch at the all-zero commanded pose can be placed using these DH-frame origins and axis directions, expressed in `R0` coordinates. `z(i-1)` is the rotation axis for joint `i`; `x(i-1)` is the corresponding DH X axis. Use these to mark a *candidate* onto Figure 2, then check the arrows against the actual simulator joint motions.

| Joint i | Origin of frame R(i-1), m | Candidate +Z axis z(i-1) in R0 | Candidate +X axis x(i-1) in R0 |
|---:|---:|---:|---:|
| 1 | (0, 0, 0) | (0, 0, 1) | (1, 0, 0) |
| 2 | (0, 0, 0.2433) | (0, -1, 0) | (1, 0, 0) |
| 3 | (0, -0.030, 0.5233) | (0, 1, 0) | (0, 0, 1) |
| 4 | (0, -0.010, 0.5233) | (0, 0, 1) | (1, 0, 0) |
| 5 | (0, -0.010, 0.7683) | (1, 0, 0) | (0, 1, 0) |
| 6 | (0.057, -0.010, 0.7683) | (0, 0, 1) | (0, -1, 0) |

These coordinates are mathematical consequences of the proposed DH table, not independent measurements of the physical robot. If the observed positive rotation is opposite to an arrow, revise that joint's axis/frame convention and then re-derive its DH parameters.

## Classical DH table in the Lab 2 guide's column order

The convention is the classical DH matrix implemented in `dhTransform.m`. `q(i)` is the commanded joint angle in radians. The manufacturer gives some offsets as sums of dimensions in millimetres; these are converted to metres below.

| i | l(i) = a(i) (m) | d(i) (m) | alpha(i) | theta(i) |
|---:|---:|---:|---:|---|
| 1 | 0 | (128.3 + 115.0) mm = 0.2433 | pi/2 | q1 |
| 2 | 280.0 mm = 0.280 | 30.0 mm = 0.030 | pi | q2 + pi/2 |
| 3 | 0 | 20.0 mm = 0.020 | pi/2 | q3 + pi/2 |
| 4 | 0 | (140.0 + 105.0) mm = 0.245 | pi/2 | q4 + pi/2 |
| 5 | 0 | (28.5 + 28.5) mm = 0.057 | pi/2 | q5 + pi |
| 6 | 0 | (105.0 + 130.0) mm = 0.235 | 0 | q6 + pi/2 |

Define `phi = [q1, q2+pi/2, q3+pi/2, q4+pi/2, q5+pi, q6+pi/2]`, and `ci = cos(phi(i))`, `si = sin(phi(i))`. Substitution into the classical DH matrix gives:

```text
A1 = [ c1  0   s1   0     ]
     [ s1  0  -c1   0     ]
     [  0  1    0   0.2433]
     [  0  0    0   1     ]

A2 = [ c2  s2   0   0.280*c2]
     [ s2 -c2   0   0.280*s2]
     [  0   0  -1   0.030   ]
     [  0   0   0   1       ]

A3 = [ c3  0   s3   0     ]
     [ s3  0  -c3   0     ]
     [  0  1    0   0.020 ]
     [  0  0    0   1     ]

A4 = [ c4  0   s4   0     ]
     [ s4  0  -c4   0     ]
     [  0  1    0   0.245 ]
     [  0  0    0   1     ]

A5 = [ c5  0   s5   0     ]
     [ s5  0  -c5   0     ]
     [  0  1    0   0.057 ]
     [  0  0    0   1     ]

A6 = [ c6 -s6   0   0     ]
     [ s6  c6   0   0     ]
     [  0   0   1   0.235 ]
     [  0   0   0   1     ]

T_base_tool = A1*A2*A3*A4*A5*A6
```

`gen3liteFK.m` computes this product and also returns the intermediate transforms. It is a candidate implementation, not a simulator-validated final model.

## Simulator pose conversion

The Lab 2 guide defines `EndEffectorPose = [x y z alpha beta gamma]`, with position in metres, angles in radians, and:

```text
T_sim = Trans(x,y,z) * RotX(alpha) * RotY(beta) * RotZ(gamma)
```

`gen3litePoseToT.m` implements that conversion. Compare its result with `gen3liteFK(q)` using the same measured joint values. `lab2_validate_fk.m` runs a set of guide-recommended configurations and reports position and orientation errors.

## Files

- `gen3liteFK.m`: candidate FK and intermediate transforms.
- `dhTransform.m`: classical DH matrix.
- `gen3litePoseToT.m`: simulator pose vector to homogeneous matrix.
- `lab2_validate_fk.m`: scene setup, configuration commands, and comparison output.

The validation script uses a manually opened `Gen3Lite fk` scene by default; set `loadSceneFromMatlab = true` at its top to load it from the configured simAPI path instead. It stops the simulation after the comparison and leaves the scene visible by default. Review the reported errors and update the axes/offsets/table if they are not close to zero. MATLAB and CoppeliaSim were unavailable here, so the comparison has not been run.

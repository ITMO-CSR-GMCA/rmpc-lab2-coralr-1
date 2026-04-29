[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/kdWjk76G)

# Lab #2 — Manipulator Kinematics and Trajectory Planning

Course: Robot Modelling, Planning and Control (RMPC), ITMO University

## Overview

This lab implements the full kinematics-and-planning pipeline for the
**Unimation Puma 560** 6-DOF serial manipulator using the
[Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python).
Each step from the assignment brief is realised in a single Jupyter notebook
([lab2.ipynb](lab2.ipynb)) with detailed inline comments and a final
conclusions section.

## Assignment checklist

| # | Task | Notebook section |
|---|------|------------------|
| 1 | Load the manipulator model from the toolbox | §1 |
| 2 | Fill in all robot parameters (from Lab №1) | §2 |
| 3 | Set an arbitrary initial configuration | §3 |
| 4 | Solve the forward kinematics problem | §4 |
| 5 | Build the workspace under joint constraints | §5 |
| 6 | Pick an end point and solve inverse kinematics | §6 |
| 7 | Plan trajectories with at least three methods | §7 |
| 8 | Plot joint positions, velocities, accelerations | §8 |
| 9 | Report in `.ipynb` with detailed comments | whole notebook |
| 10 | Draw conclusions | §9 |

## Methods used

* **Forward kinematics** — `robot.fkine(q)` from standard DH parameters.
* **Workspace generation** — uniform sweep of the first three joint
  variables over their `qlim` ranges (n³ = 18³ = 5832 samples).
* **Inverse kinematics** — Levenberg–Marquardt solver (`ikine_LM`) seeded
  from the initial configuration. Position error after back-substitution is
  on the order of 10⁻⁹ m.
* **Trajectory planning** — three methods are compared:
  1. `rtb.jtraj` — quintic polynomial joint-space trajectory.
  2. `rtb.mtraj(rtb.trapezoidal, ...)` — Linear Segment with Parabolic
     Blends (LSPB / trapezoidal velocity profile).
  3. `rtb.mtraj(rtb.quintic, ...)` — quintic polynomial profile via `mtraj`.

## Key numerical results

* Initial configuration:
  `q_start = [π/4, −π/3, −π/4, π/3, −π/3, π/4]` rad
  → end-effector at `(0.550, 0.338, 0.167)` m.
* Target end-effector position: `(−0.500, 0.100, 0.200)` m.
* IK convergence: 14 iterations, residual position error ≈ 3.8·10⁻⁹ m.
* Trajectory peak magnitudes (5-second move, N = 100 samples):
  | Method | max ǀq̇ǀ, rad/s | max ǀq̈ǀ, rad/s² |
  | --- | --- | --- |
  | jtraj (quintic) | 1.570 | 0.967 |
  | mtraj / trapezoidal | 1.257 | 0.754 |
  | mtraj / quintic | 1.570 | 0.967 |

## Files

* [lab2.ipynb](lab2.ipynb) — completed lab notebook with code, plots and
  conclusions.
* [lab2-template.ipynb](lab2-template.ipynb) — original template provided
  with the assignment.
* [assignment-lab2](assignment-lab2) — assignment brief.
* `build_notebook.py` — helper script that programmatically rebuilds
  `lab2.ipynb` from source if needed.

## Reproducing the results

```bash
pip install "numpy<2" roboticstoolbox-python spatialmath-python matplotlib jupyter
jupyter nbconvert --to notebook --execute lab2.ipynb --output lab2.ipynb
```

Tested with Python 3.11, `roboticstoolbox-python` 1.1.1, `numpy` 1.26.4,
`spatialmath-python` 1.1.15.

## Conclusions (short)

All three planners drive the manipulator from `q_start` to `q_end` over
`T = 5 s`, but their dynamic behaviour differs:

* **Trapezoidal (LSPB)** minimises peak joint velocity and acceleration but
  produces piecewise-constant acceleration — i.e. unbounded jerk at the
  blend points.
* **Quintic / jtraj** yield smooth (continuously differentiable) position,
  velocity *and* acceleration profiles; peak magnitudes are higher than
  LSPB for the same motion duration, but the bounded jerk makes the
  motion much friendlier to real actuators.
* In practice LSPB is preferred when peak torque is the binding
  constraint, and quintic profiles are preferred for precision tasks
  where smoothness matters more than peak velocity.

The detailed discussion is in the *Выводы* section at the end of
[lab2.ipynb](lab2.ipynb).

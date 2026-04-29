#!/usr/bin/env python3
"""test_rob.py — Run your robot solution and view the results.

Usage (from lab4_student/):
    python3 test_rob.py 1      # trajectory 1
    python3 test_rob.py 2      # trajectory 2
    python3 test_rob.py 3      # trajectory 3

The script runs your RobStudent implementation on one of the three graded
trajectories and shows the diagnostic plots.  No pass/fail checks are
performed — use this to visually debug your trajectory and controller.
"""

import sys
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

from RobStudent import RobStudent

# ── Test trajectories ─────────────────────────────────────────────────────────
TRAJECTORIES = {
    1: dict(
        ball_start = np.array([783.288, -783.288, 224.3548]),
        ball_end   = np.array([923.636,  923.636, 1350.0]),
        notes      = "Diagonal cross-quadrant, low → high",
    ),
    2: dict(
        ball_start = np.array([500.0, -300.0,  500.0]),
        ball_end   = np.array([400.0,  400.0, 1100.0]),
        notes      = "θ1 sweeps −0.54 → +0.79 rad, moderate height gain",
    ),
    3: dict(
        ball_start = np.array([ 600.0, 200.0,  300.0]),
        ball_end   = np.array([-500.0, 600.0, 1200.0]),
        notes      = "θ1 sweeps +0.32 → +2.27 rad, large joint-1 rotation",
    ),
}

# Home configuration matching the autograder (joint 2 = −20°, joint 3 = +20°)
HOME_THETA = np.array([0.0, np.radians(-20), np.radians(20)])

# ── Parse argument ────────────────────────────────────────────────────────────
if len(sys.argv) != 2 or sys.argv[1] not in ('1', '2', '3'):
    print("Usage: python3 test_rob.py <1|2|3>")
    sys.exit(1)

traj_id = int(sys.argv[1])
cfg     = TRAJECTORIES[traj_id]

# ── Build robot and waypoints ─────────────────────────────────────────────────
rob = RobStudent(drawing_enabled=True)

rob.calculate_fk(HOME_THETA)
home_pos  = rob.ee_pos.copy()
waypoints = np.array([home_pos, cfg['ball_start'], cfg['ball_end'], home_pos])

print("=" * 60)
print(f"Trajectory {traj_id}: {cfg['notes']}")
print("Waypoints (mm):")
labels = ["home (start)", "ball start  ", "ball end    ", "home (end)  "]
for label, wp in zip(labels, waypoints):
    print(f"  {label}: {np.round(wp, 2)}")
print("=" * 60)

# ── Run simulation ────────────────────────────────────────────────────────────
print("Running simulation …")
rob.simulate_rob(waypoints)
print(f"Done — {len(rob._data)} timesteps recorded.\n")

# ── Plot results ──────────────────────────────────────────────────────────────
rob._data.plot_all(waypoints=waypoints, label=f"Trajectory {traj_id}")
plt.show()

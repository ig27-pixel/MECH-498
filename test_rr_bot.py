#!/usr/bin/env python3
"""Quick smoke-test for RRSolution.

Runs both the free (uncontrolled) and PD-controlled simulations and
prints a summary of the energy history so you can verify the dynamics
and control are behaving correctly.

Run from lab4_test/source/:
    python3 test_rr_solution.py
"""

from RRBot import RRBot
import matplotlib.pyplot as plt


if __name__ == "__main__":

  # ------------------------------------------------------------------
  # Case 1: Free motion (tau = 0) — energy should be approximately
  # conserved (small drift is expected from trapezoidal integration).
  # ------------------------------------------------------------------
  print("\n=== Case 1: Uncontrolled (free motion) ===")
  rob_free = RRBot(drawing_enabled=True)
  rob_free.simulate_rr(controlled=False)
  rob_free.plot_energy_summary("free")
  rob_free.plot_joint_states("free")

  # ------------------------------------------------------------------
  # Case 2: Stepoint 1 Under damped PD control — robot should converge to
  # desired positions with oscillation.
  # ------------------------------------------------------------------
  print("\n=== Case 2: Set Point 1 Controlled — under damped ===")
  rob_ud = RRBot(drawing_enabled=False)
  rob_ud.test_setpoint_1_underdamped()
  rob_ud.plot_joint_states("SP 1 underdamped")

  # ------------------------------------------------------------------
  # Case 3: Setpoint 1 Criticall damped PD control — robot should  not 
  # oscillate before settling.
  # ------------------------------------------------------------------
  print("\n=== Case 3: Controlled — underdamped ===")
  rob_cd = RRBot(drawing_enabled=False)
  rob_cd.test_setpoint_1_critically_damped()
  rob_cd.plot_joint_states("SP 1 critically damped")

	# ------------------------------------------------------------------
  # Case 4: Setpoint 2 Criticall damped PD control — robot should  not 
  # oscillate before settling.
  # ------------------------------------------------------------------
  print("\n=== Case 4: Controlled — critically damped ===")
  rob_sp2 = RRBot(drawing_enabled=False)
  rob_sp2.test_setpoint_2_critically_damped()
  rob_sp2.plot_joint_states("SP 2 critically damped")

  plt.show()

  print("\nDone.")

import math
import numpy as np
from typing import Tuple, List
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from copy import deepcopy

from robot_components import Link, Joint
import general_utility as general

TAU_MAX = 100.0  # N·m — maximum allowable torque per joint


def trapazoid_calc(prev_value: float, prev_der: float, new_der: float,
                   timestep: float) -> float:
  """Forward-propagate one step using the trapezoidal rule.

  Args:
      prev_value (float): Previous value (position or velocity).
      prev_der (float): Previous derivative (velocity or acceleration).
      new_der (float): New derivative (velocity or acceleration).
      timestep (float): Integration timestep in seconds.

  Returns:
      float: New integrated value.
  """
  return prev_value + 0.5 * (prev_der + new_der) * timestep


class Workspace(object):
  """Axis-aligned bounding box describing the robot workspace."""

  def __init__(self, x_min: float, x_max: float, y_min: float, y_max: float,
               z_min: float, z_max: float):
    self.x_min = x_min
    self.x_max = x_max
    self.y_min = y_min
    self.y_max = y_max
    self.z_min = z_min
    self.z_max = z_max


class JointState(object):
  """Holds position, velocity, and acceleration for a single joint."""

  @property
  def pos(self):
    return self._pos

  @pos.setter
  def pos(self, value):
    self._pos = value

  @property
  def vel(self):
    return self._vel

  @vel.setter
  def vel(self, value):
    self._vel = value

  @property
  def accel(self):
    return self._accel

  @accel.setter
  def accel(self, value):
    self._accel = value

  def __init__(self,
               position: float = 0.0,
               velocity: float = 0.0,
               acceleration: float = 0.0) -> None:
    self._pos = position
    self._vel = velocity
    self._accel = acceleration

  def __repr__(self):
    return "pos = {}\nvel = {}\naccel = {}".format(self.pos, self.vel,
                                                   self.accel)


class RRBot(object):
  """2-DOF planar RR robot for dynamics simulation and PD control.

  Students must:
    1. Fill in the physical parameters in __init__ (marked TODO).
    2. Implement update_dynamics(), calculate_energy(), and simulate_rr().
    3. Implement control_underdamped() and control_critically_damped()
       with gains of their choosing.

  Provided helper methods (do not modify):
    - set_desired_positions()
    - calculate_fk(), draw_rr()
    - test_setpoint_1_underdamped(), test_setpoint_1_critically_damped(),
      test_setpoint_2_critically_damped()
    - plot_energy_summary(), plot_joint_states()
  """

  @property
  def joints(self) -> List[Joint]:
    return [self._joint_1, self._joint_2, self._joint_ee]

  @property
  def ee_frame(self) -> np.ndarray:
    """End-effector frame as the product of joint DH transforms."""
    return self._joint_1.dh_transform @ self._joint_2.dh_transform

  def __init__(self, drawing_enabled: bool = True) -> None:
    """Initialize the RR robot.

    Args:
        drawing_enabled (bool): If True, create a Matplotlib figure and
                                draw the robot during simulation.
    """
    self._drawing_enabled = drawing_enabled
    self._drawn_once = False
    self.colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

    # PD control gains — set by control_underdamped() / control_critically_damped()
    self._kp = None
    self._kv = None

    # Simulation parameters (do not change)
    self._dt = 0.005    # s
    self._duration = 5.0  # s

    # Desired joint positions used by PD control set by set_desired_positions()
    self._joint_1_des = 0.0
    self._joint_2_des = 0.0

    
    # Dynamics matrices — populated by update_dynamics()
    ## NOTE -- you must populate these in simulate_rr() for the autograder to work.
    self.M = None
    self.C = None
    self.G = None

    # ------------------------------------------------------------------ #
    # Values to populate during dynamics for plotting                    #
    # ------------------------------------------------------------------ #
    # Energy history — populated by simulate_rr()
    ## NOTE -- you must populate these in simulate_rr() for the autograder to work.
    self.KE_data = []
    self.PE_data = []
    self.E_total_data = []

    # Joint state history — populated by simulate_rr()
    ## NOTE -- you must populate these in simulate_rr() for the autograder to work.
    self.joint_1_data = []  # list of JointState per timestep
    self.joint_2_data = []  # list of JointState per timestep

    # Torque history — list of (2,1) arrays, one per timestep
    ## NOTE -- you must populate this in simulate_rr() for the autograder to work.
    self.tau_data = []

    # Physical parameters (lab spec) — must be set before Workspace is created
    self._m1  = 1.0    # kg  — point mass at end of link 1
    self._m2  = 5.0    # kg  — point mass at end of link 2
    self._mr1 = 2.3    # kg  — distributed mass of link 1 rod
    self._mr2 = 2.3    # kg  — distributed mass of link 2 rod
    self._l_1 = 1.0    # m   — length of link 1
    self._l_2 = 1.41   # m   — length of link 2
    self._g   = 9.81   # m/s²

    # Composite masses
    self._M1 = self._m1 + self._mr1
    self._M2 = self._m2 + self._mr2

    # Centers of mass (distance from proximal joint along link)
    self._Lc1 = (self._m1 + 0.5 * self._mr1) * self._l_1 / self._M1
    self._Lc2 = (self._m2 + 0.5 * self._mr2) * self._l_2 / self._M2

    # Moments of inertia about CoM (lab spec formula)
    self._I1 = (1.0/12.0) * self._mr1 * self._l_1**2 + self._m1 * (self._l_1/2)**2
    self._I2 = (1.0/12.0) * self._mr2 * self._l_2**2 + self._m2 * (self._l_2/2)**2

    self.workspace = Workspace(-(self._l_1 + self._l_2),
                               (self._l_1 + self._l_2),
                               -(self._l_1 + self._l_2),
                               (self._l_1 + self._l_2), -self._l_2,
                               self._l_2)

    self.fig = None
    self.ax = None
    if self._drawing_enabled:
      self._create_plot()

    self._joint_1 = None
    self._joint_2 = None
    self._joint_ee = None
    self._setup_joints()

    if self._drawing_enabled:
      self._links = [Link(self.ax, self.colors[i]) for i in range(3)]
      self._base_frame = Joint(self.ax)
      self._base_frame.set_final_transform(np.eye(4))
    else:
      self._links = None
      self._base_frame = None

  # ------------------------------------------------------------------ #
  # Provided — do not modify                                             #
  # ------------------------------------------------------------------ #

  def _create_plot(self) -> None:
    """Initialize the Matplotlib 3-D figure."""
    self.fig = plt.figure(figsize=(8, 8), facecolor='w')
    self.ax = self.fig.add_subplot(111, projection='3d')
    plt.xlim([self.workspace.x_min, self.workspace.x_max])
    plt.ylim([self.workspace.y_min, self.workspace.y_max])
    self.ax.set_zlim([self.workspace.z_min, self.workspace.z_max])
    self.ax.set_xlabel('X (m)', fontsize=16)
    self.ax.set_ylabel('Y (m)', fontsize=16)
    self.ax.set_zlabel('Z (m)', fontsize=16)
    plt.grid(True)
    self.ax.set_autoscale_on(False)

  def _setup_joints(self) -> None:
    """Instantiate and configure the three joints (J1, J2, EE)."""
    ax = self.ax  # None when drawing is disabled

    self._joint_1 = Joint(ax, drawing_enabled=self._drawing_enabled)
    self._joint_1.set_joint_limits(-math.inf, math.inf)
    self._joint_1.set_dh_value_a(0)
    self._joint_1.set_dh_value_alpha(0)
    self._joint_1.set_dh_value_d(0)
    self._joint_1.set_theta(0)

    self._joint_2 = Joint(ax, drawing_enabled=self._drawing_enabled)
    self._joint_2.set_joint_limits(-math.inf, math.inf)
    self._joint_2.set_dh_value_a(self._l_1)
    self._joint_2.set_dh_value_alpha(math.pi / 2)
    self._joint_2.set_dh_value_d(0)
    self._joint_2.set_theta(0)

    self._joint_ee = Joint(ax, drawing_enabled=self._drawing_enabled)
    self._joint_ee.set_joint_limits(-math.inf, math.inf)
    self._joint_ee.set_dh_value_a(self._l_2)
    self._joint_ee.set_dh_value_alpha(0)
    self._joint_ee.set_dh_value_d(0)
    self._joint_ee.set_theta(0)

  def calculate_fk(self, joint_angles: np.ndarray) -> None:
    """Update joint DH transforms for the given joint angles.

    Args:
        joint_angles (np.ndarray): (2,) array [theta_1, theta_2] in radians.
    """
    general.check_proper_numpy_format(joint_angles, (2,))
    self._joint_1.set_theta(joint_angles[0])
    self._joint_2.set_theta(joint_angles[1])

  def set_desired_positions(self, joint_1_des: float,
                            joint_2_des: float) -> None:
    """Set the target joint positions used by PD control.

    Args:
        joint_1_des (float): Desired angle for joint 1 in radians.
        joint_2_des (float): Desired angle for joint 2 in radians.
    """
    self._joint_1_des = joint_1_des
    self._joint_2_des = joint_2_des

  def clear_history(self) -> None:
    """Clear any existing data from previous runs."""
    self.KE_data = []
    self.PE_data = []
    self.E_total_data = []
    self.joint_1_data = []
    self.joint_2_data = []
    self.tau_data = []

  # ------------------------------------------------------------------ #
  # TODO: Implement the methods below                                    #
  # ------------------------------------------------------------------ #

  def update_dynamics(self, js1: JointState, js2: JointState) -> None:
    """Compute and store the M, C, G dynamics matrices.

    After this method returns, the following attributes must be set:
      self.M  — (2, 2) numpy array — mass/inertia matrix
      self.C  — (2, 1) numpy array — Coriolis/centrifugal vector
      self.G  — (2, 1) numpy array — gravity vector

    Args:
        js1 (JointState): State of joint 1 — use js1.pos (θ1) and js1.vel (θ̇1).
        js2 (JointState): State of joint 2 — use js2.pos (θ2) and js2.vel (θ̇2).
    """
    theta2     = js2.pos
    theta1_dot = js1.vel
    theta2_dot = js2.vel

    c2 = np.cos(theta2)
    s2 = np.sin(theta2)

    # Frequently used intermediate value
    r = self._l_1 + self._Lc2 * c2   # (L1 + Lc2*cos(θ2))

    # Mass/inertia matrix M (2x2)
    m11 = self._M1 * self._Lc1 ** 2 + self._M2 * r ** 2 + self._I1 + self._I2
    m22 = self._M2 * self._Lc2 ** 2 + self._I2

    self.M = np.array([[m11, 0.0],
                       [0.0, m22]])

    # Coriolis / centrifugal vector C (2x1)
    c1_val = -2.0 * self._M2 * self._Lc2 * s2 * r * theta1_dot * theta2_dot
    c2_val =        self._M2 * self._Lc2 * s2 * r * theta1_dot ** 2

    self.C = np.array([[c1_val],
                       [c2_val]])

    # Gravity vector G (2x1)
    self.G = np.array([[0.0],
                       [self._M2 * self._g * self._Lc2 * c2]])

  def calculate_energy(self, js1: JointState,
                       js2: JointState) -> Tuple[float, float, float]:
    """Calculate mechanical energy at the given joint states.

    Tip: Call update_dynamics() first so that self.M is current.

    Args:
        js1 (JointState): State of joint 1.
        js2 (JointState): State of joint 2.

    Returns:
        Tuple[float, float, float]: Current (KE, PE, E_total) in Joules.
    """
    # Ensure M reflects the current configuration
    self.update_dynamics(js1, js2)

    # Kinetic energy: KE = ½ θ̇ᵀ M θ̇
    theta_dot = np.array([[js1.vel], [js2.vel]])
    KE = 0.5 * float(theta_dot.T @ self.M @ theta_dot)

    # Potential energy: PE = M2*g*Lc2*sin(θ2)
    PE = self._M2 * self._g * self._Lc2 * np.sin(js2.pos)

    E_total = KE + PE
    return KE, PE, E_total

  def simulate_rr(self, controlled: bool = False) -> None:
    """Run the 2-DOF RR robot dynamics simulation.

    Initial conditions are fixed at the start of every call:
      theta_1 = pi/3 rad, theta_2 = pi/4 rad, all velocities = 0.

    The general process it to calculate your M, C, and G arrays (via self.update_dynamics()) 
    and with your tau, calculate your theta_accel and use that to do trapezoidal calculation to propogte states 

    Each timestep performs the following steps:
      1. Call self.update_dynamics(curr_js1, curr_js2) to update M, C, G.
      2. Compute tau: note whether controlled is True or False.
      3. Compute angular accelerations:
      4. Integrate velocities using trapazoid_calc.
      5. Integrate positions using trapazoid_calc.
      6. Call self.calculate_energy(curr_js1, curr_js2) and store results.

    Data stored on self after the call:
      self.KE_data      — list of float, one per timestep
      self.PE_data      — list of float, one per timestep
      self.E_total_data — list of float, one per timestep
      self.joint_1_data — list of JointState, one per timestep
      self.joint_2_data — list of JointState, one per timestep
      self.tau_data     — list of (2, 1) ndarray, one per timestep

    Args:
        controlled (bool): If True, apply PD control. If False, tau = 0.
    """
    self.clear_history()  # Clear any existing data from previous runs

    # Initial conditions: θ1=π/3, θ2=π/4, at rest
    curr_js1 = JointState(position=math.pi / 3, velocity=0.0, acceleration=0.0)
    curr_js2 = JointState(position=math.pi / 4, velocity=0.0, acceleration=0.0)

    n_steps = int(round(self._duration / self._dt))

    for _ in range(n_steps):
      # Step 1 — update M, C, G for the current state
      self.update_dynamics(curr_js1, curr_js2)

      # Step 2 — compute joint torques
      if controlled and self._kp is not None and self._kv is not None:
        tau1 = (-self._kp * (curr_js1.pos - self._joint_1_des)
                - self._kv * curr_js1.vel)
        tau2 = (-self._kp * (curr_js2.pos - self._joint_2_des)
                - self._kv * curr_js2.vel)
        tau1 = float(np.clip(tau1, -TAU_MAX, TAU_MAX))
        tau2 = float(np.clip(tau2, -TAU_MAX, TAU_MAX))
      else:
        tau1, tau2 = 0.0, 0.0

      tau = np.array([[tau1], [tau2]])

      # Step 3 — solve for angular accelerations: θ̈ = M⁻¹(τ − C − G)
      theta_ddot = np.linalg.solve(self.M, tau - self.C - self.G)
      new_accel1 = float(theta_ddot[0])
      new_accel2 = float(theta_ddot[1])

      # Step 4 — integrate velocities (trapezoidal rule)
      new_vel1 = trapazoid_calc(curr_js1.vel, curr_js1.accel, new_accel1, self._dt)
      new_vel2 = trapazoid_calc(curr_js2.vel, curr_js2.accel, new_accel2, self._dt)

      # Step 5 — integrate positions (trapezoidal rule)
      new_pos1 = trapazoid_calc(curr_js1.pos, curr_js1.vel, new_vel1, self._dt)
      new_pos2 = trapazoid_calc(curr_js2.pos, curr_js2.vel, new_vel2, self._dt)

      # Advance state
      curr_js1 = JointState(position=new_pos1, velocity=new_vel1,
                            acceleration=new_accel1)
      curr_js2 = JointState(position=new_pos2, velocity=new_vel2,
                            acceleration=new_accel2)

      # Step 6 — calculate and record energy
      KE, PE, E_total = self.calculate_energy(curr_js1, curr_js2)
      self.KE_data.append(KE)
      self.PE_data.append(PE)
      self.E_total_data.append(E_total)

      # Record joint states and torques
      self.joint_1_data.append(deepcopy(curr_js1))
      self.joint_2_data.append(deepcopy(curr_js2))
      self.tau_data.append(tau)

      # Optional live drawing
      self.draw_rr(np.array([curr_js1.pos, curr_js2.pos]))

  def control_underdamped(self) -> None:
    """Set PD gains for an underdamped closed-loop response.

    Choose self._kp and self._kv so that the robot oscillates but
    ultimately converges toward the desired joint positions.

    Do NOT call simulate_rr() here. The autograder drives the simulation
    via test_setpoint_1_underdamped().
    """
    # Underdamped: choose ζ ≈ 0.1-0.2 so the response clearly oscillates.
    self._kp = 50.0
    self._kv = 10.0

  def control_critically_damped(self) -> None:
    """Set PD gains for a critically damped (or overdamped) response.

    Choose self._kp and self._kv so that the robot converges smoothly to
    the desired joint positions without oscillating.

    Do NOT call simulate_rr() here. The autograder drives the simulation
    via test_setpoint_1_critically_damped() and
    test_setpoint_2_critically_damped().

    For test_setpoint_2_critically_damped() the autograder checks that
    both joints reach within 0.1 rad and 0.01 rad/s of the target by the
    end of the 5-second simulation.
    """
    # Critically damped / overdamped: ζ ≥ 1 for both joints.
    self._kp = 500.0
    self._kv = 400.0

  # ------------------------------------------------------------------ #
  # Provided — autograder entry points; do not modify                   #
  # ------------------------------------------------------------------ #

  def test_setpoint_1_underdamped(self) -> None:
    """Run underdamped control to setpoint 1: θd = [−π, −π/3].

    Sets desired joint positions, configures underdamped gains via
    control_underdamped(), and runs the simulation. Results are available
    on self.joint_1_data, self.joint_2_data, and self.tau_data.
    """
    self.set_desired_positions(-math.pi, -math.pi / 3)
    self.control_underdamped()
    self.simulate_rr(controlled=True)

  def test_setpoint_1_critically_damped(self) -> None:
    """Run critically damped control to setpoint 1: θd = [−π, −π/3].

    Sets desired joint positions, configures critically damped gains via
    control_critically_damped(), and runs the simulation. Results are
    available on self.joint_1_data, self.joint_2_data, and self.tau_data.
    """
    self.set_desired_positions(-math.pi, -math.pi / 3)
    self.control_critically_damped()
    self.simulate_rr(controlled=True)

  def test_setpoint_2_critically_damped(self) -> None:
    """Run critically damped control to setpoint 2: θd = [−π/2, 1.3].

    Sets desired joint positions, configures critically damped gains via
    control_critically_damped(), and runs the simulation. Results are
    available on self.joint_1_data, self.joint_2_data, and self.tau_data.

    The autograder checks that both joints converge within 0.1 rad and
    0.01 rad/s of the target by the end of the 5-second simulation.
    """
    self.set_desired_positions(-math.pi / 2, 1.3)
    self.control_critically_damped()
    self.simulate_rr(controlled=True)

  # ------------------------------------------------------------------ #
  # Provided plotting helpers — do not modify                          #
  # Can be used by students to visualize results, 
  # but are not called by the autograder.
  # ------------------------------------------------------------------ #

  def draw_rr(self, joint_angles: np.ndarray) -> None:
    """Render the robot at the given joint angles.

    Args:
        joint_angles (np.ndarray): (2,) array [theta_1, theta_2] in radians.
    """
    if not self._drawing_enabled:
      return

    general.check_proper_numpy_format(joint_angles, (2,))
    self.calculate_fk(joint_angles)

    if not self._drawn_once:
      plt.show(block=False)
      plt.pause(0.5)
      self._drawn_once = True

    self._base_frame.draw()
    current_transform = self._base_frame.dh_transform

    for index, joint in enumerate(self.joints):
      prev_transform = current_transform
      current_transform = current_transform @ joint.dh_transform
      joint.set_final_transform(current_transform)
      joint.draw()
      self._links[index].update_frames(prev_transform, current_transform)
      self._links[index].draw()

    plt.pause(0.001)

  def plot_energy_summary(self, label: str = "") -> None:
    """Print an energy summary and display a KE / PE / E_total plot.

    Prints min, max, and std of each energy component to stdout, then
    opens a three-panel figure.  Call this after simulate_rr().

    Args:
        label (str): Title string for the figure and printed header.
    """
    ke = np.array(self.KE_data)
    pe = np.array(self.PE_data)
    et = np.array(self.E_total_data)

    print("  {:<30s}  min={:8.4f}  max={:8.4f}  std={:8.4f}".format(
        label + " KE", ke.min(), ke.max(), ke.std()))
    print("  {:<30s}  min={:8.4f}  max={:8.4f}  std={:8.4f}".format(
        label + " PE", pe.min(), pe.max(), pe.std()))
    print("  {:<30s}  min={:8.4f}  max={:8.4f}  std={:8.4f}".format(
        label + " E_total", et.min(), et.max(), et.std()))

    timesteps = np.arange(len(ke)) * self._dt

    fig, axes = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
    fig.suptitle(label)

    axes[0].plot(timesteps, ke, color='tab:blue')
    axes[0].set_ylabel('KE (J)')
    axes[0].grid(True)

    axes[1].plot(timesteps, pe, color='tab:orange')
    axes[1].set_ylabel('PE (J)')
    axes[1].grid(True)

    axes[2].plot(timesteps, et, color='tab:green')
    axes[2].set_ylabel('E total (J)')
    axes[2].set_xlabel('Time (s)')
    axes[2].grid(True)

    plt.tight_layout()
    plt.show(block=False)

  def plot_joint_states(self, label: str = "") -> None:
    """Display joint positions, velocities, and torques from the last run.

    Opens a figure with three vertically stacked subplots:
      - Top:    joint 1 and joint 2 position (rad) vs time
      - Middle: joint 1 and joint 2 velocity (rad/s) vs time
      - Bottom: joint 1 and joint 2 torque (N·m) vs time,
                with ±TAU_MAX limits shown as dashed lines

    Call this after simulate_rr().

    Args:
        label (str): Title string for the figure.
    """
    pos1 = np.array([js.pos for js in self.joint_1_data])
    pos2 = np.array([js.pos for js in self.joint_2_data])
    vel1 = np.array([js.vel for js in self.joint_1_data])
    vel2 = np.array([js.vel for js in self.joint_2_data])
    tau1 = np.array([t[0][0] for t in self.tau_data])
    tau2 = np.array([t[1][0] for t in self.tau_data])

    timesteps = np.arange(len(pos1)) * self._dt

    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    fig.suptitle(label)

    axes[0].plot(timesteps, pos1, color='tab:blue', label='Joint 1')
    axes[0].plot(timesteps, pos2, color='tab:orange', label='Joint 2')
    axes[0].set_ylabel('Position (rad)')
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(timesteps, vel1, color='tab:blue', label='Joint 1')
    axes[1].plot(timesteps, vel2, color='tab:orange', label='Joint 2')
    axes[1].set_ylabel('Velocity (rad/s)')
    axes[1].legend()
    axes[1].grid(True)

    axes[2].plot(timesteps, tau1, color='tab:blue', label='Joint 1')
    axes[2].plot(timesteps, tau2, color='tab:orange', label='Joint 2')
    axes[2].axhline(TAU_MAX, color='red', linestyle='--', linewidth=0.8,
                    label=f'±{TAU_MAX} N·m')
    axes[2].axhline(-TAU_MAX, color='red', linestyle='--', linewidth=0.8)
    axes[2].set_ylabel('Torque (N·m)')
    axes[2].set_xlabel('Time (s)')
    axes[2].legend()
    axes[2].grid(True)

    plt.tight_layout()
    plt.show(block=False)

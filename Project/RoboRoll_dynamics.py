"""RoboRollDynamics.py — Section 4 Option A: Dynamics for RoboRoll 4-DOF robot.

Saves two PNG files to the Project folder (same directory as this script):
  dynamics_passive.png    — Part 1 energy plot
  dynamics_controlled.png — Part 2 torque / velocity / position plot


Part 1 — Passive motion: robot released from an unstable configuration and
         falls under gravity with tau = 0.  Plots KE, PE, and total energy
         (total must be conserved).

Part 2 — Controlled motion: PD + gravity-feedforward controller drives the
         arm from home to a painting target.  Plots joint torques and joint
         velocities.

Physics notes
─────────────
After the fixed Rx(-π/2) tilt at the base, joints J2/J3/J4 operate in a
vertical plane whose normal rotates with J1.  The world-frame Z coordinate
of a point at planar reach r from the J2 origin and planar elevation angle ψ
is D1 - r*sin(ψ), so positive θ2 lowers the elbow and positive (θ2+θ3) lowers
the wrist.  All dynamics math uses SI units (m, kg, N·m, s).
"""

import os

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# ── Physical parameters ──────────────────────────────────────────────────────
G_ACCEL = 9.81        # gravitational acceleration [m/s²]

# Link lengths [m]  (robot is defined in mm; divide by 1000)
A2  = 0.700           # shoulder → elbow
A3  = 0.500           # elbow    → wrist / nozzle
D1  = 0.500           # base column height
LC2 = A2 / 2          # link-2 COM from J2 (slender rod)
LC3 = A3 / 2          # link-3 COM from J3

# Masses [kg]  (physically plausible for a construction painting robot)
M1 = 30.0             # base column (contributes only to J1 yaw inertia)
M2 =  8.0             # upper arm aluminium tube + shoulder motor
M3 =  5.0             # forearm aluminium tube  + elbow motor
M4 =  2.0             # spray nozzle assembly   + wrist motor

# Slender-rod rotational inertias about link COMs [kg·m²]
I2        = M2 * A2**2 / 12
I3        = M3 * A3**2 / 12
I4_NOZZLE = 0.02              # nozzle rotational inertia about wrist axis
I1_BASE   = M1 * D1**2 / 12  # base column about J1 axis


# ── Potential energy ─────────────────────────────────────────────────────────

def potential_energy(q: np.ndarray) -> float:
    """Total gravitational PE [J]."""
    t2, t3 = q[1], q[2]
    h2 = D1 - LC2 * np.sin(t2)
    h3 = D1 - A2  * np.sin(t2) - LC3 * np.sin(t2 + t3)
    h4 = D1 - A2  * np.sin(t2) - A3  * np.sin(t2 + t3)
    return G_ACCEL * (M2 * h2 + M3 * h3 + M4 * h4)


# ── Mass matrix M(q) ─────────────────────────────────────────────────────────

def mass_matrix(q: np.ndarray) -> np.ndarray:
    """4×4 configuration-dependent mass matrix [kg·m²].

    J1 (yaw) inertia is computed from the horizontal reach of each link COM.
    J2/J3 use the standard planar two-link result with a distal point mass.
    J4 (wrist) appears only on the diagonal with the nozzle inertia.
    Off-diagonal coupling between J1 and the planar joints is zero because the
    planar arm's net horizontal momentum has no preferred J1 direction.
    """
    t2, t3 = q[1], q[2]
    c2  = np.cos(t2)
    c23 = np.cos(t2 + t3)
    c3  = np.cos(t3)

    # Horizontal reach of each COM from the J1 axis
    r2  = LC2 * c2
    r3h = A2 * c2 + LC3 * c23
    r4h = A2 * c2 + A3  * c23

    # J1 yaw (entire arm swings about world Z)
    M11 = I1_BASE + M2 * r2**2 + M3 * r3h**2 + M4 * r4h**2

    # J2 planar shoulder
    M22 = (I2 + M2 * LC2**2
           + I3 + M3 * (A2**2 + 2*A2*LC3*c3 + LC3**2)
           +      M4 * (A2**2 + 2*A2*A3 *c3 + A3**2))

    # J2–J3 coupling
    M23 = I3 + M3 * (LC3**2 + A2*LC3*c3) + M4 * (A3**2 + A2*A3*c3)

    # J3 planar elbow
    M33 = I3 + M3 * LC3**2 + M4 * A3**2

    # J4 wrist (only nozzle inertia; no position coupling)
    M44 = I4_NOZZLE

    M = np.zeros((4, 4))
    M[0, 0] = M11
    M[1, 1] = M22
    M[1, 2] = M[2, 1] = M23
    M[2, 2] = M33
    M[3, 3] = M44
    return M


# ── Gravity vector G(q) = ∂PE/∂q ─────────────────────────────────────────────

def gravity_vector(q: np.ndarray) -> np.ndarray:
    """Generalised gravity forces [N·m].  Sign: M*qdd + C*qd + G = tau."""
    t2, t3 = q[1], q[2]
    c2  = np.cos(t2)
    c23 = np.cos(t2 + t3)

    g2 = -G_ACCEL * ((M2*LC2 + (M3+M4)*A2) * c2
                     + (M3*LC3 + M4*A3) * c23)
    g3 = -G_ACCEL * (M3*LC3 + M4*A3) * c23
    return np.array([0.0, g2, g3, 0.0])


# ── Kinetic energy ────────────────────────────────────────────────────────────

def kinetic_energy(q: np.ndarray, qd: np.ndarray) -> float:
    """T = ½ qd^T M(q) qd [J]."""
    M = mass_matrix(q)
    return 0.5 * qd @ M @ qd


# ── Coriolis / centrifugal torques via Christoffel symbols ───────────────────

def coriolis_torque(q: np.ndarray, qd: np.ndarray, dq: float = 1e-5) -> np.ndarray:
    """τ_cor[i] = Σ_{j,k} Γ_ijk · qd_j · qd_k.

    Γ_ijk = ½(∂M_ij/∂q_k + ∂M_ik/∂q_j − ∂M_jk/∂q_i) computed by
    central-difference on mass_matrix().
    """
    n = 4
    dM = np.zeros((n, n, n))   # dM[i, j, k] = ∂M_ij/∂q_k
    for k in range(n):
        qp, qm = q.copy(), q.copy()
        qp[k] += dq
        qm[k] -= dq
        dM[:, :, k] = (mass_matrix(qp) - mass_matrix(qm)) / (2.0 * dq)

    tau = np.zeros(n)
    for i in range(n):
        for j in range(n):
            for k in range(n):
                gamma = 0.5 * (dM[i, j, k] + dM[i, k, j] - dM[j, k, i])
                tau[i] += gamma * qd[j] * qd[k]
    return tau


# ── Forward dynamics ──────────────────────────────────────────────────────────

def forward_dynamics(q: np.ndarray, qd: np.ndarray,
                     tau: np.ndarray) -> np.ndarray:
    """Solve M(q)q̈ = τ − C(q,qd)qd − G(q) for q̈ [rad/s²]."""
    M   = mass_matrix(q)
    rhs = tau - coriolis_torque(q, qd) - gravity_vector(q)
    return np.linalg.solve(M, rhs)


# ── Part 1: Passive simulation ────────────────────────────────────────────────

def simulate_passive(q0: np.ndarray, qd0: np.ndarray | None = None,
                     t_end: float = 4.0, dt: float = 0.005):
    """Integrate free fall under gravity (τ = 0).

    Returns
    -------
    t  : (N,)    time array [s]
    q  : (N, 4)  joint angles [rad]
    qd : (N, 4)  joint velocities [rad/s]
    """
    if qd0 is None:
        qd0 = np.zeros(4)

    def _ode(t, state):
        q_s  = state[:4]
        qd_s = state[4:]
        qdd  = forward_dynamics(q_s, qd_s, np.zeros(4))
        return np.concatenate([qd_s, qdd])

    state0 = np.concatenate([np.asarray(q0, float), np.asarray(qd0, float)])
    sol = solve_ivp(_ode, [0.0, t_end], state0, method='RK45',
                    max_step=dt, dense_output=False, rtol=1e-6, atol=1e-8)
    t  = sol.t
    q  = sol.y[:4].T
    qd = sol.y[4:].T
    return t, q, qd


# ── Part 2: Controlled simulation ─────────────────────────────────────────────

def simulate_controlled(q0: np.ndarray, q_des: np.ndarray,
                        t_end: float = 8.0, dt: float = 0.005,
                        Kp: np.ndarray | None = None,
                        Kd: np.ndarray | None = None):
    """PD controller with gravity feed-forward.

    τ = Kp*(q_des − q) + Kd*(0 − qd) + G(q)

    Returns
    -------
    t   : (N,)    time [s]
    q   : (N, 4)  joint angles [rad]
    qd  : (N, 4)  joint velocities [rad/s]
    tau : (N, 4)  applied torques [N·m]
    """
    if Kp is None:
        Kp = np.array([100.0, 200.0, 120.0, 15.0])   # [J1, J2, J3, J4]
    if Kd is None:
        Kd = np.array([ 30.0,  80.0,  50.0,  6.0])

    q_des = np.asarray(q_des, float)

    def _ode(t, state):
        q_s  = state[:4]
        qd_s = state[4:]
        tau_s = Kp * (q_des - q_s) + Kd * (-qd_s) + gravity_vector(q_s)
        qdd   = forward_dynamics(q_s, qd_s, tau_s)
        return np.concatenate([qd_s, qdd])

    state0 = np.concatenate([np.asarray(q0, float), np.zeros(4)])
    sol = solve_ivp(_ode, [0.0, t_end], state0, method='RK45',
                    max_step=dt, dense_output=False, rtol=1e-6, atol=1e-8)
    t  = sol.t
    q  = sol.y[:4].T
    qd = sol.y[4:].T

    # Recompute torques at saved timepoints for plotting
    tau = np.zeros((len(t), 4))
    for i in range(len(t)):
        tau[i] = Kp*(q_des - q[i]) + Kd*(-qd[i]) + gravity_vector(q[i])
    return t, q, qd, tau


# ── Plotting — Part 1 ─────────────────────────────────────────────────────────

def plot_energy(t: np.ndarray, q: np.ndarray, qd: np.ndarray,
                title: str = "RoboRoll — Passive Motion (τ = 0)") -> None:
    """Three-panel energy plot: KE, PE, and total (should be conserved)."""
    KE = np.array([kinetic_energy(q[i], qd[i]) for i in range(len(t))])
    PE = np.array([potential_energy(q[i])       for i in range(len(t))])
    TE = KE + PE

    fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    fig.suptitle(title, fontsize=13)

    axes[0].plot(t, KE, color='tab:blue')
    axes[0].set_ylabel('KE (J)');   axes[0].grid(True)

    axes[1].plot(t, PE, color='tab:orange')
    axes[1].set_ylabel('PE (J)');   axes[1].grid(True)

    axes[2].plot(t, TE, color='tab:green', linewidth=2)
    drift_pct = 100 * (TE.max() - TE.min()) / (abs(TE.mean()) + 1e-9)
    axes[2].set_ylabel('Total Energy (J)')
    axes[2].set_xlabel('Time (s)')
    axes[2].set_title(f'Total Energy  (drift = {drift_pct:.2f} %)')
    axes[2].grid(True)

    plt.tight_layout()
    return fig


# ── Plotting — Part 2 ─────────────────────────────────────────────────────────

def plot_controlled(t: np.ndarray, q: np.ndarray, qd: np.ndarray,
                    tau: np.ndarray, q_des: np.ndarray,
                    title: str = "RoboRoll — Controlled Motion") -> None:
    """Four-panel plot: joint torques, joint velocities, joint positions, energy."""
    labels = [r'$\theta_1$ (yaw)', r'$\theta_2$ (shoulder)',
              r'$\theta_3$ (elbow)', r'$\theta_4$ (wrist)']
    colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red']

    KE = np.array([kinetic_energy(q[i], qd[i]) for i in range(len(t))])
    PE = np.array([potential_energy(q[i])       for i in range(len(t))])

    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    fig.suptitle(title, fontsize=13)

    # Joint torques
    ax = axes[0, 0]
    for i in range(4):
        ax.plot(t, tau[:, i], color=colors[i], label=labels[i])
    ax.set_ylabel('Torque (N·m)')
    ax.set_title('Joint Torques')
    ax.legend(fontsize=8); ax.grid(True)

    # Joint velocities
    ax = axes[0, 1]
    for i in range(4):
        ax.plot(t, np.degrees(qd[:, i]), color=colors[i], label=labels[i])
    ax.set_ylabel('Velocity (deg/s)')
    ax.set_title('Joint Velocities')
    ax.legend(fontsize=8); ax.grid(True)

    # Joint positions vs desired
    ax = axes[1, 0]
    for i in range(4):
        ax.plot(t, np.degrees(q[:, i]),     color=colors[i], label=labels[i])
        ax.axhline(np.degrees(q_des[i]), color=colors[i], linestyle='--',
                   linewidth=0.9, alpha=0.6)
    ax.set_ylabel('Angle (deg)')
    ax.set_xlabel('Time (s)')
    ax.set_title('Joint Positions  (dashed = target)')
    ax.legend(fontsize=8); ax.grid(True)

    # Energy
    ax = axes[1, 1]
    ax.plot(t, KE,      color='tab:blue',   label='Kinetic')
    ax.plot(t, PE,      color='tab:orange', label='Potential')
    ax.plot(t, KE + PE, color='tab:green',  label='Total', linestyle='--', linewidth=2)
    ax.set_ylabel('Energy (J)')
    ax.set_xlabel('Time (s)')
    ax.set_title('Energy')
    ax.legend(fontsize=8); ax.grid(True)

    plt.tight_layout()
    return fig


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    # ── Part 1: Passive motion ────────────────────────────────────────────────
    print("Part 1 — Passive motion under gravity")
    print("  Initial config: θ1=0, θ2=-60° (arm above horizontal), θ3=0, θ4=0")
    print("  Releasing from rest — arm falls under gravity, τ = 0")

    q0_passive = np.array([0.0, np.radians(-60.0), 0.0, 0.0])
    t1, q1, qd1 = simulate_passive(q0_passive, t_end=3.5)

    KE0 = kinetic_energy(q1[0], qd1[0])
    PE0 = potential_energy(q1[0])
    print(f"  Initial energy: KE={KE0:.2f} J  PE={PE0:.2f} J  TE={KE0+PE0:.2f} J")
    TE1 = np.array([kinetic_energy(q1[i], qd1[i]) + potential_energy(q1[i])
                    for i in range(len(t1))])
    drift = 100 * (TE1.max() - TE1.min()) / abs(TE1.mean())
    print(f"  Energy drift:   {drift:.3f} %  (should be < 1 %)")

    fig1 = plot_energy(t1, q1, qd1)
    out1 = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dynamics_passive.png")
    fig1.savefig(out1, dpi=150, bbox_inches="tight", facecolor="white")
    print(f"  Saved: {out1}")

    # ── Part 2: Controlled motion ─────────────────────────────────────────────
    print("\nPart 2 — Controlled motion (PD + gravity feed-forward)")
    print("  From home [0,0,0,0] → painting target on a wall")

    q0_ctrl  = np.zeros(4)
    # Target: face the wall (θ1=45°), shoulder slightly up (θ2=-25°),
    # elbow bent inward (θ3=15°), wrist neutral
    q_des = np.array([np.radians(45.0), np.radians(-25.0),
                      np.radians(15.0), 0.0])

    t2, q2, qd2, tau2 = simulate_controlled(q0_ctrl, q_des, t_end=7.0)

    peak_tau = np.abs(tau2).max(axis=0)
    for i, name in enumerate(['J1 yaw', 'J2 shoulder', 'J3 elbow', 'J4 wrist']):
        print(f"  Peak |τ_{i+1}| ({name}): {peak_tau[i]:.1f} N·m")

    peak_vel = np.degrees(np.abs(qd2).max(axis=0))
    for i, name in enumerate(['J1 yaw', 'J2 shoulder', 'J3 elbow', 'J4 wrist']):
        print(f"  Peak |qd_{i+1}| ({name}): {peak_vel[i]:.1f} deg/s")

    fig2 = plot_controlled(t2, q2, qd2, tau2, q_des)
    out2 = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dynamics_controlled.png")
    fig2.savefig(out2, dpi=150, bbox_inches="tight", facecolor="white")
    print(f"  Saved: {out2}")

    plt.show()
    print("\nClose the plot windows to exit.")


if __name__ == "__main__":
    main()

"""Generate product-style mockup renders for the RoboRoll pitch deck."""

import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


OUT_DIR = Path(__file__).resolve().parent


def _set_clean_axes(ax, title=None):
    ax.set_facecolor("#f8fafc")
    ax.figure.set_facecolor("#f8fafc")
    ax.set_xlim(-900, 1100)
    ax.set_ylim(-900, 1100)
    ax.set_zlim(0, 1250)
    ax.set_box_aspect((2.0, 2.0, 1.25))
    ax.view_init(elev=24, azim=-48)
    ax.grid(False)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    ax.xaxis.pane.set_alpha(0)
    ax.yaxis.pane.set_alpha(0)
    ax.zaxis.pane.set_alpha(0)
    ax.xaxis.line.set_alpha(0)
    ax.yaxis.line.set_alpha(0)
    ax.zaxis.line.set_alpha(0)
    if title:
        ax.set_title(title, fontsize=18, pad=18, color="#17202a", weight="bold")


def _cuboid(ax, center, size, color, alpha=1.0):
    cx, cy, cz = center
    sx, sy, sz = size
    x = [cx - sx / 2, cx + sx / 2]
    y = [cy - sy / 2, cy + sy / 2]
    z = [cz - sz / 2, cz + sz / 2]
    verts = [
        [(x[0], y[0], z[0]), (x[1], y[0], z[0]), (x[1], y[1], z[0]), (x[0], y[1], z[0])],
        [(x[0], y[0], z[1]), (x[1], y[0], z[1]), (x[1], y[1], z[1]), (x[0], y[1], z[1])],
        [(x[0], y[0], z[0]), (x[1], y[0], z[0]), (x[1], y[0], z[1]), (x[0], y[0], z[1])],
        [(x[0], y[1], z[0]), (x[1], y[1], z[0]), (x[1], y[1], z[1]), (x[0], y[1], z[1])],
        [(x[0], y[0], z[0]), (x[0], y[1], z[0]), (x[0], y[1], z[1]), (x[0], y[0], z[1])],
        [(x[1], y[0], z[0]), (x[1], y[1], z[0]), (x[1], y[1], z[1]), (x[1], y[0], z[1])],
    ]
    poly = Poly3DCollection(verts, facecolors=color, edgecolors="#94a3b8", linewidths=0.5, alpha=alpha)
    ax.add_collection3d(poly)


def _cylinder_between(ax, start, end, radius, color, alpha=1.0, n=24):
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    axis = end - start
    length = np.linalg.norm(axis)
    if length < 1e-9:
        return
    axis = axis / length
    tmp = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(axis, tmp)) > 0.95:
        tmp = np.array([1.0, 0.0, 0.0])
    v = np.cross(axis, tmp)
    v = v / np.linalg.norm(v)
    w = np.cross(axis, v)
    theta = np.linspace(0, 2 * np.pi, n)
    circle = radius * (np.outer(np.cos(theta), v) + np.outer(np.sin(theta), w))
    p0 = start + circle
    p1 = end + circle
    x = np.vstack([p0[:, 0], p1[:, 0]])
    y = np.vstack([p0[:, 1], p1[:, 1]])
    z = np.vstack([p0[:, 2], p1[:, 2]])
    ax.plot_surface(x, y, z, color=color, alpha=alpha, shade=True, linewidth=0)


def _sphere(ax, center, radius, color):
    u = np.linspace(0, 2 * np.pi, 28)
    v = np.linspace(0, np.pi, 14)
    x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    z = center[2] + radius * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(x, y, z, color=color, shade=True, linewidth=0)


def _robot_points(base=(0, 0, 0), yaw=0.72, shoulder=-0.28, elbow=0.78, wrist=-0.22):
    base = np.asarray(base, dtype=float)
    z0 = base + np.array([0, 0, 115])
    shoulder_p = base + np.array([0, 0, 500])
    r2 = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    zdir = np.array([0.0, 0.0, 1.0])
    p1 = shoulder_p + 700 * (math.cos(shoulder) * r2 - math.sin(shoulder) * zdir)
    p2 = p1 + 500 * (math.cos(shoulder + elbow) * r2 - math.sin(shoulder + elbow) * zdir)
    p3 = p2 + 200 * (math.cos(shoulder + elbow + wrist) * r2 - math.sin(shoulder + elbow + wrist) * zdir)
    return [base, z0, shoulder_p, p1, p2, p3]


def _draw_robot(ax, color="#1f6feb"):
    pts = _robot_points()
    _cuboid(ax, (0, 0, 45), (360, 300, 90), "#334155")
    _cylinder_between(ax, pts[1], pts[2], 58, "#475569")
    _sphere(ax, pts[2], 86, "#1f6feb")
    _cylinder_between(ax, pts[2], pts[3], 45, color)
    _sphere(ax, pts[3], 70, "#d65f21")
    _cylinder_between(ax, pts[3], pts[4], 38, "#10b981")
    _sphere(ax, pts[4], 56, "#7c3aed")
    _cylinder_between(ax, pts[4], pts[5], 20, "#111827")
    _sphere(ax, pts[5], 34, "#111827")
    ax.scatter([pts[-1][0]], [pts[-1][1]], [pts[-1][2]], s=120, c="#ef4444", depthshade=False)
    return pts


def _draw_room(ax):
    floor = [(-850, -850, 0), (1000, -850, 0), (1000, 1000, 0), (-850, 1000, 0)]
    wall_x = [(900, -520, 120), (900, 520, 120), (900, 520, 980), (900, -520, 980)]
    wall_y = [(-520, 900, 120), (520, 900, 120), (520, 900, 980), (-520, 900, 980)]
    for verts, color, alpha in [(floor, "#e2e8f0", 0.55), (wall_x, "#dbeafe", 0.7), (wall_y, "#dcfce7", 0.7)]:
        ax.add_collection3d(Poly3DCollection([verts], facecolors=color, edgecolors="#94a3b8", alpha=alpha, linewidths=1.0))


def _draw_paint_paths(ax):
    for i, z in enumerate([820, 720, 620, 520, 420]):
        y = np.linspace(-360, 360, 80)
        x = np.full_like(y, 902)
        ax.plot(x, y, np.full_like(y, z), color=["#1f6feb", "#16a34a", "#d65f21", "#7c3aed", "#16a34a"][i], linewidth=4)
    t = np.linspace(0, 2 * np.pi, 160)
    ax.plot(205 * np.cos(t), np.full_like(t, 902), 610 + 205 * np.sin(t), color="#d65f21", linewidth=4)
    for cx in [-90, 90]:
        ax.plot(cx + 38 * np.cos(t), np.full_like(t, 902), 690 + 38 * np.sin(t), color="#d65f21", linewidth=4)
    s = np.linspace(math.radians(205), math.radians(335), 80)
    ax.plot(145 * np.cos(s), np.full_like(s, 902), 580 + 145 * np.sin(s), color="#d65f21", linewidth=4)


def render_hero():
    fig = plt.figure(figsize=(14, 8), dpi=180)
    ax = fig.add_subplot(111, projection="3d")
    _set_clean_axes(ax, "Portable robotic painting system")
    _draw_room(ax)
    _draw_paint_paths(ax)
    pts = _draw_robot(ax)
    ax.plot([pts[-1][0], 900], [pts[-1][1], pts[-1][1]], [pts[-1][2], pts[-1][2]], color="#ef4444", linestyle="--", linewidth=1.4)
    fig.savefig(OUT_DIR / "mockup_robot_hero.png", bbox_inches="tight", pad_inches=0.1)
    plt.close(fig)


def render_architecture():
    fig = plt.figure(figsize=(12, 8), dpi=180)
    ax = fig.add_subplot(111, projection="3d")
    _set_clean_axes(ax, "RoboRoll 4-DOF arm architecture")
    pts = _draw_robot(ax)
    labels = ["mobile base", "base column", "shoulder", "elbow", "wrist", "paint nozzle"]
    offsets = [(-120, -110, 35), (-170, 20, 20), (-160, 40, 120), (70, 60, 80), (80, -50, 50), (70, 30, 25)]
    for p, label, off in zip(pts, labels, offsets):
        q = p + np.array(off)
        ax.text(q[0], q[1], q[2], label, fontsize=10, color="#17202a")
        ax.plot([p[0], q[0]], [p[1], q[1]], [p[2], q[2]], color="#64748b", linewidth=1)
    fig.savefig(OUT_DIR / "mockup_robot_architecture.png", bbox_inches="tight", pad_inches=0.1)
    plt.close(fig)


def render_workflow():
    fig = plt.figure(figsize=(14, 8), dpi=180)
    ax = fig.add_subplot(111, projection="3d")
    _set_clean_axes(ax, "Room-scale wall painting workflow")
    _draw_room(ax)
    _draw_paint_paths(ax)
    _draw_robot(ax, color="#0f766e")
    ax.text(-780, -760, 70, "1. Position", fontsize=12, weight="bold", color="#17202a")
    ax.text(-780, -760, 10, "2. Calibrate wall plane", fontsize=12, weight="bold", color="#17202a")
    ax.text(120, -760, 70, "3. Execute path", fontsize=12, weight="bold", color="#17202a")
    ax.text(120, -760, 10, "4. Inspect coverage", fontsize=12, weight="bold", color="#17202a")
    fig.savefig(OUT_DIR / "mockup_robot_workflow.png", bbox_inches="tight", pad_inches=0.1)
    plt.close(fig)


def render_control():
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), dpi=180)
    fig.patch.set_facecolor("#f8fafc")
    t = np.linspace(0, 8, 400)
    series = [
        ("Joint tracking", np.exp(-0.55 * t) * np.cos(2.5 * t), "#1f6feb", "error"),
        ("Nozzle speed", 0.45 + 0.08 * np.sin(1.8 * t) + 0.03 * np.sin(5.1 * t), "#16a34a", "m/s"),
        ("Paint flow", 0.75 + 0.05 * np.sin(2.4 * t), "#d65f21", "normalized"),
        ("Torque demand", 18 * np.exp(-0.24 * t) * np.sin(3.2 * t), "#7c3aed", "N m"),
    ]
    for ax, (title, y, color, ylabel) in zip(axes.ravel(), series):
        ax.plot(t, y, color=color, linewidth=2.5)
        ax.fill_between(t, y, np.min(y), color=color, alpha=0.14)
        ax.set_title(title, loc="left", fontsize=13, weight="bold", color="#17202a")
        ax.set_xlabel("time (s)")
        ax.set_ylabel(ylabel)
        ax.grid(True, color="#d7dde5")
        ax.set_facecolor("white")
    fig.suptitle("Control dashboard mockup", fontsize=18, weight="bold", color="#17202a")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "mockup_control_dashboard.png", bbox_inches="tight", pad_inches=0.1)
    plt.close(fig)

def render_problem_scene():
    fig = plt.figure(figsize=(14, 8), dpi=180)
    ax = fig.add_subplot(111, projection="3d")
    _set_clean_axes(ax, "Manual wall finishing bottleneck")
    _draw_room(ax)
    wall = [(902, -620, 160), (902, 620, 160), (902, 620, 1050), (902, -620, 1050)]
    ax.add_collection3d(Poly3DCollection([wall], facecolors="#fef3c7", edgecolors="#d6a23a", alpha=0.72, linewidths=1.0))
    for z in [300, 450, 600, 750, 900]:
        ax.plot([902, 902], [-540, 540], [z, z], color="#f59e0b", linewidth=2.0, alpha=0.65)
    _cuboid(ax, (-520, -520, 38), (260, 180, 76), "#475569")
    _cylinder_between(ax, (-520, -520, 90), (-520, -520, 520), 28, "#64748b")
    _cylinder_between(ax, (-520, -520, 520), (-520, -520, 950), 18, "#94a3b8")
    ax.text(-760, -640, 160, "repetitive passes", fontsize=13, weight="bold", color="#17202a")
    ax.text(-760, -640, 80, "fatigue + rework risk", fontsize=12, color="#475569")
    fig.savefig(OUT_DIR / "mockup_problem_scene.png", bbox_inches="tight", pad_inches=0.1)
    plt.close(fig)


def render_nozzle_closeup():
    fig = plt.figure(figsize=(12, 8), dpi=180)
    ax = fig.add_subplot(111, projection="3d")
    _set_clean_axes(ax, "Paint end-effector concept")
    ax.set_xlim(-450, 650)
    ax.set_ylim(-450, 450)
    ax.set_zlim(0, 900)
    wrist = np.array([-260, 0, 430])
    nozzle = np.array([360, 0, 470])
    _sphere(ax, wrist, 72, "#7c3aed")
    _cylinder_between(ax, wrist, nozzle, 42, "#111827")
    _cylinder_between(ax, nozzle, nozzle + np.array([180, 0, 0]), 24, "#ef4444")
    wall = [(570, -260, 180), (570, 260, 180), (570, 260, 760), (570, -260, 760)]
    ax.add_collection3d(Poly3DCollection([wall], facecolors="#dbeafe", edgecolors="#94a3b8", alpha=0.75, linewidths=1.0))
    cone = [
        [(540, -120, 370), (540, 120, 370), (540, 120, 570), (540, -120, 570)],
        [(360, 0, 470), (540, -120, 370), (540, 120, 370)],
        [(360, 0, 470), (540, 120, 570), (540, -120, 570)],
    ]
    ax.add_collection3d(Poly3DCollection(cone, facecolors="#f97316", edgecolors="#fb923c", alpha=0.18, linewidths=0.6))
    for x in [572, 574, 576]:
        ax.plot([x, x], [-110, 110], [470, 470], color="#f97316", linewidth=4, alpha=0.75)
    ax.text(-300, -270, 620, "wrist joint", fontsize=12, weight="bold", color="#17202a")
    ax.text(120, 130, 650, "controlled spray zone", fontsize=12, weight="bold", color="#17202a")
    fig.savefig(OUT_DIR / "mockup_nozzle_closeup.png", bbox_inches="tight", pad_inches=0.1)
    plt.close(fig)


def render_path_planning():
    fig, ax = plt.subplots(figsize=(12, 8), dpi=180)
    fig.patch.set_facecolor("#f8fafc")
    ax.set_facecolor("white")
    ax.set_aspect("equal")
    ax.set_xlim(-360, 360)
    ax.set_ylim(250, 850)
    ax.set_title("Wall path planning mockup", fontsize=18, weight="bold", color="#17202a", pad=16)
    ax.set_xlabel("wall horizontal position (mm)")
    ax.set_ylabel("height (mm)")
    ax.grid(True, color="#e2e8f0")
    colors = ["#1f6feb", "#16a34a", "#d65f21", "#7c3aed", "#16a34a"]
    for i, z in enumerate([760, 650, 540, 430, 320]):
        y = np.linspace(-270, 270, 90)
        ax.plot(y, np.full_like(y, z), color=colors[i], linewidth=4)
        ax.scatter(y[::15], np.full_like(y[::15], z), s=24, color=colors[i])
    t = np.linspace(0, 2 * np.pi, 160)
    ax.plot(175 * np.cos(t), 560 + 175 * np.sin(t), color="#d65f21", linewidth=4)
    ax.plot(-75 + 32 * np.cos(t), 630 + 32 * np.sin(t), color="#d65f21", linewidth=3)
    ax.plot(75 + 32 * np.cos(t), 630 + 32 * np.sin(t), color="#d65f21", linewidth=3)
    s = np.linspace(math.radians(205), math.radians(335), 80)
    ax.plot(120 * np.cos(s), 535 + 120 * np.sin(s), color="#d65f21", linewidth=4)
    ax.text(-330, 815, "coverage passes", fontsize=12, weight="bold", color="#17202a")
    ax.text(-330, 285, "detail paths", fontsize=12, weight="bold", color="#17202a")
    fig.savefig(OUT_DIR / "mockup_path_planning.png", bbox_inches="tight", pad_inches=0.1)
    plt.close(fig)


def _style_wall_path_axis(ax, title):
    ax.set_facecolor("white")
    ax.set_aspect("equal")
    ax.set_xlim(-360, 360)
    ax.set_ylim(250, 850)
    ax.set_title(title, fontsize=18, weight="bold", color="#17202a", pad=16)
    ax.set_xlabel("wall horizontal position (mm)")
    ax.set_ylabel("height (mm)")
    ax.grid(True, color="#e2e8f0")


def render_line_passes():
    fig, ax = plt.subplots(figsize=(12, 6), dpi=180)
    fig.patch.set_facecolor("#f8fafc")
    _style_wall_path_axis(ax, "Wall 1: coverage stripe passes")
    colors = ["#1f6feb", "#16a34a", "#d65f21", "#7c3aed", "#16a34a"]
    for i, z in enumerate([760, 650, 540, 430, 320]):
        y = np.linspace(-270, 270, 90)
        ax.plot(y, np.full_like(y, z), color=colors[i], linewidth=5)
        ax.scatter(y[::12], np.full_like(y[::12], z), s=26, color=colors[i], zorder=3)
        direction = ">" if i % 2 == 0 else "<"
        ax.text(300 if i % 2 == 0 else -330, z + 8, direction, fontsize=18, weight="bold", color=colors[i])
    ax.text(-335, 805, "repeatable wall coverage", fontsize=12, weight="bold", color="#17202a")
    fig.savefig(OUT_DIR / "mockup_product_lines.png", bbox_inches="tight", pad_inches=0.1)
    plt.close(fig)


def render_smiley_path():
    fig, ax = plt.subplots(figsize=(12, 6), dpi=180)
    fig.patch.set_facecolor("#f8fafc")
    _style_wall_path_axis(ax, "Wall 2: curved detail path")
    t = np.linspace(0, 2 * np.pi, 180)
    ax.plot(190 * np.cos(t), 570 + 190 * np.sin(t), color="#d65f21", linewidth=5)
    ax.plot(-78 + 34 * np.cos(t), 650 + 34 * np.sin(t), color="#d65f21", linewidth=4)
    ax.plot(78 + 34 * np.cos(t), 650 + 34 * np.sin(t), color="#d65f21", linewidth=4)
    s = np.linspace(math.radians(205), math.radians(335), 90)
    ax.plot(130 * np.cos(s), 545 + 130 * np.sin(s), color="#d65f21", linewidth=5)
    for angle in np.linspace(0, 2 * np.pi, 10, endpoint=False):
        ax.scatter([190 * math.cos(angle)], [570 + 190 * math.sin(angle)], s=28, color="#f97316", zorder=3)
    ax.text(-335, 805, "curves prove detail work, not only straight passes", fontsize=12, weight="bold", color="#17202a")
    fig.savefig(OUT_DIR / "mockup_product_smiley.png", bbox_inches="tight", pad_inches=0.1)
    plt.close(fig)


def render_calibration():
    fig = plt.figure(figsize=(14, 8), dpi=180)
    ax = fig.add_subplot(111, projection="3d")
    _set_clean_axes(ax, "Wall calibration and work envelope")
    _draw_room(ax)
    _draw_robot(ax, color="#0f766e")
    corners = [(900, -430, 260), (900, 430, 260), (900, 430, 900), (900, -430, 900)]
    for c in corners:
        _sphere(ax, c, 22, "#ef4444")
    for a, b in [(0, 1), (1, 2), (2, 3), (3, 0)]:
        p, q = np.array(corners[a]), np.array(corners[b])
        ax.plot([p[0], q[0]], [p[1], q[1]], [p[2], q[2]], color="#ef4444", linewidth=2.5)
    ax.text(720, -520, 980, "registered wall plane", fontsize=12, weight="bold", color="#17202a")
    ax.text(-760, -600, 120, "robot work envelope", fontsize=12, weight="bold", color="#17202a")
    fig.savefig(OUT_DIR / "mockup_calibration.png", bbox_inches="tight", pad_inches=0.1)
    plt.close(fig)


def render_market_segments():
    fig, ax = plt.subplots(figsize=(14, 8), dpi=180)
    fig.patch.set_facecolor("#f8fafc")
    ax.set_facecolor("#f8fafc")
    ax.axis("off")
    ax.set_title("Beachhead market: multifamily apartment turns", fontsize=20, weight="bold", color="#17202a", pad=20)

    primary = plt.Rectangle((0.08, 0.22), 0.42, 0.56, transform=ax.transAxes, facecolor="#dbeafe", edgecolor="#1f6feb", linewidth=2.2)
    ax.add_patch(primary)
    ax.text(0.29, 0.64, "Primary first customer", transform=ax.transAxes, ha="center", va="center", fontsize=13, weight="bold", color="#1f6feb")
    ax.text(0.29, 0.55, "Multifamily property teams", transform=ax.transAxes, ha="center", va="center", fontsize=20, weight="bold", color="#17202a")
    ax.text(0.29, 0.44, "Apartment turns need repeatable repainting across similar rooms, often under tight move-in schedules.", transform=ax.transAxes, ha="center", va="center", fontsize=12, color="#475569", wrap=True)
    for j in range(6):
        ax.add_patch(plt.Rectangle((0.15 + j * 0.045, 0.27), 0.03, 0.075, transform=ax.transAxes, facecolor="#64748b", alpha=0.75))

    expansion = [
        ("Expansion 1", "Commercial tenant improvements", "Large interior walls and predictable coverage paths.", "#dcfce7"),
        ("Expansion 2", "New-build interior finishing", "Schedule pressure near turnover after drywall completion.", "#ffedd5"),
    ]
    for i, (label, title, sub, color) in enumerate(expansion):
        y0 = 0.51 - i * 0.29
        rect = plt.Rectangle((0.57, y0), 0.35, 0.22, transform=ax.transAxes, facecolor=color, edgecolor="#94a3b8", linewidth=1.5)
        ax.add_patch(rect)
        ax.text(0.60, y0 + 0.155, label, transform=ax.transAxes, ha="left", va="center", fontsize=11, weight="bold", color="#64748b")
        ax.text(0.60, y0 + 0.105, title, transform=ax.transAxes, ha="left", va="center", fontsize=15, weight="bold", color="#17202a")
        ax.text(0.60, y0 + 0.045, sub, transform=ax.transAxes, ha="left", va="center", fontsize=11, color="#475569")
    fig.savefig(OUT_DIR / "mockup_market_segments.png", bbox_inches="tight", pad_inches=0.1)
    plt.close(fig)


def render_roadmap():
    fig, ax = plt.subplots(figsize=(14, 8), dpi=180)
    fig.patch.set_facecolor("#f8fafc")
    ax.set_facecolor("#f8fafc")
    ax.axis("off")
    ax.set_title("Prototype roadmap", fontsize=20, weight="bold", color="#17202a", pad=20)
    steps = [
        ("1", "Paint tool", "Nozzle, flow, cleanup"),
        ("2", "Calibration", "Wall plane sensing"),
        ("3", "Drywall tests", "Repeatability + finish"),
        ("4", "Operator workflow", "Setup and supervision"),
    ]
    xs = np.linspace(0.12, 0.88, 4)
    ax.plot(xs, [0.52] * 4, transform=ax.transAxes, color="#94a3b8", linewidth=4)
    for x, (num, title, sub) in zip(xs, steps):
        ax.add_patch(plt.Circle((x, 0.52), 0.055, transform=ax.transAxes, facecolor="#1f6feb", edgecolor="white", linewidth=3))
        ax.text(x, 0.52, num, transform=ax.transAxes, ha="center", va="center", fontsize=18, weight="bold", color="white")
        ax.text(x, 0.36, title, transform=ax.transAxes, ha="center", va="center", fontsize=16, weight="bold", color="#17202a")
        ax.text(x, 0.28, sub, transform=ax.transAxes, ha="center", va="center", fontsize=12, color="#475569")
    fig.savefig(OUT_DIR / "mockup_roadmap.png", bbox_inches="tight", pad_inches=0.1)
    plt.close(fig)


def main():
    render_hero()
    render_architecture()
    render_workflow()
    render_control()
    render_problem_scene()
    render_nozzle_closeup()
    render_path_planning()
    render_line_passes()
    render_smiley_path()
    render_calibration()
    render_market_segments()
    render_roadmap()
    print("Generated mockup renders:")
    for name in [
        "mockup_robot_hero.png",
        "mockup_robot_architecture.png",
        "mockup_robot_workflow.png",
        "mockup_control_dashboard.png",
        "mockup_problem_scene.png",
        "mockup_nozzle_closeup.png",
        "mockup_path_planning.png",
        "mockup_product_lines.png",
        "mockup_product_smiley.png",
        "mockup_calibration.png",
        "mockup_market_segments.png",
        "mockup_roadmap.png",
    ]:
        print(OUT_DIR / name)


if __name__ == "__main__":
    main()

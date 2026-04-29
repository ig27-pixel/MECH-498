"""Generate the Section 1 DH parameter diagram for RoboRoll Coatings."""
import math
import os

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arc

# ── colours ──────────────────────────────────────────────────────────────────
C_STRUCT = "#404040"
C_JOINT  = "#1a6fba"
C_X      = "#c0392b"
C_Z      = "#1565c0"
C_DIM    = "#2c3e50"
C_ALPHA  = "#7b1f1f"
C_THETA  = "#b35a00"

# ── figure ───────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(16, 9), facecolor="white")
ax   = fig.add_axes([0.02, 0.07, 0.59, 0.89])
ax_t = fig.add_axes([0.63, 0.13, 0.35, 0.74])

ax.set_xlim(-420, 1750)
ax.set_ylim(-200, 960)
ax.set_aspect("equal")
ax.axis("off")
ax.set_title(
    "RoboRoll Coatings — Modified DH Frame Assignment\n"
    r"(Home configuration: all $\theta_i = 0$)",
    fontsize=13, fontweight="bold", pad=6,
)

# ── joint positions (x, z) mm ────────────────────────────────────────────────
O  = np.array([   0.,   0.])
J1 = np.array([   0., 500.])
J3 = np.array([ 700., 500.])
J4 = np.array([1200., 500.])
JN = np.array([1400., 500.])

# ── ground ───────────────────────────────────────────────────────────────────
ax.fill([-190, 190, 190, -190], [-55, -55, 0, 0], color="#bbb", alpha=0.5, zorder=1)
ax.plot([-190, 190], [0, 0], "k-", lw=1.5, zorder=2)
for xi in np.linspace(-170, 170, 15):
    ax.plot([xi, xi - 13], [0, -28], color="#999", lw=0.8, zorder=1)

# ── links ────────────────────────────────────────────────────────────────────
def rect(ax, x, z, w, h, fc="#dadada", ec=C_STRUCT, lw=2, zorder=2):
    ax.add_patch(mpatches.FancyBboxPatch(
        (x, z), w, h, boxstyle="square,pad=0",
        ec=ec, fc=fc, lw=lw, zorder=zorder,
    ))

rect(ax, -30,  0,  60, 500)
rect(ax,   0, J1[1] - 18, 700, 36)
rect(ax, 700, J3[1] - 14, 500, 28)
rect(ax, J4[0], J4[1] - 10, 200, 20, fc="#b0cce8", ec="#1a6fba")
ax.fill([JN[0], JN[0]+55, JN[0]+55],
        [JN[1], JN[1]+38,  JN[1]-38],
        color="#b0cce8", alpha=0.5, zorder=3)

# ── joints ───────────────────────────────────────────────────────────────────
for pos, r in [(O, 34), (J1, 34), (J3, 26), (J4, 20)]:
    ax.add_patch(plt.Circle(pos, r,        color=C_JOINT, ec="k", lw=1.5, zorder=4))
    ax.add_patch(plt.Circle(pos, r * 0.40, color="white", ec="k", lw=1.0, zorder=5))

# ── frame helpers ─────────────────────────────────────────────────────────────
L = 88

def arrow(ax, ox, oz, dx, dz, color, lw=2.5):
    ax.annotate("", xy=(ox + dx, oz + dz), xytext=(ox, oz),
                arrowprops=dict(arrowstyle="->", color=color,
                                lw=lw, mutation_scale=16),
                zorder=7)

def z_dot(ax, cx, cz, color=C_Z, r=13):
    ax.add_patch(plt.Circle((cx, cz), r, color="white", ec=color, lw=2, zorder=7))
    ax.plot(cx, cz, ".", ms=6, color=color, zorder=8)

# ── Frame {0} — left of column at z = 0 ──────────────────────────────────────
ox0, oz0 = -130, 0
arrow(ax, ox0, oz0, L, 0, C_X)
arrow(ax, ox0, oz0, 0, L, C_Z)
ax.text(ox0+L+6, oz0,     r"$x_0$", fontsize=10, color=C_X, va="center")
ax.text(ox0+4,   oz0+L+6, r"$z_0$", fontsize=10, color=C_Z)
ax.text(ox0+10,  oz0-32,  r"$\{0\}$", fontsize=11, fontweight="bold", ha="center")

# ── Frame {1} — co-located with {0} at z = 0 (z₁ shares z₀ axis) ────────────
ox1, oz1 = -240, 0
arrow(ax, ox1, oz1, L, 0, C_X)
arrow(ax, ox1, oz1, 0, L, C_Z)
ax.text(ox1+L+6, oz1,     r"$x_1$", fontsize=10, color=C_X, va="center")
ax.text(ox1+4,   oz1+L+6, r"$z_1$", fontsize=10, color=C_Z)
ax.text(ox1+10,  oz1-32,  r"$\{1\}$", fontsize=11, fontweight="bold", ha="center")

# ── Frame {2} — co-located with {1}, z₂ = out of page ────────────────────────
ox2, oz2 = 30, 565
arrow(ax, ox2, oz2, L, 0, C_X)
z_dot(ax, ox2+12, oz2+56)
ax.text(ox2+L+6, oz2,    r"$x_2$", fontsize=10, color=C_X, va="center")
ax.text(ox2+30,  oz2+72, r"$z_2$", fontsize=10, color=C_Z)
ax.text(ox2+55,  oz2+93, r"$\{2\}$", fontsize=11, fontweight="bold", ha="center")

ax.annotate(
    r"$\alpha_1 = -90°$" + "\n" + r"($z_1 \rightarrow z_2$)",
    xy=(-20, 548), xytext=(-265, 700),
    fontsize=10, color=C_ALPHA,
    arrowprops=dict(arrowstyle="->", color=C_ALPHA, lw=1.5),
    bbox=dict(boxstyle="round,pad=0.35", fc="#fdf0f0", ec=C_ALPHA, alpha=0.95),
    zorder=9,
)

# ── Frame {3} — above elbow ───────────────────────────────────────────────────
ox3, oz3 = J3[0]+10, J3[1]+55
arrow(ax, ox3, oz3, L, 0, C_X)
z_dot(ax, ox3+12, oz3+56)
ax.text(ox3+L+6, oz3,    r"$x_3$", fontsize=10, color=C_X, va="center")
ax.text(ox3+30,  oz3+72, r"$z_3$", fontsize=10, color=C_Z)
ax.text(ox3+55,  oz3+93, r"$\{3\}$", fontsize=11, fontweight="bold", ha="center")

# ── Frame {4} — above wrist ───────────────────────────────────────────────────
ox4, oz4 = J4[0]+10, J4[1]+48
arrow(ax, ox4, oz4, L, 0, C_X)
z_dot(ax, ox4+12, oz4+54)
ax.text(ox4+L+6, oz4,    r"$x_4$", fontsize=10, color=C_X, va="center")
ax.text(ox4+30,  oz4+68, r"$z_4$", fontsize=10, color=C_Z)
ax.text(ox4+55,  oz4+88, r"$\{4\}$", fontsize=11, fontweight="bold", ha="center")

# ── θ arcs ────────────────────────────────────────────────────────────────────
# θ₁: yaw about z₀ — shown as dashed ellipse (horizontal rotation)
ax.add_patch(Arc(O, 78, 28, angle=0, theta1=10, theta2=170,
                 color=C_THETA, lw=2, ls="--", zorder=6))
ax.text(O[0], O[1]+30, r"$\theta_1$", fontsize=10, color=C_THETA, ha="center")
ax.text(O[0], O[1]+50, "(yaw)",        fontsize=8.5, color=C_THETA, ha="center")

# θ₂, θ₃, θ₄: pitch arcs in XZ plane
for pos, label, r in [(J1, "2", 52), (J3, "3", 42), (J4, "4", 32)]:
    ax.add_patch(Arc(pos, r*2, r*2, angle=0, theta1=0, theta2=52,
                     color=C_THETA, lw=2.2, zorder=6))
    end = math.radians(52)
    ax.text(pos[0] + r*math.cos(end) + 8,
            pos[1] + r*math.sin(end) + 4,
            f"$\\theta_{label}$", fontsize=10, color=C_THETA)

# ── dimension lines ───────────────────────────────────────────────────────────
def dim(ax, x1, z1, x2, z2, label, lx=None, lz=None, color=C_DIM, fs=10):
    ax.annotate("", xy=(x2, z2), xytext=(x1, z1),
                arrowprops=dict(arrowstyle="<->", color=color, lw=1.8), zorder=6)
    mx = lx if lx is not None else (x1+x2)/2
    mz = lz if lz is not None else (z1+z2)/2
    ax.text(mx, mz, label, fontsize=fs, color=color,
            ha="center", va="center",
            bbox=dict(fc="white", ec="none", pad=1.5))

def leader(ax, x1, x2, z):
    ax.plot([x1, x2], [z, z], color="#aaa", lw=0.9, ls=":")

def vleader(ax, x, z1, z2):
    ax.plot([x, x], [z1, z2], color="#aaa", lw=0.9, ls=":")

# d₁ = 500 (vertical)
vleader(ax, -290, 0,   0)
vleader(ax, -290, 500, 500)
ax.plot([-290, -30], [0,   0],   color="#aaa", lw=0.9, ls=":")
ax.plot([-290, -30], [500, 500], color="#aaa", lw=0.9, ls=":")
dim(ax, -290, 0, -290, 500, "$d_1 = 500$ mm", lx=-365, lz=250)

# a₂ = 700 (horizontal)
ax.plot([0,   0],   [390, J1[1]], color="#aaa", lw=0.9, ls=":")
ax.plot([700, 700], [390, J3[1]], color="#aaa", lw=0.9, ls=":")
dim(ax, 0, 390, 700, 390, "$a_2 = 700$ mm", lz=358)

# a₃ = 500 (horizontal)
ax.plot([700,  700],  [390, J3[1]], color="#aaa", lw=0.9, ls=":")
ax.plot([1200, 1200], [390, J4[1]], color="#aaa", lw=0.9, ls=":")
dim(ax, 700, 390, 1200, 390, "$a_3 = 500$ mm", lz=358)

# nozzle offset = 200
ax.plot([1200, 1200], [462, J4[1]+10], color="#8899bb", lw=0.9, ls=":")
ax.plot([1400, 1400], [462, JN[1]+10], color="#8899bb", lw=0.9, ls=":")
dim(ax, 1200, 462, 1400, 462, "200 mm", lz=484, color="#1a6fba", fs=9)
ax.text(1300, 510, "nozzle offset", fontsize=8.5, color="#1a6fba", ha="center")

# ── DH table (right panel) ────────────────────────────────────────────────────
ax_t.axis("off")
ax_t.set_title("Modified DH Parameter Table",
               fontsize=12, fontweight="bold", loc="center", pad=6)

headers = [r"$i$", r"$\alpha_{i-1}$", r"$a_{i-1}$", r"$d_i$", r"$\theta_i$"]
rows = [
    ["1", r"$0°$",   "0 mm",   "500 mm", r"$\theta_1$"],
    ["2", r"$-90°$", "0 mm",   "0 mm",   r"$\theta_2$"],
    ["3", r"$0°$",   "700 mm", "0 mm",   r"$\theta_3$"],
    ["4", r"$0°$",   "500 mm", "0 mm",   r"$\theta_4$"],
]

tbl = ax_t.table(cellText=rows, colLabels=headers,
                 loc="upper center", cellLoc="center")
tbl.auto_set_font_size(False)
tbl.set_fontsize(11)
tbl.scale(1.12, 2.3)

for (r, c), cell in tbl.get_celld().items():
    cell.set_edgecolor("#999")
    if r == 0:
        cell.set_facecolor("#c6d8f2")
        cell.set_text_props(fontweight="bold")
    elif r % 2 == 1:
        cell.set_facecolor("#eef3fc")
    else:
        cell.set_facecolor("#ffffff")

ax_t.text(
    0.50, 0.04,
    "Joint Limits:\n"
    r"$\theta_1$: ±180°    $\theta_2$: ±90°" + "\n"
    r"$\theta_3$: $-60°$ to $+120°$    $\theta_4$: ±90°",
    transform=ax_t.transAxes, ha="center", va="bottom", fontsize=10,
    bbox=dict(boxstyle="round,pad=0.4", fc="#f4f6fb", ec="#aaa"),
)

# ── save ──────────────────────────────────────────────────────────────────────
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dh_parameters.png")
fig.savefig(out, dpi=150, bbox_inches="tight", facecolor="white")
plt.show()
print(f"Saved: {out}")

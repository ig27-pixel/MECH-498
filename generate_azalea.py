"""
Generates azalea.yaml — a colorful azalea flower drawing.

Color mapping (4 brushes):
  1 = pink/magenta  -> outer petals
  2 = deep rose     -> petal base markings / inner petal fill
  3 = yellow        -> stamen cluster
  4 = green         -> leaves
"""

import numpy as np
import yaml

# ---------------------------------------------------------------------------
# Drawing canvas — stay comfortably inside the robot workspace
# ---------------------------------------------------------------------------
CX   = 1500.0   # flower center X (mm)
CY   = -400.0   # flower center Y (mm)
CZ   =    0.0   # flat drawing plane Z (mm)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def transition(xs, ys, zs, cs,
               x0, y0, x1, y1, z=CZ, n=12, color=1):
    """Glide from (x0,y0) to (x1,y1) routing through the flower center
    so transition strokes are hidden under other strokes."""
    mid_x, mid_y = CX, CY
    for seg_start, seg_end in [((x0, y0), (mid_x, mid_y)), ((mid_x, mid_y), (x1, y1))]:
        sx, sy = seg_start
        ex, ey = seg_end
        for k in range(1, n + 1):
            a = k / n
            xs.append(sx + a * (ex - sx))
            ys.append(sy + a * (ey - sy))
            zs.append(z)
            cs.append(color)


def ellipse_pts(ecx, ecy, semi_a, semi_b, tilt, n=80):
    """Parametric ellipse centred at (ecx,ecy), semi-axes a/b, rotated by tilt."""
    t  = np.linspace(0, 2 * np.pi, n, endpoint=True)
    xl = semi_a * np.cos(t)
    yl = semi_b * np.sin(t)
    ca, sa = np.cos(tilt), np.sin(tilt)
    return ecx + ca * xl - sa * yl, ecy + sa * xl + ca * yl


def petal_pts(ecx, ecy, length, width, tilt, n=80):
    """
    A teardrop / petal shape: broad base, pointed tip.
    Parametrised so the tip points in the tilt direction away from the center.
    """
    t = np.linspace(0, 2 * np.pi, n, endpoint=True)
    # local: x along petal axis, y across
    xl = length * 0.5 * (np.cos(t) + 0.3 * np.cos(2 * t) - 0.3)
    yl = width  * 0.5 *  np.sin(t)
    ca, sa = np.cos(tilt), np.sin(tilt)
    return ecx + ca * xl - sa * yl, ecy + sa * xl + ca * yl


# ---------------------------------------------------------------------------
# Accumulator
# ---------------------------------------------------------------------------
xs, ys, zs, cs = [], [], [], []

def append_stroke(px, py, color, z=CZ):
    for xi, yi in zip(px, py):
        xs.append(float(xi))
        ys.append(float(yi))
        zs.append(float(z))
        cs.append(int(color))


# ---------------------------------------------------------------------------
# 1. OUTER PETALS  (color 1)
# ---------------------------------------------------------------------------
N_PETALS     = 5
PETAL_DIST   = 60      # center-of-ellipse offset from flower center (mm)
PETAL_LEN    = 200     # semi-major (radial) length of petal (mm)
PETAL_WID    = 100     # semi-minor (tangential) width (mm)

petal_starts = []  # (x,y) of first point of each petal, for transition use

for i in range(N_PETALS):
    angle = i * 2 * np.pi / N_PETALS - np.pi / 2   # start at top
    pcx   = CX + PETAL_DIST * np.cos(angle)
    pcy   = CY + PETAL_DIST * np.sin(angle)
    px, py = petal_pts(pcx, pcy, PETAL_LEN, PETAL_WID, angle)

    if not xs:
        append_stroke(px, py, 1)
    else:
        transition(xs, ys, zs, cs,
                   xs[-1], ys[-1], px[0], py[0], color=1)
        append_stroke(px, py, 1)

    petal_starts.append((px[0], py[0]))


# ---------------------------------------------------------------------------
# 2. INNER PETAL BASE MARKINGS  (color 2)
#    Short filled ellipses near the petal base, creating the azalea's
#    distinctive dark-throated look.
# ---------------------------------------------------------------------------
INNER_LEN = 70
INNER_WID = 45

for i in range(N_PETALS):
    angle = i * 2 * np.pi / N_PETALS - np.pi / 2
    # place just inside the petal, close to the center
    pcx = CX + (PETAL_DIST + 20) * np.cos(angle)
    pcy = CY + (PETAL_DIST + 20) * np.sin(angle)
    px, py = ellipse_pts(pcx, pcy, INNER_LEN, INNER_WID, angle, n=50)

    transition(xs, ys, zs, cs,
               xs[-1], ys[-1], px[0], py[0], color=2)
    append_stroke(px, py, 2)


# ---------------------------------------------------------------------------
# 3. STAMEN CLUSTER  (color 3)
#    A tight circle + 8 short radial "filaments" from the center.
# ---------------------------------------------------------------------------
STAMEN_R   = 28   # ring radius (mm)
STAMEN_LEN = 55   # filament length (mm)
N_STAMEN   = 8

# central ring
px, py = ellipse_pts(CX, CY, STAMEN_R, STAMEN_R, 0, n=50)
transition(xs, ys, zs, cs, xs[-1], ys[-1], px[0], py[0], color=3)
append_stroke(px, py, 3)

# filaments
for i in range(N_STAMEN):
    a  = i * 2 * np.pi / N_STAMEN
    x0 = CX + STAMEN_R  * np.cos(a)
    y0 = CY + STAMEN_R  * np.sin(a)
    x1 = CX + STAMEN_LEN * np.cos(a)
    y1 = CY + STAMEN_LEN * np.sin(a)

    transition(xs, ys, zs, cs, xs[-1], ys[-1], x0, y0, color=3)
    t_line = np.linspace(0, 1, 15)
    for t in t_line:
        xs.append(float(x0 + t * (x1 - x0)))
        ys.append(float(y0 + t * (y1 - y0)))
        zs.append(CZ)
        cs.append(3)

    # dot at the anther tip (small circle)
    px, py = ellipse_pts(x1, y1, 8, 8, 0, n=20)
    append_stroke(px, py, 3)


# ---------------------------------------------------------------------------
# 4. LEAVES  (color 4)
#    Two leaves below the flower, angled outward.
# ---------------------------------------------------------------------------
leaf_defs = [
    # (offset_x, offset_y, length, width, tilt)
    ( -90,  210, 170, 60,  np.radians(110)),   # left leaf
    (  90,  210, 170, 60,  np.radians( 70)),   # right leaf
    (   0,  240, 140, 50,  np.radians( 90)),   # centre leaf
]

for (dx, dy, llen, lwid, ltilt) in leaf_defs:
    lcx = CX + dx
    lcy = CY + dy
    px, py = petal_pts(lcx, lcy, llen, lwid, ltilt, n=60)

    transition(xs, ys, zs, cs, xs[-1], ys[-1], px[0], py[0], color=4)
    append_stroke(px, py, 4)

    # midrib line down the leaf
    tip_x  = lcx + llen * 0.5 * np.cos(ltilt)
    tip_y  = lcy + llen * 0.5 * np.sin(ltilt)
    base_x = lcx - llen * 0.4 * np.cos(ltilt)
    base_y = lcy - llen * 0.4 * np.sin(ltilt)
    transition(xs, ys, zs, cs, xs[-1], ys[-1], base_x, base_y, color=4)
    t_line = np.linspace(0, 1, 25)
    for t in t_line:
        xs.append(float(base_x + t * (tip_x - base_x)))
        ys.append(float(base_y + t * (tip_y - base_y)))
        zs.append(CZ)
        cs.append(4)


# ---------------------------------------------------------------------------
# Serialise
# ---------------------------------------------------------------------------
data = {
    'x':     [float(v) for v in xs],
    'y':     [float(v) for v in ys],
    'z':     [float(v) for v in zs],
    'color': [int(v)   for v in cs],
}

out_path = 'azalea.yaml'
with open(out_path, 'w') as f:
    yaml.dump(data, f, default_flow_style=False)

print(f"Generated {len(xs)} waypoints -> {out_path}")
print(f"  X range : {min(xs):.1f}  {max(xs):.1f}")
print(f"  Y range : {min(ys):.1f}  {max(ys):.1f}")
print(f"  Z       : {CZ}")
print(f"  Colors  : {sorted(set(cs))}")

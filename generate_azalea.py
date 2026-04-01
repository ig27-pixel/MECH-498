"""
Generates azalea.yaml — a bouquet of 3 azalea flowers for Robo Picasso.

Color mapping:
  1 (purple) : outer petals
  2 (gold)   : inner petal base markings
  3 (orange) : stamen
  4 (green)  : stems + leaves
"""

import numpy as np
import yaml

# ---------------------------------------------------------------------------
# Robot workspace
# ---------------------------------------------------------------------------
CX  = 1510.0   # bouquet center X (mm)
CY  = -390.0   # bouquet center Y (mm)
CZ  =    0.0

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def ellipse_pts(ecx, ecy, semi_a, semi_b, tilt, n=80):
    t  = np.linspace(0, 2 * np.pi, n, endpoint=True)
    xl = semi_a * np.cos(t)
    yl = semi_b * np.sin(t)
    ca, sa = np.cos(tilt), np.sin(tilt)
    return ecx + ca * xl - sa * yl, ecy + sa * xl + ca * yl

def petal_pts(ecx, ecy, length, width, tilt, n=80):
    t  = np.linspace(0, 2 * np.pi, n, endpoint=True)
    xl = length * 0.5 * (np.cos(t) + 0.3 * np.cos(2 * t) - 0.3)
    yl = width  * 0.5 *  np.sin(t)
    ca, sa = np.cos(tilt), np.sin(tilt)
    return ecx + ca * xl - sa * yl, ecy + sa * xl + ca * yl

xs, ys, zs, cs = [], [], [], []

def append_stroke(px, py, color):
    for xi, yi in zip(px, py):
        xs.append(float(xi)); ys.append(float(yi))
        zs.append(CZ);        cs.append(int(color))

def move_to(tx, ty, color, n=14):
    if not xs:
        return
    x0, y0 = xs[-1], ys[-1]
    for k in range(1, n + 1):
        a = k / n
        xs.append(x0 + a * (tx - x0))
        ys.append(y0 + a * (ty - y0))
        zs.append(CZ); cs.append(int(color))

# ---------------------------------------------------------------------------
# Draw one flower head at (fcx, fcy) with given scale
# ---------------------------------------------------------------------------
def draw_flower(fcx, fcy, scale=1.0):
    N_PETALS   = 5
    PETAL_DIST = 36  * scale
    PETAL_LEN  = 122 * scale
    PETAL_WID  = 61  * scale
    INNER_LEN  = 43  * scale
    INNER_WID  = 27  * scale
    STAMEN_R   = 17  * scale
    STAMEN_LEN = 34  * scale

    # Outer petals (color 1)
    for i in range(N_PETALS):
        angle = i * 2 * np.pi / N_PETALS - np.pi / 2
        pcx = fcx + PETAL_DIST * np.cos(angle)
        pcy = fcy + PETAL_DIST * np.sin(angle)
        px, py = petal_pts(pcx, pcy, PETAL_LEN, PETAL_WID, angle)
        if not xs:
            append_stroke(px, py, 1)
        else:
            move_to(px[0], py[0], 1)
            append_stroke(px, py, 1)

    # Inner petal markings (color 2)
    for i in range(N_PETALS):
        angle = i * 2 * np.pi / N_PETALS - np.pi / 2
        pcx = fcx + (PETAL_DIST + 16 * scale) * np.cos(angle)
        pcy = fcy + (PETAL_DIST + 16 * scale) * np.sin(angle)
        px, py = ellipse_pts(pcx, pcy, INNER_LEN, INNER_WID, angle, n=50)
        move_to(px[0], py[0], 2)
        append_stroke(px, py, 2)

    # Stamen ring (color 3)
    px, py = ellipse_pts(fcx, fcy, STAMEN_R, STAMEN_R, 0, n=40)
    move_to(px[0], py[0], 3)
    append_stroke(px, py, 3)

    # Stamen filaments (color 3)
    for i in range(8):
        a  = i * 2 * np.pi / 8
        x0 = fcx + STAMEN_R   * np.cos(a)
        y0 = fcy + STAMEN_R   * np.sin(a)
        x1 = fcx + STAMEN_LEN * np.cos(a)
        y1 = fcy + STAMEN_LEN * np.sin(a)
        move_to(x0, y0, 3)
        t_line = np.linspace(0, 1, 12)
        for t in t_line:
            xs.append(float(x0 + t * (x1 - x0)))
            ys.append(float(y0 + t * (y1 - y0)))
            zs.append(CZ); cs.append(3)
        anth_px, anth_py = ellipse_pts(x1, y1, 6 * scale, 6 * scale, 0, n=16)
        append_stroke(anth_px, anth_py, 3)

# ---------------------------------------------------------------------------
# Bouquet flower centers (triangle arrangement)
# ---------------------------------------------------------------------------
#   top-center, bottom-left, bottom-right
flower_centers = [
    (CX,        CY - 110),   # top
    (CX - 110,  CY +  70),   # bottom-left
    (CX + 110,  CY +  70),   # bottom-right
]

SCALE = 1.0   # each flower at full individual size

for (fcx, fcy) in flower_centers:
    draw_flower(fcx, fcy, scale=SCALE)

# ---------------------------------------------------------------------------
# Stems — three lines converging to a common grip point  (color 4)
# ---------------------------------------------------------------------------
grip_x = CX
grip_y = CY + 230

for (fcx, fcy) in flower_centers:
    # stem base: bottom of flower head
    stem_top_x = fcx
    stem_top_y = fcy + 36 * SCALE   # just below flower center

    move_to(stem_top_x, stem_top_y, 4)
    t_line = np.linspace(0, 1, 30)
    for t in t_line:
        xs.append(float(stem_top_x + t * (grip_x - stem_top_x)))
        ys.append(float(stem_top_y + t * (grip_y - stem_top_y)))
        zs.append(CZ); cs.append(4)

# ---------------------------------------------------------------------------
# Two leaves along the outer stems (color 4)
# ---------------------------------------------------------------------------
for side, (fcx, fcy) in [(-1, flower_centers[1]), (1, flower_centers[2])]:
    # midpoint along the stem
    stem_top_x = fcx
    stem_top_y = fcy + 36 * SCALE
    mid_x = (stem_top_x + grip_x) * 0.5
    mid_y = (stem_top_y + grip_y) * 0.5
    leaf_tilt = np.radians(90 + side * 40)
    px, py = petal_pts(mid_x + side * 30, mid_y, 90, 35, leaf_tilt, n=50)
    move_to(px[0], py[0], 4)
    append_stroke(px, py, 4)

# ---------------------------------------------------------------------------
# Serialise
# ---------------------------------------------------------------------------
data = {
    'x':     [float(v) for v in xs],
    'y':     [float(v) for v in ys],
    'z':     [float(v) for v in zs],
    'color': [int(v)   for v in cs],
}

with open('azalea.yaml', 'w') as f:
    yaml.dump(data, f, default_flow_style=False)

print(f"Generated {len(xs)} waypoints -> azalea.yaml")
print(f"  X range : {min(xs):.1f}  {max(xs):.1f}")
print(f"  Y range : {min(ys):.1f}  {max(ys):.1f}")
print(f"  Z       : {CZ}")
print(f"  Colors  : {sorted(set(cs))}")

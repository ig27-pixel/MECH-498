"""
Generates rice_r.yaml — The Rice University block 'R' for Robo Picasso.

Color mapping:
  1 (purple) : decorative ring around the R
  2 (gold)   : serifs and accents
  3 (orange) : bowl and leg fill strokes
  4 (blue)   : main R outline  (Rice blue)
"""

import numpy as np
import yaml

# ---------------------------------------------------------------------------
# Robot workspace center
# ---------------------------------------------------------------------------
CX = 1500.0
CY = -310.0
CZ = 0.0

# ---------------------------------------------------------------------------
# R geometry  (local coords: origin = bottom-left of R, x right, y up)
# ---------------------------------------------------------------------------
H   = 240.0   # total height of R
W   = 165.0   # total width  (tip of leg)
SW  = 40.0    # stem width
BH  = 118.0   # height where bowl ends / leg starts
BW  = 88.0    # max bowl extension rightward from stem
iBW = 50.0    # inner bowl arc radius (creates the "hole" in the bowl)

# ---------------------------------------------------------------------------
# Accumulators
# ---------------------------------------------------------------------------
xs, ys, zs, cs = [], [], [], []

def to_robot(lx, ly):
    return CX - W / 2 + lx, CY + H / 2 - ly

def add(lx, ly, c):
    rx, ry = to_robot(lx, ly)
    xs.append(float(rx)); ys.append(float(ry))
    zs.append(CZ);        cs.append(int(c))

def add_arr(lxs, lys, c):
    for lx, ly in zip(lxs, lys):
        add(lx, ly, c)

def move_to(lx, ly, c, n=14):
    """Straight-line transition in robot space using color c."""
    if not xs:
        return
    rx, ry = to_robot(lx, ly)
    x0, y0 = xs[-1], ys[-1]
    for k in range(1, n + 1):
        a = k / n
        xs.append(x0 + a * (rx - x0))
        ys.append(y0 + a * (ry - y0))
        zs.append(CZ)
        cs.append(c)

# ---------------------------------------------------------------------------
# Bowl helpers
# ---------------------------------------------------------------------------
bowl_cy = (H + BH) / 2          # vertical centre of the bowl ellipse
bowl_ry = (H - BH) / 2          # vertical radius of the bowl ellipse

# Outer bowl arc:  top of stem → right → bowl junction  (top-to-bottom)
theta_out = np.linspace(np.pi / 2, -np.pi / 2, 80)
outer_arc_lx = SW + BW  * np.cos(theta_out)
outer_arc_ly = bowl_cy  + bowl_ry * np.sin(theta_out)

# Inner bowl arc:  bowl junction → right → top of stem  (bottom-to-top)
theta_in = np.linspace(-np.pi / 2, np.pi / 2, 60)
inner_arc_lx = SW + iBW * np.cos(theta_in)
inner_arc_ly = bowl_cy  + bowl_ry * np.sin(theta_in)

# ---------------------------------------------------------------------------
# 1.  MAIN R OUTLINE  (color 4 – blue)
# ---------------------------------------------------------------------------

# Left edge of stem: bottom → top
add_arr(np.zeros(50), np.linspace(0, H, 50), 4)

# Top of stem: left → right
add_arr(np.linspace(0, SW, 12), np.full(12, H), 4)

# Outer bowl arc: top of stem → (rightward) → bowl junction
add_arr(outer_arc_lx, outer_arc_ly, 4)

# Leg: bowl junction → bottom-right tip
t = np.linspace(0, 1, 50)
add_arr(SW + t * (W - SW), BH * (1 - t), 4)

# Bottom edge: right → left
add_arr(np.linspace(W, 0, 30), np.zeros(30), 4)

# Inner bowl arc: bowl junction → (rightward) → top of stem
move_to(inner_arc_lx[0], inner_arc_ly[0], 4)
add_arr(inner_arc_lx, inner_arc_ly, 4)

# Right edge of stem: bottom → top  (closes the stem on the right side)
move_to(SW, 0, 4)
add_arr(np.full(40, SW), np.linspace(0, H, 40), 4)

# ---------------------------------------------------------------------------
# 2.  BOWL + LEG FILL STROKES  (color 3 – orange)
# ---------------------------------------------------------------------------

# Parallel arcs to fill the bowl wall
for scale in [0.62, 0.77, 0.92]:
    th = np.linspace(np.pi / 2, -np.pi / 2, 55)
    flx = SW + BW * scale * np.cos(th)
    fly = bowl_cy + bowl_ry * np.sin(th)
    move_to(flx[0], fly[0], 3)
    add_arr(flx, fly, 3)

# Parallel strokes to fill the stem interior
for i in range(3):
    lx = SW * (i + 1) / 4
    lys = np.linspace(8, H - 8, 40)
    lxs = np.full(40, lx)
    if i % 2 == 1:
        lys = lys[::-1]
    move_to(lxs[0], lys[0], 3)
    add_arr(lxs, lys, 3)

# Parallel strokes to fill the leg
for frac in [0.25, 0.55, 0.80]:
    perp_offset = frac * 14 - 7   # offset perpendicular to leg direction
    dx, dy = W - SW, -BH
    length = np.sqrt(dx ** 2 + dy ** 2)
    px, py = -dy / length * perp_offset, dx / length * perp_offset
    t = np.linspace(0.05, 0.95, 35)
    lxs = SW + t * (W - SW) + px
    lys = BH * (1 - t) + py
    move_to(lxs[0], lys[0], 3)
    add_arr(lxs, lys, 3)

# ---------------------------------------------------------------------------
# 3.  SERIFS  (color 2 – gold)
# ---------------------------------------------------------------------------
SERIF = 20

def draw_serif(cx, cy, half_len, vertical=False):
    if vertical:
        lxs = np.full(20, cx)
        lys = np.linspace(cy - half_len, cy + half_len, 20)
    else:
        lxs = np.linspace(cx - half_len, cx + half_len, 20)
        lys = np.full(20, cy)
    move_to(lxs[0], lys[0], 2)
    add_arr(lxs, lys, 2)

draw_serif(SW / 2, 0,  SERIF)           # bottom of stem
draw_serif(SW / 2, H,  SERIF)           # top of stem
draw_serif(W - 12, 4,  SERIF * 0.75)    # tip of leg

# Short vertical tick at the bowl junction on the stem right edge
draw_serif(SW, BH, SERIF * 0.5, vertical=True)

# ---------------------------------------------------------------------------
# 4.  DECORATIVE RING  (color 1 – purple)
# ---------------------------------------------------------------------------
ring_rx = W  * 0.70
ring_ry = H  * 0.60
theta_ring = np.linspace(0, 2 * np.pi, 200)
ring_lx = W / 2 + ring_rx * np.cos(theta_ring)
ring_ly = H / 2 + ring_ry * np.sin(theta_ring)
move_to(ring_lx[0], ring_ly[0], 1)
add_arr(ring_lx, ring_ly, 1)

# Second, slightly smaller ring for a double-ring look
ring_rx2 = ring_rx * 0.87
ring_ry2 = ring_ry * 0.87
ring_lx2 = W / 2 + ring_rx2 * np.cos(theta_ring)
ring_ly2 = H / 2 + ring_ry2 * np.sin(theta_ring)
move_to(ring_lx2[0], ring_ly2[0], 1)
add_arr(ring_lx2, ring_ly2, 1)

# ---------------------------------------------------------------------------
# Serialise
# ---------------------------------------------------------------------------
data = {
    'x':     [float(v) for v in xs],
    'y':     [float(v) for v in ys],
    'z':     [float(v) for v in zs],
    'color': [int(v)   for v in cs],
}

out_path = 'rice_r.yaml'
with open(out_path, 'w') as f:
    yaml.dump(data, f, default_flow_style=False)

print(f"Generated {len(xs)} waypoints -> {out_path}")
print(f"  X range : {min(xs):.1f}  {max(xs):.1f}")
print(f"  Y range : {min(ys):.1f}  {max(ys):.1f}")
print(f"  Z       : {CZ}")
print(f"  Colors  : {sorted(set(cs))}")

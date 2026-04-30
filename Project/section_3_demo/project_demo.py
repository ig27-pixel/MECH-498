"""project_demo.py — Section 3: RoboRoll Coatings demonstration.

Wall 1 (X = +900 mm): a smiley face (outline, eyes, smile).
Wall 2 (Y = +900 mm): five horizontal colour-sweep stripes.

Run from the repository root:
    python Project/section_3_demo/project_demo.py
"""

import math
import os
import sys
import time
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(PROJECT_DIR, "section_1_2_kinematics"))
sys.path.append(os.path.join(PROJECT_DIR, "shared"))

from RoboRoll import RoboRoll
from robot_components import Brush

# ── display offset: robot base is shifted -200 mm in X so it sits inside the room
_DISPLAY_OFFSET = np.array([-200.0, 0.0, 0.0])


# ── helpers ──────────────────────────────────────────────────────────────────

def _yaw_transform(theta: float) -> np.ndarray:
    """4×4 pure yaw rotation for visual EE frame tweaks."""
    c, s = np.cos(theta), np.sin(theta)
    T = np.eye(4)
    T[0, 0], T[0, 1] = c, -s
    T[1, 0], T[1, 1] = s,  c
    return T


def _nozzle_room_to_ee_fk(room_x: float, room_y: float, room_z: float) -> np.ndarray:
    """Return an EE FK target frame whose nozzle tip lands at room (x, y, z).

    The nozzle sits 200 mm ahead of the EE in the EE x-direction.
    The orientation uses wrist_yaw = 0 (home wrist pose) so IK uniquely
    determines θ1 from the horizontal bearing to the target.

    Args:
        room_x, room_y, room_z: desired nozzle position in room coordinates [mm].

    Returns:
        np.ndarray: 4×4 EE target frame in FK (robot-base) coordinates.
    """
    # Convert room → FK by undoing the display offset
    fk_nx = room_x - _DISPLAY_OFFSET[0]   # = room_x + 200
    fk_ny = room_y
    fk_nz = room_z

    # θ1 = horizontal bearing from FK origin to the nozzle point
    t1 = math.atan2(fk_ny, fk_nx)
    ct, st = math.cos(t1), math.sin(t1)

    # EE sits 200 mm behind the nozzle along the θ1 direction
    fk_ex = fk_nx - 200.0 * ct
    fk_ey = fk_ny - 200.0 * st
    fk_ez = fk_nz

    # EE rotation: Rz(t1) @ Rx(-π/2)  →  wrist_yaw = 0
    #   R = [[ct,  0, -st],
    #        [st,  0,  ct],
    #        [0,  -1,   0]]
    T = np.array([[ ct,   0.0, -st,  fk_ex],
                  [ st,   0.0,  ct,  fk_ey],
                  [0.0,  -1.0, 0.0,  fk_ez],
                  [0.0,   0.0, 0.0,   1.0 ]])
    return T


# ── main demo class ───────────────────────────────────────────────────────────

class RoboRollRoomDemo(RoboRoll):
    """Two-wall painting demonstration for RoboRoll Coatings."""

    ROOM_HALF = 1000.0   # room extends ±1000 mm in X and Y, 0–1000 mm in Z

    # Wall 1 (X = 900): painting panel bounds
    W1_X  = 900.0
    W1_Y0, W1_Y1 = -240.0, 240.0
    W1_Z0, W1_Z1 =  310.0, 750.0

    # Wall 2 (Y = 900): painting panel bounds
    W2_Y  = 900.0
    W2_X0, W2_X1 = -240.0, 240.0
    W2_Z0, W2_Z1 =  310.0, 750.0

    def __init__(self, drawing_enabled: bool = True,
                 frame_delay: float = 0.005,
                 samples_per_segment: int = 7):
        super().__init__(drawing_enabled=drawing_enabled)
        self.brush = Brush(self.ax, drawing_enabled=drawing_enabled)
        self.display_offset = _DISPLAY_OFFSET.copy()
        self.frame_delay = max(0.0, float(frame_delay))
        self.samples_per_segment = max(2, int(samples_per_segment))
        self._ee_marker = None

        if self._drawing_enabled:
            self._configure_room_view()
            self._draw_room()

    # ── view / room ──────────────────────────────────────────────────────────

    def _configure_room_view(self) -> None:
        if not self._drawing_enabled or self.ax is None:
            return
        r = self.ROOM_HALF
        self.ax.set_xlim([-r, r]);  self.ax.set_ylim([-r, r])
        self.ax.set_zlim([0.0, r])
        self.ax.set_xlabel("X (mm)", fontsize=12)
        self.ax.set_ylabel("Y (mm)", fontsize=12)
        self.ax.set_zlabel("Z (mm)", fontsize=12)
        self.ax.set_title("RoboRoll Coatings — Demo", fontsize=14)
        self.ax.view_init(elev=22, azim=42)
        try:
            self.ax.set_box_aspect((2.0, 2.0, 1.0))
        except Exception:
            pass

    def _draw_room(self) -> None:
        if not self._drawing_enabled or self.ax is None:
            return
        r = self.ROOM_HALF
        corners = np.array([[-r,-r,0],[r,-r,0],[r,r,0],[-r,r,0],
                             [-r,-r,r],[r,-r,r],[r,r,r],[-r,r,r]])
        edges = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),
                 (0,4),(1,5),(2,6),(3,7)]
        for a, b in edges:
            self.ax.plot(*zip(corners[a], corners[b]),
                         color="#aaaaaa", linewidth=0.9, alpha=0.8)

        # Wall 1 panel (X = 900)
        yw = np.array([self.W1_Y0, self.W1_Y1])
        zw = np.array([self.W1_Z0, self.W1_Z1])
        YY, ZZ = np.meshgrid(yw, zw)
        XX = np.full_like(YY, self.W1_X)
        self.ax.plot_surface(XX, YY, ZZ, color="#c8d8ee", alpha=0.22, shade=False)

        # Wall 2 panel (Y = 900)
        xw = np.array([self.W2_X0, self.W2_X1])
        zw2 = np.array([self.W2_Z0, self.W2_Z1])
        XX2, ZZ2 = np.meshgrid(xw, zw2)
        YY2 = np.full_like(XX2, self.W2_Y)
        self.ax.plot_surface(XX2, YY2, ZZ2, color="#c8eec8", alpha=0.22, shade=False)

        # Wall labels
        self.ax.text(self.W1_X, 0, self.W1_Z1 + 40,
                     "Wall 1", color="#2255aa", fontsize=9, ha="center")
        self.ax.text(0, self.W2_Y, self.W2_Z1 + 40,
                     "Wall 2", color="#227722", fontsize=9, ha="center")

    # ── drawing ──────────────────────────────────────────────────────────────

    def _shift_frame(self, frame: np.ndarray) -> np.ndarray:
        s = frame.copy()
        s[:3, 3] += self.display_offset
        return s

    def draw_demo_pose(self, joint_angles: np.ndarray, color: int = 0,
                       leave_paint: bool = False,
                       wrist_visual_angle: float = 0.0) -> None:
        if not self._drawing_enabled:
            return
        self.brush.selection = int(color)
        self.calculate_fk(joint_angles)

        if not self._drawn_once:
            plt.show(block=False)
            plt.pause(0.5)
            self._drawn_once = True

        base_frame = np.eye(4)
        base_frame[:3, 3] = self.display_offset
        cur = base_frame.copy()
        for idx, joint in enumerate(self.joints):
            prev = cur.copy()
            cur = cur @ joint.dh_transform
            joint.set_final_transform(cur)
            joint.draw()
            self._links[idx].update_frames(prev, cur)
            self._links[idx].draw()

        visual_ee = self._shift_frame(self.ee_frame) @ _yaw_transform(wrist_visual_angle)
        self.brush.update_tool_frame(visual_ee)
        self.brush.show_all()
        if leave_paint:
            self.brush.paint(show_all_tools=True)
        plt.pause(0.000_01)

    # ── path building ─────────────────────────────────────────────────────────

    def _interpolate_waypoints(
        self,
        waypoints: List[Tuple[np.ndarray, int, bool, float]],
    ) -> List[Tuple[np.ndarray, int, bool, float]]:
        frames: List[Tuple[np.ndarray, int, bool, float]] = []
        for (qa, ca, pa, wa), (qb, _cb, _pb, wb) in zip(waypoints[:-1], waypoints[1:]):
            for alpha in np.linspace(0.0, 1.0, self.samples_per_segment, endpoint=False):
                blend = alpha**2 * (3.0 - 2.0 * alpha)   # smoothstep
                q = qa + blend * (qb - qa)
                w = wa + blend * (wb - wa)
                frames.append((q, ca, pa, w))
        frames.append(waypoints[-1])
        return frames

    def _ik(self, room_x: float, room_y: float, room_z: float,
            prev: np.ndarray) -> np.ndarray:
        """Solve IK for a nozzle room position; returns best joint angles."""
        frame = _nozzle_room_to_ee_fk(room_x, room_y, room_z)
        ok, q = self.calculate_ik(frame, prev)
        return q.copy() if ok else prev.copy()

    def build_demo_path(self) -> List[Tuple[np.ndarray, int, bool, float]]:
        """Build the full two-wall painting path.

        Wall 2 — five horizontal colour stripes (top → bottom).
        Wall 1 — smiley face (outline, eyes, smile arc).
        """
        home = np.zeros(4, dtype=float)
        prev = home.copy()
        wp: List[Tuple[np.ndarray, int, bool, float]] = []

        def lift(q):
            return (q.copy(), 0, False, 0.0)

        def paint(q, col):
            return (q.copy(), col, True, 0.0)

        def ik(rx, ry, rz):
            nonlocal prev
            q = self._ik(rx, ry, rz, prev)
            prev = q
            return q

        # ── start at home ────────────────────────────────────────────────────
        wp.append(lift(home))

        # ── Wall 2: five horizontal stripes ──────────────────────────────────
        stripe_z      = [730, 625, 520, 415, 320]
        stripe_colors = [  1,   2,   3,   4,   2]
        stripe_xs     = np.linspace(self.W2_X0 + 20, self.W2_X1 - 20, 7)

        for i, (sz, sc) in enumerate(zip(stripe_z, stripe_colors)):
            xs = stripe_xs if (i % 2 == 0) else stripe_xs[::-1]

            # approach: lift above the stripe start (no paint)
            wp.append(lift(ik(xs[0], self.W2_Y, sz + 70)))
            # paint the full stripe
            for x in xs:
                wp.append(paint(ik(x, self.W2_Y, sz), sc))
            # lift after stripe end
            wp.append(lift(ik(xs[-1], self.W2_Y, sz + 70)))

        wp.append(lift(home))

        # ── Wall 1: smiley face ───────────────────────────────────────────────
        FC = (self.W1_X, 0.0, 530.0)   # face centre (room coords)
        FR = 200.0                      # face radius [mm]
        EC = 36.0                       # eye circle radius [mm]
        face_col = 3                    # orange

        # seed IK toward wall 1 before starting
        _ = ik(*FC)

        def face_transit(a_from: float, a_to: float, n: int = 5) -> None:
            """Lift-only arc along the face circle from a_from to a_to (radians)."""
            diff = (a_to - a_from + math.pi) % (2 * math.pi) - math.pi
            for a in np.linspace(a_from, a_from + diff, n + 2)[1:-1]:
                pt = (FC[0], FC[1] + FR * math.cos(a), FC[2] + FR * math.sin(a))
                wp.append(lift(ik(*pt)))

        # — face outline —
        n_out = 18
        out_angles = np.linspace(0, 2 * math.pi, n_out, endpoint=False)
        out_pts = [(FC[0], FC[1] + FR * math.cos(a), FC[2] + FR * math.sin(a))
                   for a in out_angles]

        wp.append(lift(ik(*out_pts[0])))
        for pt in out_pts:
            wp.append(paint(ik(*pt), face_col))
        wp.append(paint(ik(*out_pts[0]), face_col))   # close the circle

        # transit along face circle: outline end (0°) → left-eye entry (~138°)
        face_transit(0.0, math.radians(138))

        # — left eye —
        LE = (FC[0], FC[1] - 88.0, FC[2] + 80.0)
        le_angles = np.linspace(0, 2 * math.pi, 8, endpoint=False)
        le_pts = [(FC[0], LE[1] + EC * math.cos(a), LE[2] + EC * math.sin(a))
                  for a in le_angles]

        wp.append(lift(ik(*le_pts[0])))
        for pt in le_pts:
            wp.append(paint(ik(*pt), face_col))
        wp.append(paint(ik(*le_pts[0]), face_col))

        # transit along face circle: left-eye (~138°) → right-eye entry (~42°)
        face_transit(math.radians(138), math.radians(42))

        # — right eye —
        RE = (FC[0], FC[1] + 88.0, FC[2] + 80.0)
        re_pts = [(FC[0], RE[1] + EC * math.cos(a), RE[2] + EC * math.sin(a))
                  for a in le_angles]

        wp.append(lift(ik(*re_pts[0])))
        for pt in re_pts:
            wp.append(paint(ik(*pt), face_col))
        wp.append(paint(ik(*re_pts[0]), face_col))

        # transit along face circle: right-eye (~42°) → smile start (200°)
        face_transit(math.radians(42), math.radians(200))

        # — smile arc (200° → 340°, lifted slightly above centre) —
        smile_angles = np.linspace(math.radians(200), math.radians(340), 11)
        smile_rc = (FC[0], FC[1], FC[2] + 10.0)   # arc centre, slightly above face centre
        smile_pts = [(FC[0],
                      smile_rc[1] + 145 * math.cos(a),
                      smile_rc[2] + 145 * math.sin(a))
                     for a in smile_angles]

        wp.append(lift(ik(*smile_pts[0])))
        for pt in smile_pts:
            wp.append(paint(ik(*pt), face_col))

        return self._interpolate_waypoints(wp)

    # ── run ──────────────────────────────────────────────────────────────────

    def run_demo(self) -> None:
        path = self.build_demo_path()
        print(f"  Path built: {len(path)} rendered frames")
        for joint_angles, color, should_paint, wrist_angle in path:
            self.draw_demo_pose(joint_angles, color=color,
                                leave_paint=should_paint,
                                wrist_visual_angle=wrist_angle)
            time.sleep(self.frame_delay)


# ── entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    print("RoboRoll Coatings — Demo")
    print("  Wall 1 (X=900 mm): smiley face")
    print("  Wall 2 (Y=900 mm): five horizontal colour stripes")

    robot = RoboRollRoomDemo(
        drawing_enabled=True,
        frame_delay=0.005,
        samples_per_segment=7,
    )

    robot.run_demo()

    print("Demo complete. Close the window when finished.")
    plt.show()


if __name__ == "__main__":
    main()

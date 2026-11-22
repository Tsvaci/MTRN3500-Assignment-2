"""Simulation core: vehicle kinematics, world geometry, LiDAR ray-casting, presets.

World frame: metres, x to the right, y up. Room spans (0,0)-(ROOM,ROOM).
Vehicle pose is the rear-axle centre; heading 0 = +x, counter-clockwise positive.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

# ---- Vehicle / sensor constants (edit to match the real weeder) -------------
BODY_LEN = 1.2          # m
BODY_WID = 0.7          # m
WHEELBASE = 0.8         # m
REAR_OVERHANG = (BODY_LEN - WHEELBASE) / 2   # rear axle -> rear bumper
MAX_STEER_DEG = 40.0
MAX_SPEED = 1.0         # m/s
ACCEL = 2.0             # m/s^2, speed ramp toward command
STEER_SIGN = +1         # +1: positive steer command turns LEFT. Flip to -1 if reversed.

LIDAR_MAX = 8.0         # m; beyond this the LiDAR reports 0 (no echo)
NOISE_M = 0.005         # +/- uniform noise on each range
N_RAYS = 361            # 0..180 deg in 0.5 deg steps; 0 = right, 90 = ahead, 180 = left

ROOM = 12.0             # m, square walled room
WALL_T = 0.2            # wall thickness (outside the room)


@dataclass
class Box:
    cx: float
    cy: float
    w: float
    h: float

    def contains(self, x: float, y: float) -> bool:
        return abs(x - self.cx) <= self.w / 2 and abs(y - self.cy) <= self.h / 2

    def corners(self) -> list[tuple[float, float]]:
        hw, hh = self.w / 2, self.h / 2
        return [(self.cx - hw, self.cy - hh), (self.cx + hw, self.cy - hh),
                (self.cx + hw, self.cy + hh), (self.cx - hw, self.cy + hh)]


@dataclass
class Vehicle:
    x: float
    y: float
    heading: float
    speed: float = 0.0

    def _fwd(self) -> tuple[float, float]:
        return math.cos(self.heading), math.sin(self.heading)

    def lidar_pose(self) -> tuple[float, float, float]:
        fx, fy = self._fwd()
        d = BODY_LEN - REAR_OVERHANG
        return self.x + fx * d, self.y + fy * d, self.heading

    def centre(self) -> tuple[float, float]:
        fx, fy = self._fwd()
        d = BODY_LEN / 2 - REAR_OVERHANG
        return self.x + fx * d, self.y + fy * d

    def corners(self) -> list[tuple[float, float]]:
        fx, fy = self._fwd()
        lx, ly = -fy, fx                      # left unit vector
        cx, cy = self.centre()
        hl, hw = BODY_LEN / 2, BODY_WID / 2
        return [(cx + fx * a + lx * b, cy + fy * a + ly * b)
                for a, b in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]


class World:
    def __init__(self, walls: bool = True):
        self.walls = walls
        self.boxes: list[Box] = []

    def wall_boxes(self) -> list[Box]:
        if not self.walls:
            return []
        t, r = WALL_T, ROOM
        return [Box(r / 2, -t / 2, r + 2 * t, t), Box(r / 2, r + t / 2, r + 2 * t, t),
                Box(-t / 2, r / 2, t, r + 2 * t), Box(r + t / 2, r / 2, t, r + 2 * t)]

    def solids(self) -> list[Box]:
        return self.wall_boxes() + list(self.boxes)

    def segments(self) -> list[tuple[tuple[float, float], tuple[float, float]]]:
        segs = []
        for b in self.solids():
            c = b.corners()
            segs.extend((c[i], c[(i + 1) % 4]) for i in range(4))
        return segs


# ---- Geometry helpers -------------------------------------------------------
def _project(poly, ax, ay):
    vals = [px * ax + py * ay for px, py in poly]
    return min(vals), max(vals)


def _polys_overlap(a, b) -> bool:
    """Separating-axis test for two convex polygons."""
    for poly in (a, b):
        n = len(poly)
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            ax, ay = y1 - y2, x2 - x1
            amin, amax = _project(a, ax, ay)
            bmin, bmax = _project(b, ax, ay)
            if amax <= bmin or bmax <= amin:
                return False
    return True


def vehicle_overlaps(v: Vehicle, world: World) -> bool:
    body = v.corners()
    return any(_polys_overlap(body, b.corners()) for b in world.solids())


def clamp_cmd(steer_deg: float, speed: float) -> tuple[float, float]:
    s = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, float(steer_deg)))
    v = max(-MAX_SPEED, min(MAX_SPEED, float(speed)))
    return s, v


# ---- Motion -----------------------------------------------------------------
def step(v: Vehicle, world: World, cmd_steer_deg: float, cmd_speed: float, dt: float) -> bool:
    """Advance one step. Returns True if the move was refused (collision)."""
    steer, target = clamp_cmd(cmd_steer_deg, cmd_speed)
    dv = target - v.speed
    max_dv = ACCEL * dt
    speed = v.speed + max(-max_dv, min(max_dv, dv))

    heading = v.heading + speed * math.tan(math.radians(STEER_SIGN * steer)) / WHEELBASE * dt
    mid = (v.heading + heading) / 2
    nx = v.x + speed * math.cos(mid) * dt
    ny = v.y + speed * math.sin(mid) * dt

    was_overlapping = vehicle_overlaps(v, world)
    cand = Vehicle(nx, ny, heading, speed)
    if speed != 0.0 and not was_overlapping and vehicle_overlaps(cand, world):
        v.speed = 0.0
        return True
    v.x, v.y, v.heading, v.speed = nx, ny, heading, speed
    return False


# ---- LiDAR ------------------------------------------------------------------
def _ray_hit(ox, oy, dx, dy, segs) -> float | None:
    best = None
    for (x1, y1), (x2, y2) in segs:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        wx, wy = x1 - ox, y1 - oy
        t = (wx * ey - wy * ex) / den
        u = (wx * dy - wy * dx) / den
        if t > 1e-9 and 0.0 <= u <= 1.0 and (best is None or t < best):
            best = t
    return best


def scan(v: Vehicle, world: World, noise: bool = True) -> list[int]:
    """361 ranges in mm (0 = no echo), index i at i*0.5 deg; 0 = right, 180 = left."""
    ox, oy, hd = v.lidar_pose()
    if any(b.contains(ox, oy) for b in world.solids()):
        return [1] * N_RAYS          # sensor window blocked: everything reads ~touching
    segs = world.segments()
    out = []
    for i in range(N_RAYS):
        a = hd - math.pi / 2 + math.radians(i * 0.5)
        t = _ray_hit(ox, oy, math.cos(a), math.sin(a), segs)
        if t is None or t > LIDAR_MAX:
            out.append(0)
            continue
        if noise:
            t += random.uniform(-NOISE_M, NOISE_M)
        out.append(max(1, int(round(t * 1000))))
    return out


# ---- Presets ----------------------------------------------------------------
START_X, START_Y = ROOM / 2, 1.5          # rear axle, facing +y
LIDAR_Y0 = START_Y + BODY_LEN - REAR_OVERHANG


def _start() -> Vehicle:
    return Vehicle(START_X, START_Y, math.pi / 2, 0.0)


PRESETS = {
    # 1: wall 1.5 m straight ahead of the LiDAR
    "1": lambda: (_start(), [Box(START_X, LIDAR_Y0 + 1.5 + 0.1, 3.0, 0.2)]),
    # 2: box ahead-left (1.2 m forward, 0.6 m left of the LiDAR)
    "2": lambda: (_start(), [Box(START_X - 0.6, LIDAR_Y0 + 1.2, 0.4, 0.4)]),
    # 3: box ahead-right (mirror of 2)
    "3": lambda: (_start(), [Box(START_X + 0.6, LIDAR_Y0 + 1.2, 0.4, 0.4)]),
    # 4: narrow gap 1.0 m wide, 2 m ahead
    "4": lambda: (_start(), [Box(START_X - 1.0, LIDAR_Y0 + 2.0 + 0.25, 1.0, 0.5),
                             Box(START_X + 1.0, LIDAR_Y0 + 2.0 + 0.25, 1.0, 0.5)]),
}

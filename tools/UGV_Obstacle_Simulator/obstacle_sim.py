"""Obstacle Simulator — drop-in replacement for Weeder_Simulator.exe with draggable obstacles.

Run:  python obstacle_sim.py      (close Weeder_Simulator.exe first; same ports)
"""
from __future__ import annotations

import math
import os
import sys
import time
import tkinter as tk
from tkinter import messagebox

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)               # works whatever this folder is called
import sim_core as sc                                              # noqa: E402
from servers import PortInUseError, SimState, start_servers       # noqa: E402


def find_gps_file() -> str:
    """GPSData.dat next to this script, else the course simulator's copy beside this folder."""
    for p in (os.path.join(HERE, "GPSData.dat"),
              os.path.join(HERE, "..", "WeederSimulator", "GPSData.dat")):
        if os.path.isfile(p):
            return os.path.normpath(p)
    return os.path.join(HERE, "GPSData.dat")


GPS_PATH = find_gps_file()

CANVAS = 720
MARGIN = 0.4                       # m around the room
SCALE = CANVAS / (sc.ROOM + 2 * MARGIN)
HANDLE_PX = 9
WARN_DIST = 1.0                    # m, red points + arc

C_BG, C_ROOM, C_WALL = "#1e1f24", "#2a2c33", "#8a8f99"
C_BOX, C_BOX_EDGE, C_HANDLE = "#c98a2b", "#f2c172", "#ffffff"
C_UGV, C_UGV_COLL, C_RAY, C_HIT, C_NEAR = "#3d7be0", "#e04040", "#3a4a5e", "#7fd1ff", "#ff4d4d"
C_ARC = "#ff9a3d"

HELP = ("Mouse: drag box to move | drag white corner to resize | right-click deletes\n"
        "Keys: 1 wall ahead  2 box ahead-left  3 box ahead-right  4 narrow gap\n"
        "      N new box at cursor  Del delete box  R reset UGV  W walls on/off")


def w2c(x, y):
    return (x + MARGIN) * SCALE, CANVAS - (y + MARGIN) * SCALE


def c2w(px, py):
    return px / SCALE - MARGIN, (CANVAS - py) / SCALE - MARGIN


class App:
    def __init__(self, root: tk.Tk, state: SimState, handle):
        self.root, self.state, self.handle = root, state, handle
        root.title("Obstacle Simulator — UGV crash-avoidance test bench")
        root.configure(bg=C_BG)

        self.canvas = tk.Canvas(root, width=CANVAS, height=CANVAS, bg=C_BG, highlightthickness=0)
        self.canvas.grid(row=0, column=0, padx=8, pady=8)
        panel = tk.Frame(root, bg=C_BG)
        panel.grid(row=0, column=1, sticky="n", padx=(0, 10), pady=8)

        self.vars = {}
        rows = [("Cmd steer (deg)", "steer"), ("Cmd speed (m/s)", "speed"),
                ("Actual speed (m/s)", "actual"), ("Watchdog", "wdog"),
                ("LiDAR clients", "lidar"), ("GNSS clients", "gnss"), ("VC", "vc"),
                ("Nearest ahead (m)", "ahead"), ("Walls", "walls")]
        for r, (label, key) in enumerate(rows):
            tk.Label(panel, text=label, fg="#aab", bg=C_BG, font=("Consolas", 11), anchor="w",
                     width=19).grid(row=r, column=0, sticky="w")
            v = tk.StringVar(value="-")
            tk.Label(panel, textvariable=v, fg="#fff", bg=C_BG, font=("Consolas", 11, "bold"),
                     anchor="w", width=14).grid(row=r, column=1, sticky="w")
            self.vars[key] = v
        self.status = tk.Label(panel, text="", font=("Consolas", 14, "bold"), bg=C_BG, width=26, pady=8)
        self.status.grid(row=len(rows), column=0, columnspan=2, pady=(12, 4))
        sign = "LEFT" if sc.STEER_SIGN > 0 else "RIGHT"
        tk.Label(panel, text=f"+steer turns {sign}  (STEER_SIGN in sim_core.py)", fg="#889", bg=C_BG,
                 font=("Consolas", 9)).grid(row=len(rows) + 1, column=0, columnspan=2, sticky="w")
        tk.Label(panel, text=f"UGV {sc.BODY_LEN} x {sc.BODY_WID} m, LiDAR at front-centre", fg="#889",
                 bg=C_BG, font=("Consolas", 9)).grid(row=len(rows) + 2, column=0, columnspan=2, sticky="w")
        tk.Label(root, text=HELP, fg="#ccd", bg=C_BG, font=("Consolas", 10), justify="left").grid(
            row=1, column=0, columnspan=2, sticky="w", padx=10, pady=(0, 8))

        self.drag = None            # (mode, box, dx, dy)
        self.mouse = (sc.ROOM / 2, sc.ROOM / 2)
        c = self.canvas
        c.bind("<ButtonPress-1>", self.on_press)
        c.bind("<B1-Motion>", self.on_drag)
        c.bind("<ButtonRelease-1>", lambda e: setattr(self, "drag", None))
        c.bind("<Button-3>", self.on_right)
        c.bind("<Motion>", lambda e: setattr(self, "mouse", c2w(e.x, e.y)))
        for k in "1234":
            root.bind(k, lambda e, k=k: self.load_preset(k))
        for k in ("n", "N"):
            root.bind(k, lambda e: self.new_box())
        root.bind("<Delete>", lambda e: self.delete_at(*self.mouse))
        for k in ("r", "R"):
            root.bind(k, lambda e: self.reset_pose())
        for k in ("w", "W"):
            root.bind(k, lambda e: self.toggle_walls())
        root.protocol("WM_DELETE_WINDOW", self.close)

        self.last_t = time.monotonic()
        self.tick()
        self.redraw()

    # ---- editing ------------------------------------------------------------
    def box_at(self, x, y):
        for b in reversed(self.state.world.boxes):
            if b.contains(x, y):
                return b
        return None

    def handle_at(self, px, py):
        for b in reversed(self.state.world.boxes):
            hx, hy = w2c(b.cx + b.w / 2, b.cy + b.h / 2)
            if abs(px - hx) <= HANDLE_PX and abs(py - hy) <= HANDLE_PX:
                return b
        return None

    def on_press(self, e):
        x, y = c2w(e.x, e.y)
        with self.state.lock:
            b = self.handle_at(e.x, e.y)
            if b is not None:
                self.drag = ("resize", b, b.cx - b.w / 2, b.cy - b.h / 2)   # anchor = bottom-left
                return
            b = self.box_at(x, y)
            if b is not None:
                self.drag = ("move", b, b.cx - x, b.cy - y)

    def on_drag(self, e):
        if not self.drag:
            return
        x, y = c2w(e.x, e.y)
        mode, b, a1, a2 = self.drag
        with self.state.lock:
            if mode == "move":
                b.cx, b.cy = x + a1, y + a2
            else:
                x2, y2 = max(x, a1 + 0.1), max(y, a2 + 0.1)
                b.w, b.h = x2 - a1, y2 - a2
                b.cx, b.cy = (a1 + x2) / 2, (a2 + y2) / 2

    def on_right(self, e):
        self.delete_at(*c2w(e.x, e.y))

    def delete_at(self, x, y):
        with self.state.lock:
            b = self.box_at(x, y)
            if b is not None:
                self.state.world.boxes.remove(b)

    def new_box(self):
        x, y = self.mouse
        with self.state.lock:
            self.state.world.boxes.append(sc.Box(x, y, 0.4, 0.4))

    def load_preset(self, key):
        v, boxes = sc.PRESETS[key]()
        with self.state.lock:
            self.state.vehicle = v
            self.state.world.boxes = boxes
            self.state.collision = False

    def reset_pose(self):
        v, _ = sc.PRESETS["1"]()
        with self.state.lock:
            self.state.vehicle = v
            self.state.collision = False

    def toggle_walls(self):
        with self.state.lock:
            self.state.world.walls = not self.state.world.walls

    def close(self):
        try:
            self.handle.stop()
        finally:
            self.root.destroy()

    # ---- simulation ---------------------------------------------------------
    def tick(self):
        now = time.monotonic()
        dt = min(0.05, now - self.last_t)
        self.last_t = now
        st = self.state
        with st.lock:
            speed = st.cmd_speed if st.wdog_ok(now) else 0.0
            if sc.step(st.vehicle, st.world, st.cmd_steer, speed, dt):
                st.collision = True
        self.root.after(20, self.tick)

    # ---- drawing ------------------------------------------------------------
    def redraw(self):
        st, c = self.state, self.canvas
        v, w = st.snapshot()
        with st.lock:
            info = dict(steer=st.cmd_steer, speed=st.cmd_speed, wdog=st.wdog_ok(time.monotonic()),
                        vc=st.vc_connected, lidar=st.lidar_clients, gnss=st.gnss_clients,
                        coll=st.collision)
        ranges = sc.scan(v, w, noise=False)
        c.delete("all")

        x0, y0 = w2c(0, sc.ROOM)
        x1, y1 = w2c(sc.ROOM, 0)
        c.create_rectangle(x0, y0, x1, y1, fill=C_ROOM, outline="")
        for gx in range(int(sc.ROOM) + 1):
            a, b = w2c(gx, 0), w2c(gx, sc.ROOM)
            c.create_line(*a, *b, fill="#30333b")
            a, b = w2c(0, gx), w2c(sc.ROOM, gx)
            c.create_line(*a, *b, fill="#30333b")
        for wb in w.wall_boxes():
            pts = [p for xy in wb.corners() for p in w2c(*xy)]
            c.create_polygon(pts, fill=C_WALL, outline="")

        # LiDAR rays + hits
        ox, oy, hd = v.lidar_pose()
        o = w2c(ox, oy)
        near_ahead = None
        for i, r in enumerate(ranges):
            if r == 0:
                continue
            a = hd - math.pi / 2 + math.radians(i * 0.5)
            d = r / 1000.0
            p = w2c(ox + d * math.cos(a), oy + d * math.sin(a))
            if i % 4 == 0:
                c.create_line(*o, *p, fill=C_RAY)
            col = C_NEAR if d <= WARN_DIST else C_HIT
            c.create_oval(p[0] - 1.5, p[1] - 1.5, p[0] + 1.5, p[1] + 1.5, fill=col, outline="")
            # nearest point inside the vehicle's straight-ahead corridor
            lat = d * math.cos(math.radians(i * 0.5))
            fwd = d * math.sin(math.radians(i * 0.5))
            if abs(lat) <= sc.BODY_WID / 2 and fwd > 0 and (near_ahead is None or fwd < near_ahead):
                near_ahead = fwd

        # 1 m warning arc
        R = WARN_DIST * SCALE
        c.create_arc(o[0] - R, o[1] - R, o[0] + R, o[1] + R, start=math.degrees(hd) - 90, extent=180,
                     style="arc", outline=C_ARC, dash=(4, 3), width=1.5)

        # boxes
        for b in w.boxes:
            pts = [p for xy in b.corners() for p in w2c(*xy)]
            c.create_polygon(pts, fill=C_BOX, outline=C_BOX_EDGE, width=1.5)
            hx, hy = w2c(b.cx + b.w / 2, b.cy + b.h / 2)
            c.create_rectangle(hx - 4, hy - 4, hx + 4, hy + 4, fill=C_HANDLE, outline="")

        # vehicle
        pts = [p for xy in v.corners() for p in w2c(*xy)]
        c.create_polygon(pts, fill=C_UGV_COLL if info["coll"] else C_UGV, outline="#dfe8ff", width=1.5)
        cx, cy = v.centre()
        a = w2c(cx, cy)
        c.create_line(*a, *o, fill="#ffffff", width=2, arrow="last")
        c.create_oval(o[0] - 4, o[1] - 4, o[0] + 4, o[1] + 4, fill="#ffe066", outline="")

        if info["coll"]:
            c.create_text(CANVAS / 2, 30, text="COLLISION  —  press R to reset", fill="#ff5555",
                          font=("Consolas", 20, "bold"))

        # panel
        vs = self.vars
        vs["steer"].set(f"{info['steer']:+.1f}")
        vs["speed"].set(f"{info['speed']:+.3f}")
        vs["actual"].set(f"{v.speed:+.3f}")
        vs["wdog"].set("toggling" if info["wdog"] else "FROZEN")
        vs["lidar"].set(str(info["lidar"]))
        vs["gnss"].set(str(info["gnss"]))
        vs["vc"].set("connected" if info["vc"] else "not connected")
        vs["ahead"].set(f"{near_ahead:.2f}" if near_ahead is not None else "none")
        vs["walls"].set("on" if w.walls else "off")
        if info["coll"]:
            self.status.config(text="COLLISION", fg="#fff", bg="#b02a2a")
        elif not info["vc"]:
            self.status.config(text="NO VC CONNECTION", fg="#ddd", bg="#444")
        elif not info["wdog"]:
            self.status.config(text="INHIBITED (wdog)", fg="#111", bg="#f0b030")
        else:
            self.status.config(text="OK", fg="#111", bg="#4cc46a")

        self.root.after(33, self.redraw)


def fit_to_screen(root: tk.Tk):
    """Shrink the map so the whole window (map + help text + taskbar) fits on small screens."""
    global CANVAS, SCALE
    CANVAS = max(420, min(720, root.winfo_screenheight() - 240))
    SCALE = CANVAS / (sc.ROOM + 2 * MARGIN)
    root.geometry("+10+0")


def main():
    state = SimState()
    root = tk.Tk()
    fit_to_screen(root)
    try:
        handle = start_servers(state, GPS_PATH)
    except PortInUseError as e:
        root.withdraw()
        messagebox.showerror("Obstacle Simulator",
                             f"Port {e.port} is in use.\n\nClose Weeder_Simulator.exe (or another "
                             f"copy of this simulator) and try again.")
        root.destroy()
        sys.exit(1)
    print("Obstacle Simulator serving LiDAR :23000, GNSS :24000, VC :25000 on 127.0.0.1")
    App(root, state, handle)
    root.mainloop()


if __name__ == "__main__":
    main()

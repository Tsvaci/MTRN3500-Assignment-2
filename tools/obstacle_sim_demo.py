"""Watchable self-driving demo: Obstacle Simulator window + scripted-controller exemplar exe.
Start MATLAB runner.m first, then:
    python tools/obstacle_sim_demo.py tools/UGV_Obstacle_Simulator <test exe from build_test_exe.py>
"""
import os, socket, subprocess, sys, tempfile, threading, time

SIM, EXE = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2])
sys.path.insert(0, SIM)
import tkinter as tk
import obstacle_sim as osim
import sim_core as sc
from servers import SimState, start_servers

INPUT = os.path.join(tempfile.gettempdir(), "mtrn_demo_input.txt")
def ctl(rt=0.0, lt=0.0, rx=0.0, b=0):
    with open(INPUT + ".tmp", "w") as f:
        f.write(f"1 {rt} {lt} {rx} {b}")
    os.replace(INPUT + ".tmp", INPUT)
ctl()

def port_open(p):
    try:
        socket.create_connection(("127.0.0.1", p), 0.3).close(); return True
    except OSError:
        return False

MSG = {"text": "starting"}

def demo(app):
    def say(s):
        print(s, flush=True); MSG["text"] = s
    say("waiting for MATLAB display engine on port 28000...")
    # Don't probe 28000 with a connect (the engine would take it as its client): check netstat instead.
    while "127.0.0.1:28000" not in subprocess.run("netstat -an", capture_output=True, text=True, shell=True).stdout:
        time.sleep(1)
    time.sleep(2)
    subprocess.Popen([EXE], cwd=os.path.dirname(EXE), env=dict(os.environ, MTRN_TEST_INPUT=INPUT),
                     creationflags=subprocess.CREATE_NEW_CONSOLE)
    time.sleep(6)

    def step(title, preset, secs, **inp):
        say(title)
        ctl(); time.sleep(1.0)
        if preset:
            app.load_preset(preset)
        time.sleep(1.5)
        ctl(**inp); time.sleep(secs); ctl()

    step("1/6 Wall ahead: full throttle - should slow and stop ~0.4 m short", "1", 5, rt=1.0)
    say("2/6 Reverse away from the wall (never limited)"); ctl(lt=1.0); time.sleep(1.5); ctl(); time.sleep(1.5)
    step("3/6 Box ahead-left: full throttle, hard LEFT - should swing past it", "2", 4, rt=1.0, rx=-1.0)
    step("4/6 Box ahead-left: straight - only 5 cm clearance, so it stops", "2", 4, rt=1.0)
    step("5/6 1 m gap, then the far wall: straight at full speed", "4", 11, rt=1.0)
    say("6/6 Weaving at 60% throttle among boxes")
    ctl(); time.sleep(1.0)
    with app.state.lock:
        app.state.vehicle = sc._start(); app.state.collision = False
        app.state.world.boxes = [sc.Box(5.0, 4.5, 0.5, 0.5), sc.Box(7.2, 6.5, 0.5, 0.5), sc.Box(5.5, 8.5, 0.6, 0.6)]
    time.sleep(1.5)
    for rx in (-0.4, 0.5, -0.5, 0.4, -0.3, 0.6):
        ctl(rt=0.6, rx=rx); time.sleep(2.2)
    ctl(); time.sleep(1.5)
    say("Demo finished - pressing B (shutdown). Close this window when done.")
    ctl(b=1)

def main():
    state = SimState()
    root = tk.Tk()
    osim.fit_to_screen(root)
    handle = start_servers(state, osim.GPS_PATH)
    app = osim.App(root, state, handle)
    def show():                       # window title updated from the Tk thread only
        root.title("DEMO: " + MSG["text"]); root.after(200, show)
    show()
    threading.Thread(target=demo, args=(app,), daemon=True).start()
    root.mainloop()

main()

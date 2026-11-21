"""Print a per-scenario summary of run_obstacle_sim_scenarios.py output. Usage: python summarise_obstacle_sim.py <run.json>"""
import json, sys, math
d = json.load(open(sys.argv[1]))
log = d["log"]
names = []
for r in log:
    if r["s"] not in names and r["s"] not in ("idle", "startup"): names.append(r["s"])
for n in names:
    R = [r for r in log if r["s"] == n]
    t0 = R[0]["t"]
    col = any(r["col"] for r in R)
    tcol = next((r["t"]-t0 for r in R if r["col"]), None)
    dist = sum(math.hypot(b["x"]-a["x"], b["y"]-a["y"]) for a, b in zip(R, R[1:]))
    print(f"\n== {n}")
    print(f"  max speed {max(r['v'] for r in R):.2f}, final speed {R[-1]['v']:.2f}, travelled {dist:.2f} m, heading change {R[-1]['h']-R[0]['h']:+.1f} deg")
    print(f"  min gap to anything {min(r['gap'] for r in R):.3f} m, final gap {R[-1]['gap']:.3f}, collision {col} {'' if tcol is None else f'at {tcol:.2f}s'}")
    print(f"  cmd steer range {min(r['cs'] for r in R):+.1f}..{max(r['cs'] for r in R):+.1f}, wdog ok always {all(r['wd'] for r in R)}")
    # timeline every 0.5 s
    nxt = 0
    for r in R:
        if r["t"]-t0 >= nxt:
            a = r["ahead"]
            print(f"   t={r['t']-t0:4.1f} v={r['v']:+.2f} cmd v={r['cv']:+.2f} st={r['cs']:+5.1f} gap={r['gap']:.2f} ahead={'-' if a is None else f'{a:.2f}'}")
            nxt += 0.5

#!/usr/bin/env python3
"""Compare CUDA-port traces against original Box3D."""
import argparse
import math
import sys
from collections import defaultdict

def parse(path):
    traces = defaultdict(dict)
    stats = {}
    perfs = []
    for line in open(path):
        p = line.split()
        if not p:
            continue
        if p[0] == "T":
            scene, step, body = p[1], int(p[2]), int(p[3])
            vals = tuple(float(x) for x in p[4:10])
            traces[(scene, step, body)] = vals
        elif p[0] == "S":
            stats[p[1]] = p[2:]
        elif p[0] == "P":
            perfs.append(line.strip())
    return traces, stats, perfs

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ref")
    ap.add_argument("port")
    args = ap.parse_args()
    ref, rs, rp = parse(args.ref)
    port, ps, pp = parse(args.port)
    keys = sorted(set(ref) | set(port))
    missing = 0
    worst = []
    for k in keys:
        if k not in ref or k not in port:
            print("MISSING", k, "ref" if k in ref else "port")
            missing += 1
            continue
        a, b = ref[k], port[k]
        d = [abs(x - y) for x, y in zip(a, b)]
        worst.append((max(d), k, a, b, d))
    worst.sort(reverse=True)
    print(f"compared {len(keys) - missing} samples, missing {missing}")
    print("worst abs errors (pos xyz, vel xyz):")
    by_scene = defaultdict(lambda: [0.0] * 6)
    for err, k, a, b, d in worst:
        scene = k[0]
        for i in range(6):
            by_scene[scene][i] = max(by_scene[scene][i], d[i])
    labels = "x y z vx vy vz".split()
    fail = 0
    # Free-fall should match tightly. Contacts can differ more.
    tol = {
        "free_fall": (1e-3, 1e-3),
        "rest": (0.08, 0.15),
        "bounce": (0.6, 2.5),
        "sphere_hit": (0.15, 0.25),
        "kinematic": (0.12, 0.4),
        "stack": (0.15, 0.4),
    }
    for scene, d in sorted(by_scene.items()):
        pos = max(d[0], d[1], d[2])
        vel = max(d[3], d[4], d[5])
        pt, vt = tol.get(scene, (0.2, 0.5))
        ok = pos <= pt and vel <= vt
        if not ok:
            fail += 1
        print(f"  {scene:12s} max|dpos|={pos:.4f} max|dvel|={vel:.4f}"
              f"  tol=({pt},{vt})  {'OK' if ok else 'FAIL'}")
        print("    " + " ".join(f"{labels[i]}={d[i]:.4f}" for i in range(6)))
    print("top 8 samples:")
    for err, k, a, b, d in worst[:8]:
        print(f"  {k} d={['%.4f'%x for x in d]}")
        print(f"    ref={['%.4f'%x for x in a]}")
        print(f"    port={['%.4f'%x for x in b]}")
    if "bounce" in rs and "bounce" in ps:
        print("bounce stats ref", rs["bounce"], "port", ps["bounce"])
    if rp or pp:
        print("perf ref:")
        for line in rp:
            print(" ", line)
        print("perf port:")
        for line in pp:
            print(" ", line)
    if missing or fail:
        sys.exit(1)

if __name__ == "__main__":
    main()

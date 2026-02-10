from __future__ import annotations
import argparse, json
from pathlib import Path
import matplotlib.pyplot as plt

def main():
    ap = argparse.ArgumentParser("Quick visualize plan.json")
    ap.add_argument("--plan", type=Path, required=True)
    args = ap.parse_args()

    data = json.loads(args.plan.read_text(encoding="utf-8"))
    segs = data.get("segments", [])

    plt.figure()
    for seg in segs:
        poses = seg.get("poses", [])
        if len(poses) < 2:
            continue
        xs = [p["x"] for p in poses]
        ys = [p["y"] for p in poses]
        plt.plot(xs, ys, linewidth=1)

    plt.gca().set_aspect("equal")
    plt.title(args.plan.name)
    plt.show()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Plot global-reloc poses over the map footprint to visualize trajectory
coherence. Reads a file of "start tx ty tz yaw_deg tight" lines (one per
overlapping window) and scatters them on the map XY, colored by tight.
Correct localization => a smooth path; wrong/ambiguous => scattered jumps.

Usage: plot_traj.py --map map.pcd --poses traj.txt --out traj.png [--title ..]
"""
import argparse, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection


def load_pcd_xyz(path):
    with open(path, "rb") as f:
        h = b""
        while True:
            l = f.readline(); h += l
            if l.strip() == b"DATA binary": break
        for line in h.decode("ascii", "replace").splitlines():
            if line.startswith("FIELDS"): fields = line.split()[1:]
            elif line.startswith("SIZE"): sizes = [int(x) for x in line.split()[1:]]
            elif line.startswith("COUNT"): counts = [int(x) for x in line.split()[1:]]
            elif line.startswith("POINTS"): n = int(line.split()[1])
        off = {}; o = 0
        for i, nm in enumerate(fields):
            off[nm] = o; o += sizes[i] * counts[i]
        step = o; raw = f.read(step * n)
    a = np.frombuffer(raw, np.uint8).reshape(n, step)
    xyz = np.stack([np.frombuffer(a[:, off[k]:off[k] + 4].tobytes(), np.float32) for k in "xyz"], 1)
    return xyz[np.isfinite(xyz).all(1)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True)
    ap.add_argument("--poses", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--title", default="")
    args = ap.parse_args()

    m = load_pcd_xyz(args.map)
    if len(m) > 40000:
        idx = np.random.RandomState(0).choice(len(m), 40000, replace=False)
        m = m[idx]

    rows = []
    with open(args.poses) as f:
        for line in f:
            p = line.split()
            if len(p) >= 5:
                rows.append([float(x) for x in p[:5]])
    rows = np.array(rows)  # start tx ty tz yaw tight(optional)
    has_tight = rows.shape[1] >= 6

    fig, ax = plt.subplots(figsize=(12, 9))
    ax.scatter(m[:, 0], m[:, 1], s=0.3, c="0.75", alpha=0.5, label="map")

    # Connect consecutive poses in time to reveal jumps.
    order = np.argsort(rows[:, 0])
    rows = rows[order]
    pts = rows[:, 1:3]
    segs = np.stack([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, colors="red", linewidths=1.0, alpha=0.6)
    ax.add_collection(lc)

    if has_tight:
        sc = ax.scatter(pts[:, 0], pts[:, 1], c=rows[:, 5], cmap="viridis", s=40,
                        edgecolor="k", linewidth=0.3, zorder=5)
        plt.colorbar(sc, ax=ax, label="tight_inlier_ratio")
    else:
        ax.scatter(pts[:, 0], pts[:, 1], c="red", s=40, zorder=5)
    ax.scatter(pts[0, 0], pts[0, 1], c="lime", s=120, marker="^", zorder=6, label="start")
    ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_title(args.title + "  (red line = time order; jumps = wrong localizations)")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout(); fig.savefig(args.out, dpi=110)
    print("wrote", args.out)


if __name__ == "__main__":
    main()

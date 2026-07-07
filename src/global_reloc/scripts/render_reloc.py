#!/usr/bin/env python3
"""Render a relocalization result as a PNG.

Overlays map (gray) / query_raw (red) / query_aligned (green) as a top-down
(XY) scatter plus a side (ZY) view. Used for visual iteration of global_reloc.
Reads plain binary PCDs (x y z, optionally extra fields ignored) and an
optional pose [tx ty tz qx qy qz qw] to transform query_raw -> aligned.

Usage:
  render_reloc.py --map m.pcd --query q.pcd [--pose "tx ty tz qx qy qz qw"] \
                  --out r.png [--title "..." --downsample 30000]
"""
import argparse, struct, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_pcd_xyz(path):
    """Minimal binary-PCD XYZ loader (ignores extra fields like intensity)."""
    with open(path, "rb") as f:
        header = b""
        while True:
            line = f.readline()
            header += line
            if line.strip() in (b"DATA binary", b"DATA binary_compressed"):
                break
            if not line:
                raise ValueError("bad PCD header: " + path)
        txt = header.decode("ascii", errors="replace")
        fields, sizes, types, counts = [], [], [], []
        npoints = 0
        data_fmt = "binary"
        for line in txt.splitlines():
            if line.startswith("FIELDS"):
                fields = line.split()[1:]
            elif line.startswith("SIZE"):
                sizes = [int(s) for s in line.split()[1:]]
            elif line.startswith("TYPE"):
                types = line.split()[1:]
            elif line.startswith("COUNT"):
                counts = [int(c) for c in line.split()[1:]]
            elif line.startswith("POINTS"):
                npoints = int(line.split()[1])
            elif line.startswith("WIDTH"):
                pass
            elif line.startswith("DATA"):
                data_fmt = line.split()[1]
        if data_fmt != "binary":
            raise ValueError("only DATA binary supported, got " + data_fmt)
        # Build field name -> offset, and total point step.
        offsets = {}
        off = 0
        for i, name in enumerate(fields):
            cnt = counts[i] if i < len(counts) else 1
            sz = sizes[i] * cnt
            offsets[name] = off
            off += sz
        point_step = off
        raw = f.read(point_step * npoints)
        arr = np.frombuffer(raw, dtype=np.uint8).reshape(npoints, point_step)
        xyz = np.zeros((npoints, 3), dtype=np.float32)
        for col, axis in enumerate(("x", "y", "z")):
            o = offsets[axis]
            fi = fields.index(axis)
            s = sizes[fi]
            dt = types[fi]
            npfmt = {("f", 4): np.float32, ("F", 4): np.float32,
                     ("f", 8): np.float64, ("F", 8): np.float64,
                     ("i", 4): np.int32, ("u", 4): np.uint32}.get((dt, s), np.float32)
            xyz[:, col] = np.frombuffer(arr[:, o:o + s].tobytes(), dtype=npfmt)
        mask = np.isfinite(xyz).all(axis=1)
        return xyz[mask]


def quat_to_rot(qx, qy, qz, qw):
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True)
    ap.add_argument("--query", required=True)
    ap.add_argument("--pose", default="", help="tx ty tz qx qy qz qw")
    ap.add_argument("--out", required=True)
    ap.add_argument("--title", default="")
    ap.add_argument("--downsample", type=int, default=30000)
    args = ap.parse_args()

    m = load_pcd_xyz(args.map)
    q = load_pcd_xyz(args.query)
    if m.shape[0] > args.downsample:
        idx = np.random.RandomState(0).choice(m.shape[0], args.downsample, replace=False)
        m = m[idx]

    title = args.title
    if args.pose.strip():
        v = [float(x) for x in args.pose.split()]
        t = np.array(v[:3])
        R = quat_to_rot(*v[3:7]) if len(v) >= 7 else np.eye(3)
        aligned = (R @ q.T).T + t
    else:
        aligned = q
        title = (title + " (query NOT transformed)").strip()

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    ax1.scatter(m[:, 0], m[:, 1], s=0.3, c="0.6", alpha=0.5, label="map")
    ax1.scatter(q[:, 0], q[:, 1], s=1.5, c="red", alpha=0.8, label="query_raw")
    ax1.scatter(aligned[:, 0], aligned[:, 1], s=1.5, c="lime", alpha=0.9, label="aligned")
    ax1.set_aspect("equal")
    ax1.set_title("top-down (XY)")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.grid(alpha=0.3)

    ax2.scatter(m[:, 2], m[:, 1], s=0.3, c="0.6", alpha=0.5, label="map")
    ax2.scatter(q[:, 2], q[:, 1], s=1.5, c="red", alpha=0.8, label="query_raw")
    ax2.scatter(aligned[:, 2], aligned[:, 1], s=1.5, c="lime", alpha=0.9, label="aligned")
    ax2.set_aspect("equal")
    ax2.set_title("side (ZY)")
    ax2.legend(loc="upper right", fontsize=8)
    ax2.grid(alpha=0.3)

    fig.suptitle(title, fontsize=11)
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print("wrote", args.out)


if __name__ == "__main__":
    main()

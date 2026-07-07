#!/usr/bin/env python3
"""Ground-truth evaluation of global_reloc against a super_lio GT trajectory.

For a set of query times, extract a /livox/lidar window, run reloc_cli, and
compare the estimated pose to the GT pose (interpolated from gt.tum) at the
window's midpoint bag-time. Reports translation + yaw error per segment and an
overall success rate — the FIRST trustworthy correctness metric (the pipeline's
own inlier/residual/confidence are unreliable in this map).

NOTE: GT (super_lio /lio/odom) and global_reloc both express the pose in the
map.pcd frame; super_lio's origin is the IMU body, global_reloc's is the LiDAR,
differing by the lidar-imu extrinsic (~4 cm) — negligible vs the 1 m gate.

Usage (inside container, tools on PATH):
  gt_eval.py --gt runs/gt/gt.tum --bag <bag> --topic /livox/lidar \
            --map runs/map.gkey --params config/params.yaml \
            --out-dir runs/gt_eval --starts "0 20 40 ..." --duration 2.0
"""
import argparse, os, subprocess, math, csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def run(cmd, bin_dir):
    e = dict(os.environ); e["PATH"] = bin_dir + os.pathsep + e.get("PATH", "")
    return subprocess.run(cmd, shell=True, capture_output=True, text=True, env=e, executable="/bin/bash")


def parse_reloc(out):
    for line in out.splitlines():
        if line.startswith("T = ["):
            v = line.split("]", 1)[1].split()
            return [float(x) for x in v[:7]]  # tx ty tz qx qy qz qw
    return None


def quat_to_yaw(q):
    # yaw about map-Z from quaternion (x,y,z,w)
    x, y, z, w = q
    siny = 2 * (w * z + x * y); cosy = 1 - 2 * (y * y + z * z)
    return math.atan2(siny, cosy)


def interp_gt(gt, t):
    """gt: (N,8) [t,x,y,z,qx,qy,qz,qw] sorted by t. Returns (xyz,yaw) at t or None."""
    ts = gt[:, 0]
    if t < ts[0] or t > ts[-1]:
        return None
    i = np.searchsorted(ts, t) - 1
    i = max(0, min(i, len(gt) - 2))
    t0, t1 = ts[i], ts[i + 1]
    a = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
    xyz = (1 - a) * gt[i, 1:4] + a * gt[i + 1, 1:4]
    # nlerp quaternion then yaw
    q0, q1 = gt[i, 4:8], gt[i + 1, 4:8]
    if np.dot(q0, q1) < 0: q1 = -q1
    q = (1 - a) * q0 + a * q1
    q = q / np.linalg.norm(q)
    return xyz, quat_to_yaw(q)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gt", required=True)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--topic", default="/livox/lidar")
    ap.add_argument("--map", required=True)
    ap.add_argument("--params", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--starts", required=True, help="space-separated relative start seconds")
    ap.add_argument("--duration", type=float, default=2.0)
    ap.add_argument("--bin-dir", default="/ros2_ws/install/global_reloc/lib/global_reloc")
    ap.add_argument("--trans-gate", type=float, default=1.0)
    ap.add_argument("--yaw-gate", type=float, default=15.0)
    ap.add_argument("--align-gravity", action="store_true",
                    help="gravity-align query (z-up) to match super_lio GT")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    gt = np.loadtxt(args.gt)
    gt = gt[np.argsort(gt[:, 0])]
    print(f"GT: {len(gt)} poses, t[{gt[0,0]:.1f}..{gt[-1,0]:.1f}]")

    starts = [float(s) for s in args.starts.split()]
    # Extract all windows in one pass; capture ABSTIME rel->abs mapping.
    # If --align-gravity is set, the query is gravity-aligned (z-up) to match
    # the super_lio GT convention; pass it through to bag_to_query.
    starts_str = " ".join(str(int(s)) for s in starts)
    ga_flag = "--align-gravity --imu-topic /livox/imu" if args.align_gravity else ""
    r = run(f'bag_to_query --bag {args.bag} --topic {args.topic} --out-dir {args.out_dir} '
            f'--duration {args.duration} --starts "{starts_str}" {ga_flag}', args.bin_dir)
    abs_mid = {}
    for line in r.stdout.splitlines():
        if line.startswith("ABSTIME"):
            _, rel, absm = line.split()
            abs_mid[float(rel)] = float(absm)

    rows = []
    for s in starts:
        rel = float(int(s))
        f = os.path.join(args.out_dir, f"q_{int(s)}.pcd")
        if not os.path.exists(f) or rel not in abs_mid:
            print(f"start={s}: missing window"); continue
        out = run(f'reloc_cli --map {args.map} --query {f} --params {args.params}', args.bin_dir).stdout
        est = parse_reloc(out)
        if est is None:
            print(f"start={s}: reloc failed"); continue
        gt_at = interp_gt(gt, abs_mid[rel])
        if gt_at is None:
            print(f"start={s}: GT out of range"); continue
        gt_xyz, gt_yaw = gt_at
        est_xyz = np.array(est[:3]); est_yaw = quat_to_yaw(est[3:7])
        terr = np.linalg.norm(est_xyz[:2] - gt_xyz[:2])
        yerr = math.degrees((est_yaw - gt_yaw + math.pi) % (2 * math.pi) - math.pi)
        ok = terr < args.trans_gate and abs(yerr) < args.yaw_gate
        rows.append((s, est_xyz[0], est_xyz[1], est_xyz[2], est_yaw,
                     gt_xyz[0], gt_xyz[1], gt_xyz[2], gt_yaw, terr, yerr, int(ok)))
        flag = "OK " if ok else "MISS"
        print(f"start={s:6.1f} {flag} terr={terr:6.2f}m yerr={yerr:+6.1f}deg   est=({est_xyz[0]:6.1f},{est_xyz[1]:6.1f}) gt=({gt_xyz[0]:6.1f},{gt_xyz[1]:6.1f})")

    if not rows:
        print("no results"); return
    rows = np.array(rows)
    n_ok = int(rows[:, 11].sum())
    print(f"\n=== SUCCESS {n_ok}/{len(rows)} ({100*n_ok/len(rows):.0f}%) "
          f"trans_gate<{args.trans_gate}m yaw_gate<{args.yaw_gate}deg ===")
    print(f"median terr={np.median(rows[:,9]):.2f}m  median |yerr|={np.median(np.abs(rows[:,10])):.1f}deg")

    # csv
    with open(os.path.join(args.out_dir, "gt_eval.csv"), "w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["start", "est_x", "est_y", "est_z", "est_yaw", "gt_x", "gt_y", "gt_z", "gt_yaw", "terr_m", "yerr_deg", "ok"])
        for row in rows: w.writerow(row)

    # plot: GT black line + est points colored by terr (green=ok, red=miss)
    fig, ax = plt.subplots(figsize=(13, 9))
    ax.plot(gt[:, 1], gt[:, 2], "k-", lw=0.8, alpha=0.5, label="GT trajectory (super_lio)")
    # GT positions at each eval time (the ground-truth the estimate should match)
    gt_at_eval = np.array([interp_gt(gt, abs_mid.get(float(int(s)), 0)) for s in starts
                           if float(int(s)) in abs_mid and interp_gt(gt, abs_mid[float(int(s))])])
    if len(gt_at_eval):
        gtxy = np.array([g[0] for g in gt_at_eval])
        ax.scatter(gtxy[:, 0], gtxy[:, 1], c="blue", s=50, marker="x",
                   linewidth=1.5, zorder=6, label="GT pose @ eval time")
    cols = ["green" if r[11] else "red" for r in rows]
    ax.scatter(rows[:, 1], rows[:, 2], c=cols, s=70, edgecolor="k", linewidth=0.4, zorder=5, label="global_reloc est")
    for r in rows:
        ax.plot([r[1], r[5]], [r[2], r[6]], ":", color="gray", lw=0.6, zorder=4)
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend(loc="upper right")
    ax.set_title(f"GT vs global_reloc  ({n_ok}/{len(rows)} OK, green=hit red=miss; blue x=GT, dotted=est->GT)")
    fig.tight_layout(); fig.savefig(os.path.join(args.out_dir, "gt_eval.png"), dpi=110)
    print("wrote gt_eval.csv + gt_eval.png")


if __name__ == "__main__":
    main()

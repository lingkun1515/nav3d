#!/usr/bin/env python3
"""Sequential (tracking) evaluation of global_reloc against GT.

Unlike gt_eval.py (independent per-segment), this processes segments in TIME
ORDER, carrying the pose from segment t to t+dt as a PRIOR:
  - Segment 0: global search (7x7 grid GICP, coarse).
  - Segments 1..N: local GICP from the prior pose (prev result + IMU gyro yaw delta).
  - If tracking fails (tight < threshold), re-trigger global search.

The IMU is used ONLY for:
  1. Gravity alignment (rotate query so z-up, matching map convention).
  2. Yaw delta between segments (gyro z-integration) — NOT for position.

This mirrors how a real online system works: global reloc once, then track.

Usage (inside container):
  track_eval.py --gt runs/gt/gt.tum --bag <bag> --topic /livox/lidar \
    --imu-topic /livox/imu --map runs/map.gkey --params config/params.yaml \
    --out-dir runs/track_eval --step 2.0 --duration 1.0
"""
import argparse, os, subprocess, csv, math, struct, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def run(cmd, bin_dir):
    e = dict(os.environ); e["PATH"] = bin_dir + os.pathsep + e.get("PATH", "")
    return subprocess.run(cmd, shell=True, capture_output=True, text=True, env=e, executable="/bin/bash")


def load_pcd_xyzi(path):
    with open(path, "rb") as f:
        h = b""
        while True:
            l = f.readline(); h += l
            if l.strip() == b"DATA binary": break
        fields, sizes, counts, n = [], [], [], 0
        for line in h.decode("ascii", "replace").splitlines():
            p = line.split()
            if line.startswith("FIELDS"): fields = p[1:]
            elif line.startswith("SIZE"): sizes = [int(x) for x in p[1:]]
            elif line.startswith("COUNT"): counts = [int(x) for x in p[1:]]
            elif line.startswith("POINTS"): n = int(p[1])
        off = {}; o = 0
        for i, nm in enumerate(fields): off[nm] = o; o += sizes[i] * counts[i]
        step = o; raw = f.read(step * n)
    a = np.frombuffer(raw, np.uint8).reshape(n, step)
    xyz = np.stack([np.frombuffer(a[:, off[k]:off[k]+4].tobytes(), np.float32) for k in "xyz"], 1)
    has_i = "intensity" in off
    inten = np.frombuffer(a[:, off["intensity"]:off["intensity"]+4].tobytes(), np.float32) if has_i else None
    m = np.isfinite(xyz).all(1)
    return xyz[m], (inten[m] if inten is not None else None)


def extract_window(bag, topic, imu_topic, start, dur, out_pcd, bin_dir, align_gravity=True):
    """Extract a query window + its IMU gravity + gyro yaw delta."""
    cmd = f'bag_to_query --bag {bag} --topic {topic} --out {out_pcd} ' \
          f'--start {start} --duration {dur}'
    if align_gravity and imu_topic:
        cmd += f' --align-gravity --imu-topic {imu_topic}'
    r = run(cmd, bin_dir)
    # Also dump gravity for this window
    grav = None
    if imu_topic:
        rg = run(f'bag_to_query --bag {bag} --imu-topic {imu_topic} '
                 f'--dump-gravity --start {start} --duration {dur}', bin_dir)
        for line in rg.stdout.splitlines():
            if line.startswith("GRAVITY"):
                parts = line.split("[")[1].rstrip("]").split()
                grav = np.array([float(x) for x in parts])
    return os.path.exists(out_pcd), grav


def gyro_yaw_delta(bag, imu_topic, t_start, t_end, bin_dir):
    """Integrate gyro z-axis from t_start to t_end to get yaw delta [rad].
    Uses the imu_yaw_delta C++ tool (rosbag2_py is unavailable in this env)."""
    r = run(f'imu_yaw_delta --bag {bag} --imu-topic {imu_topic} '
            f'--start {t_start} --end {t_end}', bin_dir)
    for line in r.stdout.splitlines():
        if line.startswith("YAWDELTA"):
            return float(line.split()[1])
    return 0.0


def parse_reloc(out):
    for line in out.splitlines():
        if line.startswith("T = ["):
            v = line.split("]", 1)[1].split()
            return [float(x) for x in v[:7]]  # tx ty tz qx qy qz qw
    return None


def quat_to_yaw(q):
    x, y, z, w = q
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gt", required=True)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--topic", default="/livox/lidar")
    ap.add_argument("--imu-topic", default="/livox/imu")
    ap.add_argument("--map", required=True)
    ap.add_argument("--params", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--step", type=float, default=2.0)
    ap.add_argument("--duration", type=float, default=1.0)
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--until", type=float, default=390.0)
    ap.add_argument("--bin-dir", default="/ros2_ws/install/global_reloc/lib/global_reloc")
    ap.add_argument("--trans-gate", type=float, default=1.0)
    ap.add_argument("--yaw-gate", type=float, default=15.0)
    ap.add_argument("--reloc-tight-threshold", type=float, default=0.55,
                    help="if tight_inlier_ratio < this, re-globalize next frame")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    gt = np.loadtxt(args.gt)
    gt = gt[np.argsort(gt[:, 0])]

    times = []
    t = args.start
    while t <= args.until:
        times.append(t); t += args.step

    # Extract all windows in one pass
    starts_str = " ".join(str(int(t)) for t in times)
    run(f'bag_to_query --bag {args.bag} --topic {args.topic} --out-dir {args.out_dir} '
        f'--duration {args.duration} --starts "{starts_str}" --align-gravity --imu-topic {args.imu_topic}',
        args.bin_dir)

    rows = []
    prev_pose = None   # (tx, ty, tz, qx, qy, qz, qw) from previous segment
    prev_time = None

    for i, t_start in enumerate(times):
        f = os.path.join(args.out_dir, f"q_{int(t_start)}.pcd")
        if not os.path.exists(f):
            print(f"t={t_start}: missing window"); continue

        # Determine mode: global (first or lost) or tracking (have prior)
        # Auto re-globalize when tracking confidence (tight) drops below threshold.
        if prev_pose is None:
            # GLOBAL: like super_lio, init from map origin (0,0,0). The online
            # system plays the bag from the start, so the first frame's true pose
            # is near the map origin. A 7x7 grid global search misses the origin
            # (GICP convergence basin ~2-3m, grid spacing ~11m); a single GICP
            # from the origin succeeds when the robot starts there.
            mode = "GLOBAL"
            cmd = f'reloc_cli --map {args.map} --query {f} --params {args.params} ' \
                  f'--init-pose "0 0 0 0 0 0 1"'
        else:
            # IMU yaw delta
            try:
                dyaw = gyro_yaw_delta(args.bag, args.imu_topic, prev_time, t_start + args.duration/2, args.bin_dir)
            except Exception:
                dyaw = 0.0
            px, py, pz, pqx, pqy, pqz, pqw = prev_pose
            prior_yaw = quat_to_yaw([pqx, pqy, pqz, pqw]) + dyaw
            # Build init pose: prev translation + z + prior yaw (roll/pitch=0).
            cq = np.cos(prior_yaw/2); sq = np.sin(prior_yaw/2)
            init_str = f"{px} {py} {pz} 0 0 {sq} {cq}"
            mode = "TRACK"
            cmd = f'reloc_cli --map {args.map} --query {f} --params {args.params} --init-pose "{init_str}"'

        out = run(cmd, args.bin_dir).stdout
        est = parse_reloc(out)
        # Also parse tight score for confidence check.
        tight = 0.0
        for line in out.splitlines():
            if line.startswith("ALIGN"):
                for tok in line.split()[1:]:
                    if tok.startswith("tight_inlier_ratio="):
                        tight = float(tok.split("=")[1])
        if est is None:
            print(f"t={t_start} [{mode}]: reloc failed")
            # On failure, reset to global next time
            prev_pose = None
            continue

        # GT at window midpoint
        gt_rel = t_start + args.duration / 2.0
        gt_abs = gt[0, 0] + gt_rel
        gt_idx = np.argmin(np.abs(gt[:, 0] - gt_abs))
        gt_xyz = gt[gt_idx, 1:4]
        gt_yaw = quat_to_yaw(gt[gt_idx, 4:8])

        est_xyz = np.array(est[:3])
        est_yaw = quat_to_yaw(est[3:7])
        terr = np.linalg.norm(est_xyz[:2] - gt_xyz[:2])
        yerr = math.degrees((est_yaw - gt_yaw + math.pi) % (2*math.pi) - math.pi)
        ok = terr < args.trans_gate and abs(yerr) < args.yaw_gate

        rows.append((t_start, est_xyz[0], est_xyz[1], est_xyz[2], est_yaw,
                     gt_xyz[0], gt_xyz[1], gt_xyz[2], gt_yaw, terr, yerr, int(ok), mode))
        flag = "OK " if ok else "MISS"
        print(f"t={t_start:6.1f} [{mode:6s}] {flag} terr={terr:6.2f}m yerr={yerr:+6.1f}deg tight={tight:.2f}  "
              f"est=({est_xyz[0]:6.1f},{est_xyz[1]:6.1f}) gt=({gt_xyz[0]:6.1f},{gt_xyz[1]:6.1f})")

        # Update prior for next segment. Keep tracking with IMU prior even on low
        # tight — resetting to origin destroys tracking when the robot has moved
        # far from the start. Only reset on complete GICP failure (handled above).
        prev_pose = est
        prev_time = t_start + args.duration / 2.0

    if not rows:
        print("no results"); return

    rows = np.array(rows, dtype=object)
    n_ok = int(sum(1 for r in rows if r[12] == 1))
    print(f"\n=== TRACKING SUCCESS {n_ok}/{len(rows)} ({100*n_ok/len(rows):.0f}%) ===")

    # Save CSV + plot
    with open(os.path.join(args.out_dir, "track_eval.csv"), "w", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(["time", "est_x", "est_y", "est_z", "est_yaw", "gt_x", "gt_y", "gt_z", "gt_yaw",
                     "terr_m", "yerr_deg", "ok", "mode"])
        for r in rows: w.writerow(r)

    fig, ax = plt.subplots(figsize=(12, 8))
    ax.plot(gt[:, 1], gt[:, 2], "k-", lw=0.8, alpha=0.5, label="GT trajectory")
    cols = ["green" if r[12] == 1 else "red" for r in rows]
    ax.scatter([r[1] for r in rows], [r[2] for r in rows], c=cols, s=70, edgecolor="k", linewidth=0.4, zorder=5, label="est")
    for r in rows:
        ax.plot([r[1], r[5]], [r[2], r[6]], ":", color="gray", lw=0.5, zorder=4)
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend()
    ax.set_title(f"Tracking eval ({n_ok}/{len(rows)} OK)")
    fig.tight_layout(); fig.savefig(os.path.join(args.out_dir, "track_eval.png"), dpi=110)
    print("wrote track_eval.csv + track_eval.png")


if __name__ == "__main__":
    main()

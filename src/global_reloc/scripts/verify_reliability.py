#!/usr/bin/env python3
"""Reliability verification for global_reloc (ground-truth-free).

The naive quality metrics (inlier_ratio, mean_residual, tight) are unreliable in
a dense/repetitive map — wrong localizations score high too. This harness uses
TEMPORAL CONSISTENCY as a ground-truth-free oracle: the robot moves continuously,
so relocation results from overlapping time windows must form a continuous path.
A window whose pose agrees with a run (>=3) of neighbours is STABLE (correct);
isolated or jumping windows are UNSTABLE (wrong).

It then checks whether the pipeline's single-shot CONFIDENCE (candidate
distinctiveness, printed as CONF confidence=..) actually predicts stability,
sweeps a confidence threshold, and reports precision/recall + a recommended
gate. If confidence predicts stability well, it is a usable online reliability
signal ("publish the pose only if confidence > gate").

Usage (inside the container, with global_reloc tools on PATH):
  verify_reliability.py --bag <dir> --topic /utlidar/cloud --map <map.gkey> \
      --params params.yaml --step 0.5 --duration 2.0 --out-dir runs/rel [--bin-dir <d>]
"""
import argparse, os, subprocess, csv, math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def run(cmd, bin_dir=None, env=None):
    e = dict(os.environ)
    if bin_dir:
        e["PATH"] = bin_dir + os.pathsep + e.get("PATH", "")
    return subprocess.run(cmd, shell=True, capture_output=True, text=True, env=e, executable="/bin/bash")


def parse_reloc(out):
    """Return (tx,ty,tz,yaw,confidence,score,tight) or None."""
    tx = ty = tz = yaw = conf = score = tight = None
    for line in out.splitlines():
        if line.startswith("T = ["):
            v = line.split("]", 1)[1].split()
            tx, ty, tz, qx, qy, qz, qw = [float(x) for x in v[:7]]
            # yaw from quaternion (rotation about map-Z)
            siny = 2 * (qw * qz + qx * qy); cosy = 1 - 2 * (qy * qy + qz * qz)
            yaw = math.degrees(math.atan2(siny, cosy))
        elif line.startswith("CONF "):
            for tok in line.split()[1:]:
                k, v = tok.split("=")
                if k == "confidence": conf = float(v)
        elif line.startswith("ALIGN"):
            for tok in line.split()[1:]:
                if tok.startswith("tight_inlier_ratio="): tight = float(tok.split("=")[1])
    if tx is None: return None
    return tx, ty, tz, yaw, conf if conf is not None else 0.0, tight or 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--topic", default="/utlidar/cloud")
    ap.add_argument("--map", required=True)
    ap.add_argument("--params", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--step", type=float, default=0.5)
    ap.add_argument("--duration", type=float, default=2.0)
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--until", type=float, default=395.0)
    ap.add_argument("--max-speed", type=float, default=2.0, help="m/s; gates neighbor agreement")
    ap.add_argument("--bin-dir", default="/ros2_ws/install/global_reloc/lib/global_reloc")
    ap.add_argument("--plot-map", default="")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    starts = []
    s = args.start
    while s <= args.until:
        starts.append(round(s, 3)); s += args.step
    print(f"extracting {len(starts)} overlapping windows (dur={args.duration}s step={args.step}s)...")

    starts_str = " ".join(str(x) for x in starts)
    r = run(f'bag_to_query --bag {args.bag} --topic {args.topic} --out-dir {args.out_dir} '
            f'--duration {args.duration} --starts "{starts_str}"', args.bin_dir)
    if r.returncode != 0:
        print("bag_to_query failed:", r.stderr[-500:]); return

    rows = []  # (t, tx,ty,tz,yaw,conf,tight)
    for st in starts:
        f = os.path.join(args.out_dir, f"q_{int(round(st))}.pcd")
        if not os.path.exists(f):
            continue
        out = run(f'reloc_cli --map {args.map} --query {f} --params {args.params}', args.bin_dir).stdout
        p = parse_reloc(out)
        if p:
            rows.append((st,) + p)
    if len(rows) < 5:
        print("too few reloc results"); return

    arr = np.array(rows, dtype=float)  # t, tx,ty,tz,yaw,conf,tight
    t = arr[:, 0]; xy = arr[:, 1:3]; conf = arr[:, 5]

    # --- Temporal-consistency oracle ---
    # Adjacent windows (step apart) agree if their xy distance < max_speed*step (+margin).
    agree_thresh = args.max_speed * args.step + 0.5
    n = len(arr)
    agree = np.zeros(n, dtype=bool)  # agree[i] = window i agrees with i+1
    for i in range(n - 1):
        dt = t[i + 1] - t[i]
        d = np.hypot(xy[i + 1, 0] - xy[i, 0], xy[i + 1, 1] - xy[i, 1])
        agree[i] = d <= agree_thresh * (dt / args.step)
    # A window is STABLE if it sits inside a run of >=3 mutually-agreeing windows.
    stable = np.zeros(n, dtype=bool)
    i = 0
    while i < n:
        j = i
        while j < n - 1 and agree[j]:
            j += 1
        runlen = j - i + 1
        if runlen >= 3:
            stable[i:j + 1] = True
        i = j + 1

    n_stable = int(stable.sum()); n_unstable = n - n_stable
    print(f"windows={n}  STABLE(correct-ish)={n_stable} ({100*n_stable/n:.0f}%)  "
          f"UNSTABLE={n_unstable} ({100*n_unstable/n:.0f}%)")

    # --- Does confidence predict stability? ---
    c_stable = conf[stable]; c_unstable = conf[~stable]
    print(f"confidence: STABLE mean={c_stable.mean():.3f}  UNSTABLE mean={c_unstable.mean():.3f}")

    # Sweep threshold; precision = P(stable | conf>thr), recall = P(conf>thr | stable).
    thrs = np.linspace(0, 1, 101)
    prec = []; rec = []
    for thr in thrs:
        pred = conf > thr
        tp = int((pred & stable).sum()); fp = int((pred & ~stable).sum()); fn = int((~pred & stable).sum())
        prec.append(tp / (tp + fp) if tp + fp else 1.0)
        rec.append(tp / (tp + fn) if tp + fn else 0.0)
    prec = np.array(prec); rec = np.array(rec)
    youden = prec + rec - 1
    best = int(np.argmax(np.where(thrs <= 0.95, youden, -1)))
    best_thr = thrs[best]
    print(f"RECOMMENDED confidence gate: > {best_thr:.2f}  "
          f"(precision={prec[best]:.2f} recall={rec[best]:.2f})")

    # Save csv + report.
    with open(os.path.join(args.out_dir, "reliability.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time", "tx", "ty", "tz", "yaw_deg", "confidence", "tight", "stable"])
        for k in range(n):
            w.writerow([arr[k, 0], arr[k, 1], arr[k, 2], arr[k, 3], arr[k, 4], arr[k, 5], arr[k, 6], int(stable[k])])
    with open(os.path.join(args.out_dir, "reliability_report.txt"), "w") as f:
        f.write(f"windows={n} stable={n_stable} ({100*n_stable/n:.0f}%) unstable={n_unstable}\n")
        f.write(f"confidence STABLE mean={c_stable.mean():.3f} UNSTABLE mean={c_unstable.mean():.3f}\n")
        f.write(f"recommended confidence gate > {best_thr:.2f}  precision={prec[best]:.2f} recall={rec[best]:.2f}\n")

    # --- Plots ---
    # 1) trajectory colored by confidence, marker by stable.
    fig, ax = plt.subplots(figsize=(11, 8))
    if args.plot_map and os.path.exists(args.plot_map):
        from numpy import frombuffer
        # quick map scatter via render loader inline (skip if heavy)
        pass
    sc = ax.scatter(xy[:, 0], xy[:, 1], c=conf, cmap="RdYlGn", s=30,
                    edgecolor="k", linewidth=0.2, zorder=5)
    ax.scatter(xy[~stable, 0], xy[~stable, 1], facecolors="none", edgecolors="red",
               s=120, linewidths=1.8, zorder=6, label="unstable (jump)")
    plt.colorbar(sc, ax=ax, label="confidence")
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend(loc="upper right")
    ax.set_title(f"reloc trajectory (green=confident, red ring=unstable). "
                 f"stable={100*n_stable/n:.0f}%, gate>{best_thr:.2f} P={prec[best]:.2f} R={rec[best]:.2f}")
    fig.tight_layout(); fig.savefig(os.path.join(args.out_dir, "reliability_traj.png"), dpi=110)

    # 2) precision/recall vs threshold.
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(thrs, prec, label="precision (of 'conf>thr is stable')")
    ax.plot(thrs, rec, label="recall (stable detected)")
    ax.axvline(best_thr, color="k", ls="--", label=f"gate={best_thr:.2f}")
    ax.set_xlabel("confidence threshold"); ax.set_ylabel("rate"); ax.legend(); ax.grid(alpha=0.3)
    ax.set_title("confidence -> reliability precision/recall")
    fig.tight_layout(); fig.savefig(os.path.join(args.out_dir, "threshold_curve.png"), dpi=110)

    # 3) confidence histogram by label.
    fig, ax = plt.subplots(figsize=(7, 4))
    bins = np.linspace(0, 1, 21)
    ax.hist(c_stable, bins, alpha=0.6, label=f"stable (n={n_stable})", color="green")
    ax.hist(c_unstable, bins, alpha=0.6, label=f"unstable (n={n_unstable})", color="red")
    ax.axvline(best_thr, color="k", ls="--", label=f"gate={best_thr:.2f}")
    ax.set_xlabel("confidence"); ax.legend(); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(os.path.join(args.out_dir, "conf_hist.png"), dpi=110)
    print("wrote reliability.csv, reliability_report.txt, *.png in", args.out_dir)


if __name__ == "__main__":
    main()

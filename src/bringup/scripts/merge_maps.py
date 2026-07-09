#!/usr/bin/env python3
"""
Merge two preprocessed PCD maps via corner-centered quadrant registration.

KEY INSIGHT: Both maps are gravity-aligned + wall-aligned → yaw is 0/90/180/270°.
The overlap is only in the +X+Y corner → translate both corners to origin,
then do local yaw search (90°/270°) with tight ICP on the corner region only.

Pipeline:
  1. Extract +X+Y corner points from both clouds
  2. Translate both corners to origin (eliminate global offset)
  3. Try yaw = 90° and 270° (±3° fine search) on centered corners
  4. Tight ICP (0.1m threshold) with high overlap → STRICT validation
  5. Compose full transform: center → yaw → ICP → uncenter
  6. Apply to full source cloud and merge

Usage:
  python merge_maps.py <target.pcd> <source.pcd> <output.pcd> [--visualize]
"""

import argparse
import sys
import numpy as np
import open3d as o3d


# ======================================================================
# 3-DOF helpers
# ======================================================================

def rot_z(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def make_3dof(yaw, tx, ty, tz=0.0):
    T = np.eye(4)
    T[:3, :3] = rot_z(yaw)
    T[0, 3] = tx
    T[1, 3] = ty
    T[2, 3] = tz
    return T


def extract_yaw(T):
    return np.arctan2(T[1, 0], T[0, 0])


def project_3dof(T):
    return make_3dof(extract_yaw(T), T[0, 3], T[1, 3], tz=0.0)


# ======================================================================
# Point cloud prep
# ======================================================================

def extract_corner_region(pcd, x_percentile=95, y_percentile=95):
    """
    Extract points in the +X +Y corner (top percentiles, no Z filter).
    Uses percentile-based corner to handle irregular point cloud shapes.
    Returns (corner_pcd, corner_center_xy).
    """
    pts = np.asarray(pcd.points)
    x_thresh = np.percentile(pts[:, 0], x_percentile)
    y_thresh = np.percentile(pts[:, 1], y_percentile)
    mask = (pts[:, 0] > x_thresh) & (pts[:, 1] > y_thresh)

    corner = o3d.geometry.PointCloud()
    corner.points = o3d.utility.Vector3dVector(pts[mask])

    if len(pts[mask]) > 0:
        center = pts[mask][:, :2].mean(axis=0)
    else:
        center = np.array([0.0, 0.0])

    print(f"    Corner region: {len(pts[mask])} pts, "
          f"X>[{x_thresh:.1f}], Y>[{y_thresh:.1f}], "
          f"Z=[{pts[mask][:,2].min():.2f},{pts[mask][:,2].max():.2f}]")

    return corner, center


def prep_cloud(pcd, voxel):
    down = pcd.voxel_down_sample(voxel)
    if len(down.points) >= 10:
        down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2.0, max_nn=30))
    return down


# ======================================================================
# Corner-centered yaw search
# ======================================================================

def run_icp_and_score(src, tgt, T_init, dist_thresh):
    """Run ICP, return (score, fitness, rmse, inlier_count, transform)."""
    result = o3d.pipelines.registration.registration_icp(
        src, tgt, dist_thresh, T_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(1e-8, 1e-8, 200))
    n_inl = int(result.fitness * len(src.points))
    score = n_inl / (1.0 + result.inlier_rmse)
    return score, result.fitness, result.inlier_rmse, n_inl, result.transformation


def search_yaw_on_centered(src_centered, tgt_centered, base_yaw_deg,
                           fine_range=3.0, fine_step=0.5, dist_thresh=0.10):
    """
    Search yaw around base_yaw_deg on corner-centered clouds.
    Both clouds are at origin, so initial transform is pure rotation.
    Returns best (score, yaw_deg, fitness, rmse, inlier_count, T_icp).
    """
    best = None
    for delta in np.arange(-fine_range, fine_range + fine_step, fine_step):
        yaw_deg = base_yaw_deg + delta
        yaw = np.radians(yaw_deg)
        T_init = np.eye(4)
        T_init[:3, :3] = rot_z(yaw)  # pure rotation at origin

        try:
            score, fit, rmse, ninl, T_icp = run_icp_and_score(
                src_centered, tgt_centered, T_init, dist_thresh)
            if best is None or score > best[0]:
                best = (score, yaw_deg, fit, rmse, ninl, T_icp)
        except Exception:
            continue
    return best


# ======================================================================
# Visualization
# ======================================================================

def visualize_alignment(source, target, transform):
    src = o3d.geometry.PointCloud(source).voxel_down_sample(0.15)
    tgt = o3d.geometry.PointCloud(target).voxel_down_sample(0.15)
    src.transform(transform)
    src.paint_uniform_color([1.0, 0.3, 0.3])
    tgt.paint_uniform_color([0.3, 0.8, 0.3])
    o3d.visualization.draw_geometries(
        [src, tgt, o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0)],
        window_name="Red=source Green=target — Close window to merge")


# ======================================================================
# Main
# ======================================================================

def register_and_merge(target_path, source_path, output_path, visualize=True):
    print(f"Target: {target_path}")
    target = o3d.io.read_point_cloud(target_path)
    print(f"  {len(target.points)} points")

    print(f"Source: {source_path}")
    source = o3d.io.read_point_cloud(source_path)
    print(f"  {len(source.points)} points")

    # ---- Step 1: Extract corner regions ----
    print(f"\n{'='*60}")
    print("[1] Extracting +X+Y corner regions (top 5% X & Y, no Z filter) ...")
    src_corner, src_center = extract_corner_region(source, x_percentile=95, y_percentile=95)
    tgt_corner, tgt_center = extract_corner_region(target, x_percentile=95, y_percentile=95)

    print(f"  Source corner: {len(src_corner.points)} pts, "
          f"center=({src_center[0]:.2f}, {src_center[1]:.2f})")
    print(f"  Target corner: {len(tgt_corner.points)} pts, "
          f"center=({tgt_center[0]:.2f}, {tgt_center[1]:.2f})")
    print(f"  Corner offset:  ({tgt_center[0]-src_center[0]:.2f}, "
          f"{tgt_center[1]-src_center[1]:.2f})m")

    if len(src_corner.points) < 200 or len(tgt_corner.points) < 200:
        print("ERROR: Not enough corner points.")
        sys.exit(1)

    # ---- Step 2: Center both corners at origin ----
    print(f"\n[2] Translating both corners to origin ...")
    T_src_to_origin = np.eye(4)
    T_src_to_origin[0, 3] = -src_center[0]
    T_src_to_origin[1, 3] = -src_center[1]

    T_tgt_to_origin = np.eye(4)
    T_tgt_to_origin[0, 3] = -tgt_center[0]
    T_tgt_to_origin[1, 3] = -tgt_center[1]

    src_centered = o3d.geometry.PointCloud(src_corner)
    src_centered.transform(T_src_to_origin)

    tgt_centered = o3d.geometry.PointCloud(tgt_corner)
    tgt_centered.transform(T_tgt_to_origin)

    print(f"  Both corners now centered at origin (0,0)")

    # ---- Step 3: Yaw search (90° / 270° only) on centered corners ----
    print(f"\n[3] Yaw search on centered corners (90° / 270° ±3°, ICP threshold=0.10m) ...")
    src_icp = prep_cloud(src_centered, 0.10)
    tgt_icp = prep_cloud(tgt_centered, 0.10)
    print(f"  ICP clouds: src={len(src_icp.points)}, tgt={len(tgt_icp.points)} (voxel=0.10m)")

    if len(src_icp.points) < 50 or len(tgt_icp.points) < 50:
        print("ERROR: Not enough corner points after downsampling.")
        sys.exit(1)

    results = []
    for base_yaw in [90, 270]:
        print(f"\n  --- Yaw {base_yaw}° (±3°, step=0.5°) ---")
        best = search_yaw_on_centered(src_icp, tgt_icp, base_yaw,
                                      fine_range=3.0, fine_step=0.5,
                                      dist_thresh=0.10)
        if best is None:
            print(f"    No valid result.")
            results.append((0, base_yaw, 0, 999, 0, np.eye(4)))
        else:
            score, yaw_d, fit, rmse, ninl, T_icp = best
            print(f"    Best: yaw={yaw_d:.2f}°, inl={ninl}, "
                  f"fit={fit:.4f}, rmse={rmse:.3f}m, score={score:.1f}")
            # Also print the ICP transform (should be small since centered)
            print(f"    ICP T: dx={T_icp[0,3]:.3f}, dy={T_icp[1,3]:.3f}, "
                  f"dz={T_icp[2,3]:.3f}")
            results.append((score, yaw_d, fit, rmse, ninl, T_icp))

    results.sort(key=lambda x: x[0], reverse=True)

    if not results or results[0][0] <= 0:
        print("ERROR: No valid yaw found.")
        sys.exit(1)

    # ---- Step 4: Peak validation ----
    best_score = results[0][0]
    runnerup_score = results[1][0] if len(results) > 1 else 0
    peak_ratio = best_score / max(runnerup_score, 1)

    print(f"\n[4] Peak validation:")
    print(f"  Best score:     {best_score:.1f} (yaw={results[0][1]:.1f}°)")
    print(f"  Runner-up:      {runnerup_score:.1f} (yaw={results[1][1]:.1f}°)")
    print(f"  Peak ratio:     {peak_ratio:.2f}x (need ≥ 1.5x)")

    if peak_ratio < 1.5:
        print(f"  *** WARNING: Weak peak! Verify visually.")

    best = results[0]
    best_score, best_yaw_d, best_fit, best_rmse, best_ninl, T_icp_best = best

    # STRICT validation on the corner-centered ICP
    print(f"\n[5] Strict validation on corner region ...")
    MIN_CORNER_FITNESS = 0.15   # P95 corner — overlap is small fraction of corner region
    MIN_CORNER_INLIERS = 200
    MAX_CORNER_RMSE = 0.08

    ok = True
    if best_fit < MIN_CORNER_FITNESS:
        print(f"  FAIL: corner fitness {best_fit:.4f} < {MIN_CORNER_FITNESS}")
        ok = False
    if best_ninl < MIN_CORNER_INLIERS:
        print(f"  FAIL: corner inliers {best_ninl} < {MIN_CORNER_INLIERS}")
        ok = False
    if best_rmse > MAX_CORNER_RMSE:
        print(f"  FAIL: corner rmse {best_rmse:.3f} > {MAX_CORNER_RMSE}")
        ok = False

    if not ok:
        print(f"\n  *** REGISTRATION FAILED — corner region doesn't match well enough ***")
        if visualize:
            # Show the centered corner alignment
            src_vis = o3d.geometry.PointCloud(src_centered).voxel_down_sample(0.10)
            tgt_vis = o3d.geometry.PointCloud(tgt_centered).voxel_down_sample(0.10)
            src_vis.transform(T_icp_best)
            src_vis.paint_uniform_color([1.0, 0.3, 0.3])
            tgt_vis.paint_uniform_color([0.3, 0.8, 0.3])
            o3d.visualization.draw_geometries(
                [src_vis, tgt_vis, o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)],
                window_name="Corner alignment — Red=source Green=target")
        sys.exit(1)

    print(f"  ✓ All corner checks passed!")

    # Also verify that the ICP didn't drift far from origin
    icp_dx = T_icp_best[0, 3]
    icp_dy = T_icp_best[1, 3]
    icp_dz = T_icp_best[2, 3]
    icp_drift = np.sqrt(icp_dx**2 + icp_dy**2)
    print(f"  ICP drift from origin: ({icp_dx:.3f}, {icp_dy:.3f}, {icp_dz:.3f}) "
          f"mag={icp_drift:.3f}m")
    if icp_drift > 3.0:
        print(f"  WARNING: Large ICP drift ({icp_drift:.2f}m) — alignment may be wrong")

    # ---- Step 6: Compose full transform ----
    print(f"\n[6] Composing full transform ...")
    # T_full = T_origin_to_tgt @ T_icp @ T_src_to_origin
    # where T_icp includes the yaw rotation + local refinement
    T_origin_to_tgt = np.eye(4)
    T_origin_to_tgt[0, 3] = tgt_center[0]
    T_origin_to_tgt[1, 3] = tgt_center[1]

    T_full = T_origin_to_tgt @ T_icp_best @ T_src_to_origin

    # Project to 3-DOF (ensure ground flatness)
    T_final = project_3dof(T_full)
    yaw_final = np.degrees(extract_yaw(T_final))

    print(f"  Final yaw:       {yaw_final:.2f}°")
    print(f"  Final trans:     ({T_final[0,3]:.3f}, {T_final[1,3]:.3f}, {T_final[2,3]:.3f})")
    print(f"  Z-axis diag:     {T_final[2,2]:.6f} (must be ≈1.0)")

    if abs(T_final[2, 2]) < 0.999:
        print(f"  FAIL: Z-axis not preserved!")
        sys.exit(1)

    # ---- Step 7: Visualize ----
    if visualize:
        print(f"\n[7] Visual check — close window to merge ...")
        visualize_alignment(source, target, T_final)

    # ---- Step 8: Merge ----
    print(f"\n[8] Merging full-resolution clouds ...")
    src_aligned = o3d.geometry.PointCloud(source)
    src_aligned.transform(T_final)
    merged = target + src_aligned
    print(f"  Target: {len(target.points)}  |  Source: {len(source.points)}")
    print(f"  Merged: {len(merged.points)} points")
    merged = merged.voxel_down_sample(0.02)
    print(f"  After dedup (0.02m): {len(merged.points)} points")

    o3d.io.write_point_cloud(output_path, merged, write_ascii=False)
    print(f"\nSaved: {output_path}")
    return T_final


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Merge preprocessed PCD maps (corner-centered yaw search)')
    parser.add_argument('target', help='Target (reference) PCD')
    parser.add_argument('source', help='Source (to be aligned) PCD')
    parser.add_argument('output', help='Output merged PCD')
    parser.add_argument('--no-visualize', action='store_true',
                        help='Skip visualization')
    args = parser.parse_args()
    register_and_merge(args.target, args.source, args.output,
                       visualize=not args.no_visualize)
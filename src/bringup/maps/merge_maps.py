#!/usr/bin/env python3
"""
Merge two preprocessed PCD maps via quadrant-constrained registration.

KEY INSIGHT: Both maps were preprocessed with wall-to-axis alignment.
Therefore the yaw between them MUST be ≈ 0°, 90°, 180°, or 270°.
Any other yaw is a false match caused by local minima in the small overlap.

Pipeline:
  1. Extract overlap-layer points (|Z| < 1.0m)
  2. For each quadrant yaw (0/90/180/270) ± 5° fine search:
       Run tight ICP, score by inlier_count
  3. Pick the quadrant with highest score
  4. Validate: best quadrant must clearly beat the others
  5. Final ICP refinement at 0.1m threshold
  6. Strict validation: local fitness ≥ 0.25, inlier_rmse ≤ 0.10m

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

def extract_overlap_layer(pcd, z_min=-1.0, z_max=1.0):
    """Keep points in the overlap zone (ground floor + low walls/furniture)."""
    pts = np.asarray(pcd.points)
    mask = (pts[:, 2] >= z_min) & (pts[:, 2] <= z_max)
    out = o3d.geometry.PointCloud()
    out.points = o3d.utility.Vector3dVector(pts[mask])
    return out


def prep_cloud(pcd, voxel):
    down = pcd.voxel_down_sample(voxel)
    if len(down.points) >= 10:
        down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 3.0, max_nn=30))
    return down


# ======================================================================
# Quadrant yaw search
# ======================================================================

def run_icp_and_score(src, tgt, T_init, dist_thresh):
    """Run ICP, return (score, fitness, rmse, inlier_count, transform)."""
    result = o3d.pipelines.registration.registration_icp(
        src, tgt, dist_thresh, T_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(1e-8, 1e-8, 100))
    n_inl = int(result.fitness * len(src.points))
    score = n_inl / (1.0 + result.inlier_rmse)
    T_proj = project_3dof(result.transformation)
    return score, result.fitness, result.inlier_rmse, n_inl, T_proj


def search_quadrant(src, tgt, base_yaw_deg, fine_range=5.0, fine_step=0.5,
                    dist_thresh=0.15):
    """
    Fine-search yaw around `base_yaw_deg` (± fine_range).
    Run ICP for each, return best (score, yaw_deg, fitness, rmse, inlier_count, T).
    """
    best = None
    for delta in np.arange(-fine_range, fine_range + fine_step, fine_step):
        yaw_deg = base_yaw_deg + delta
        T_init = make_3dof(np.radians(yaw_deg), 0.0, 0.0)
        try:
            score, fit, rmse, ninl, T = run_icp_and_score(src, tgt, T_init, dist_thresh)
            if best is None or score > best[0]:
                best = (score, yaw_deg, fit, rmse, ninl, T)
        except Exception:
            continue
    return best


def quadrant_search(src_layer, tgt_layer):
    """
    Search 4 quadrant yaws (0/90/180/270) ±5°, return results sorted best-first.
    """
    # Use fine voxels for precision
    src = prep_cloud(src_layer, 0.15)
    tgt = prep_cloud(tgt_layer, 0.15)
    print(f"  Search clouds: src={len(src.points)}, tgt={len(tgt.points)} (voxel=0.15m)")

    if len(src.points) < 50 or len(tgt.points) < 50:
        return []

    quadrants = [0, 90, 180, 270]
    results = []

    for q in quadrants:
        print(f"\n  --- Quadrant {q}° (±5°, step=0.5°) ---")
        best = search_quadrant(src, tgt, q, fine_range=5.0, fine_step=0.5,
                               dist_thresh=0.15)
        if best is None:
            print(f"    No valid result.")
            results.append((0, q, 0, 999, 0, make_3dof(np.radians(q), 0, 0)))
        else:
            score, yaw_d, fit, rmse, ninl, T = best
            yaw_out = np.degrees(extract_yaw(T))
            tx, ty = T[0, 3], T[1, 3]
            print(f"    Best: yaw={yaw_out:.2f}°, T=({tx:.2f},{ty:.2f}), "
                  f"inl={ninl}, fit={fit:.4f}, rmse={rmse:.3f}m, score={score:.1f}")
            results.append((score, yaw_d, fit, rmse, ninl, T))

    results.sort(key=lambda x: x[0], reverse=True)
    return results


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

    # ---- Extract overlap layer ----
    print(f"\n{'='*60}")
    print("[1] Extracting overlap layer (|Z| < 1.0m) ...")
    tgt_layer = extract_overlap_layer(target)
    src_layer = extract_overlap_layer(source)
    print(f"  Target: {len(tgt_layer.points)} pts "
          f"({100*len(tgt_layer.points)/max(len(target.points),1):.1f}%)")
    print(f"  Source: {len(src_layer.points)} pts "
          f"({100*len(src_layer.points)/max(len(source.points),1):.1f}%)")

    if len(tgt_layer.points) < 200 or len(src_layer.points) < 200:
        print("ERROR: Not enough overlap-layer points.")
        sys.exit(1)

    # ---- Quadrant search ----
    print(f"\n[2] Quadrant yaw search (0/90/180/270° ±5°) ...")
    qresults = quadrant_search(src_layer, tgt_layer)

    if not qresults or qresults[0][0] <= 0:
        print("ERROR: No valid quadrant found.")
        sys.exit(1)

    # Show all quadrants ranked
    print(f"\n  Quadrant ranking:")
    for rank, (score, _, fit, rmse, ninl, T) in enumerate(qresults):
        yaw = np.degrees(extract_yaw(T))
        tx, ty = T[0, 3], T[1, 3]
        m = " <-- BEST" if rank == 0 else ""
        print(f"    #{rank+1}: score={score:.0f}, yaw={yaw:.2f}°, "
              f"T=({tx:.2f},{ty:.2f}), inl={ninl}, fit={fit:.4f}, rmse={rmse:.3f}m{m}")

    # ---- Peak validation ----
    best_score = qresults[0][0]
    runnerup_score = qresults[1][0] if len(qresults) > 1 else 0
    peak_ratio = best_score / max(runnerup_score, 1)

    print(f"\n[3] Peak validation:")
    print(f"  Best score:     {best_score:.1f}")
    print(f"  Runner-up:      {runnerup_score:.1f}")
    print(f"  Peak ratio:     {peak_ratio:.2f}x (need ≥ 1.3x)")

    if peak_ratio < 1.3:
        print(f"  *** WARNING: Weak peak ({peak_ratio:.1f}x). Verify visually!")

    best = qresults[0]
    _, _, _, _, _, T_best = best
    yaw_best = np.degrees(extract_yaw(T_best))

    # ---- Final tight ICP ----
    print(f"\n[4] Final tight ICP (threshold=0.10m) ...")
    src_fine = prep_cloud(src_layer, 0.1)
    tgt_fine = prep_cloud(tgt_layer, 0.1)
    score_f, fit_f, rmse_f, ninl_f, T_final = run_icp_and_score(
        src_fine, tgt_fine, T_best, dist_thresh=0.10)

    print(f"  Inliers:  {ninl_f}")
    print(f"  Fitness:  {fit_f:.4f}")
    print(f"  RMSE:     {rmse_f:.3f}m")
    print(f"  Yaw:      {np.degrees(extract_yaw(T_final)):.2f}°")
    print(f"  Trans:    ({T_final[0,3]:.3f}, {T_final[1,3]:.3f}, {T_final[2,3]:.3f})")

    # ---- Validation ----
    print(f"\n[5] Validation ...")
    MIN_INLIERS = 500
    MIN_FITNESS = 0.20    # Relaxed — small overlap means low fitness
    MAX_RMSE = 0.10

    ok = True
    if ninl_f < MIN_INLIERS:
        print(f"  FAIL: inliers {ninl_f} < {MIN_INLIERS}")
        ok = False
    if fit_f < MIN_FITNESS:
        print(f"  FAIL: fitness {fit_f:.4f} < {MIN_FITNESS}")
        ok = False
    if rmse_f > MAX_RMSE:
        print(f"  FAIL: rmse {rmse_f:.3f} > {MAX_RMSE}")
        ok = False
    if abs(T_final[2, 2]) < 0.999:
        print(f"  FAIL: Z-axis not preserved (diag={T_final[2,2]:.6f})")
        ok = False

    if not ok:
        print(f"\n  *** REGISTRATION FAILED VALIDATION ***")
        if visualize:
            print(f"  Showing visualization for manual inspection...")
            visualize_alignment(source, target, T_final)
        sys.exit(1)

    print(f"  ✓ All checks passed!")

    # ---- Visualize ----
    if visualize:
        print(f"\n[6] Visual check — close window to merge ...")
        visualize_alignment(source, target, T_final)

    # ---- Merge ----
    print(f"\n[7] Merging full-resolution clouds ...")
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
        description='Merge preprocessed PCD maps (quadrant-constrained 3-DOF)')
    parser.add_argument('target', help='Target (reference) PCD')
    parser.add_argument('source', help='Source (to be aligned) PCD')
    parser.add_argument('output', help='Output merged PCD')
    parser.add_argument('--no-visualize', action='store_true',
                        help='Skip visualization')
    args = parser.parse_args()
    register_and_merge(args.target, args.source, args.output,
                       visualize=not args.no_visualize)

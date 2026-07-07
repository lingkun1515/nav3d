# global_reloc — non-ROS standalone build & GUI demo

A coarse-to-fine global relocalization library for 3D LiDAR. This directory
builds **without ROS**: a `RelocFacade` API + a small ImGui GUI (`reloc_gui`).

## Build (Linux / WSL / Jetson)

```bash
sudo apt install libpcl-dev libeigen3-dev libglfw3-dev libgtest-dev
cd global_reloc
git clone --depth 1 https://github.com/ocornut/imgui.git third_party/imgui
git clone --depth 1 https://github.com/koide3/small_gicp.git third_party/small_gicp
cmake -B build -DGLOBAL_RELOC_BUILD_ROS=OFF -DGLOBAL_RELOC_BUILD_GUI=ON
cmake --build build -j
```

Binaries in `build/`:
- `reloc_gui`     — interactive demo (this is what you want to try)
- `build_map_cli` — offline map builder
- `reloc_cli`     — offline reloc from a query PCD

## Run the GUI

```bash
./build/reloc_gui
```

- **Synthetic demo** (no dataset needed): set density / scan radius / perturbation,
  click **run synthetic**. A room is built into a map, a local scan is cropped,
  perturbed, and the pipeline recovers the pose.
- **Your own data**: enter a PCD path under **Map → build from PCD**, then a
  query PCD under **Query → run on PCD**.

Visualization: gray = map, red = raw query, green = aligned (after reloc).
Left-drag rotate, right-drag pan, scroll zoom, **fit view** recenters.

## Build with ROS2 (colcon)

```bash
cd <ws>/src && git clone <this> global_reloc
colcon build --packages-select global_reloc
```
(runs the `reloc_node` ROS2 node; GUI/CLI still build alongside.)

## Algorithm

Coarse-to-fine, exploiting **gravity alignment** (map & query both z-up). Since
roll/pitch are fixed, the global unknowns reduce to **yaw + (x,y)** plus a **z**
offset — so the coarse stage can be a *deterministic global search* instead of
fragile feature matching.

- **Coarse (`BevMatcher`, default `coarse_strategy: bev`).** Rasterize the map
  to a 2D bird's-eye-view occupancy grid, take its Felzenszwalb distance
  transform → likelihood field `exp(-d²/2σ²)` (z-invariant). Brute-force yaw
  (default 2° step); per yaw, correlate the query footprint against the field to
  recover the best in-plane translation. NMS keeps top-K hypotheses; a 1D z
  overlap-scan keeps several distinct z levels per peak. Deterministic, no
  initial guess, no randomness.
- **Fine (`FineMatcher`).** small_gicp GICP refines each hypothesis to full
  6-DOF and computes inlier_ratio, **tight_inlier_ratio** (fraction within
  `tight_threshold`), and **intensity_consistency** (reflectance match of tight
  inliers).
- **Score (`Scorer`).** Reflectance consistency is the leading term — it breaks
  ties between repetitive geometric structures (same shape, different material)
  and selects the true floor among the z hypotheses. Picks the best candidate
  passing sanity filters.

The legacy `coarse_strategy: sacia` path (FPFH + SAC-IA) is retained for
comparison but is unreliable on sparse Livox data.

### Reflectance (intensity) channel — disambiguation

The query scan is near-2D (a thin slice), so geometry alone is ambiguous in
repetitive structure: several locations can match tightly. Both map and query
carry LiDAR reflectance (`intensity`); matching reflectance of the tight
correspondences disambiguates them. The map stores intensity in the `.gkey`
(`.intensity.bin`); `bag_to_query` and `reloc_cli` carry it through automatically.

### Accumulation is the dominant robustness lever

A short query (~5 frames / 0.3 s) is too sparse/ambiguous — reloc jumps between
look-alikes. **Accumulate ~2 s of scan** before relocating
(`accumulate_frames: 30`, `accumulate_max_dt: 2.5`). On a repetitive test region
this cut intermittent localization jumps from ~14/30 windows (0.6 s) to ~3/30
(2 s). Use the longer accumulation whenever the platform tolerates the motion
smear; the deterministic BEV + reflectance scoring handles the rest.

## Reliability verification

The alignment-quality metrics (`inlier_ratio`, `tight_inlier_ratio`,
`mean_residual`) and even the per-shot candidate-margin `confidence` are **not**
trustworthy correctness signals in a dense/repetitive map — wrong locations
match them too. The only ground-truth-free reliability signal that holds is
**temporal consistency**: overlapping-in-time reloc results must form a
continuous path; isolated jumps are wrong.

- **Offline method**: `scripts/verify_reliability.py` sweeps overlapping windows
  over a bag, labels each STABLE/UNSTABLE by temporal consistency, reports the
  reliable fraction and where the unstable regions are, and confirms (via a
  confidence histogram) that per-shot scores do NOT discriminate. See
  `runs/RELIABILITY.md`.
- **Online**: `reloc_node` publishes `/reloc_reliable` (true only when the
  current reloc agrees with the previous one) and `/reloc_confidence`
  (diagnostic). Set `gate_publish:=true` to withhold unreliable poses. Use
  `/reloc_reliable` to decide whether to commit `/initial_pose`.

Rule of thumb: trust a reloc only if it is temporally confirmed (≥2 consecutive
agreeing results), never on a single shot's score.



## Offline testing with a dataset (no ROS launch needed)

```bash
source /opt/ros/foxy/setup.bash && source install/setup.bash
export PATH=$(pwd)/install/global_reloc/lib/global_reloc:$PATH

# 1. Build the map (.gkey = downsampled cloud + FPFH + index) once.
build_map_cli --pcd <map.pcd> --out runs --name map --voxel 0.4

# 2. Extract query windows from a bag. Single window:
bag_to_query --bag <bag> --topic /utlidar/cloud --out runs/q.pcd --start 60 --duration 2.0
#   …or many windows in one pass (q_<start>.pcd each):
bag_to_query --bag <bag> --topic /utlidar/cloud --starts "$(seq -s' ' 0 10 390)" \
             --out-dir runs/sweep --duration 2.0

# 3. Relocate. Look for RELOC OK + low mean_residual (correct => < ~0.3 m).
reloc_cli --map runs/map.gkey --query runs/q.pcd --params config/params.yaml
#   T = [tx ty tz qx qy qz qw] ...   ALIGN inlier_ratio=.. mean_residual=..

# 4. Visualize (gray=map, red=query, green=aligned).
python3 scripts/render_reloc.py --map runs/map.pcd --query runs/q.pcd \
        --pose "<tx ty tz qx qy qz qw>" --out runs/r.png
```


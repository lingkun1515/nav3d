#!/usr/bin/env python3
"""Inspect the global_reloc dataset + map (no deps beyond numpy + rosbag2_py).

Run in the foxy container:
  python3 /ros2_ws/src/global_reloc/scripts/inspect_data.py
"""
import struct, sys, os
import numpy as np

BAG = "/ros2_ws/docker/rosbag/lidar_with_pc2"
MAP = "/ros2_ws/docker/map/map.pcd"


def parse_pcd(path):
    with open(path, "rb") as f:
        header = b""
        while True:
            line = f.readline()
            header += line
            if line.startswith(b"DATA"):
                break
        data = f.read()
    h = {}
    for line in header.split(b"\n"):
        if line.startswith(b"FIELDS"):
            h["fields"] = line.split()[1:]
        elif line.startswith(b"SIZE"):
            h["size"] = [int(x) for x in line.split()[1:]]
        elif line.startswith(b"TYPE"):
            h["type"] = line.split()[1:]
        elif line.startswith(b"COUNT"):
            h["count"] = [int(x) for x in line.split()[1:]]
        elif line.startswith(b"WIDTH"):
            h["width"] = int(line.split()[1])
        elif line.startswith(b"POINTS"):
            h["points"] = int(line.split()[1])
    n = h["points"]
    field_names = [f.decode() for f in h["fields"]]
    # build numpy dtype
    np_types = {("F", 4): "<f4", ("F", 8): "<f8", ("I", 4): "<u4", ("U", 4): "<u4"}
    dt = []
    for name, ty, sz, cnt in zip(field_names, h["type"], h["size"], h["count"]):
        npname = np_types[(ty.decode(), sz)]
        if cnt == 1:
            dt.append((name, npname))
        else:
            dt.append((name, npname, cnt))
    arr = np.frombuffer(data, dtype=np.dtype(dt))
    return arr, field_names


def stats(arr, field_names, label):
    print(f"\n=== {label} ===")
    print("fields:", field_names, "npts:", len(arr))
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float64)
    xyz = xyz[~np.isnan(xyz).any(axis=1)]
    mn, mx = xyz.min(0), xyz.max(0)
    print("min   :", np.round(mn, 2).tolist())
    print("max   :", np.round(mx, 2).tolist())
    print("extent:", np.round(mx - mn, 2).tolist(), "(x,y,z)")
    print("z range / xy-diag: %.3f" % ((mx[2]-mn[2]) / max(mx[0]-mn[0]+mx[1]-mn[1], 1e-9)))
    # ground plane: lowest z band, fit normal via SVD on a thin slice
    z = xyz[:, 2]
    lo = np.percentile(z, 5)
    ground = xyz[z < lo + 0.3]
    if len(ground) > 100:
        g = ground - ground.mean(0)
        u, s, vt = np.linalg.svd(g, full_matrices=False)
        normal = vt[2]  # smallest singular direction
        print("ground-plane normal (min-eigvec):", np.round(normal, 3).tolist(),
              " => |z-comp|=%.3f (1.0=gravity-aligned z-up)" % abs(normal[2]))
    if "intensity" in field_names:
        inten = arr["intensity"]
        print("intensity range:", float(np.nanmin(inten)), float(np.nanmax(inten)))
    return xyz


def main():
    marr, mfields = parse_pcd(MAP)
    mxyz = stats(marr, mfields, "MAP map.pcd")

    # try query
    qpath = "/ros2_ws/reloc_work/q_000.pcd"
    if os.path.exists(qpath):
        qarr, qfields = parse_pcd(qpath)
        qxyz = stats(qarr, qfields, "QUERY q_000.pcd")

    # rosbag IMU + cloud samples
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Imu, PointCloud2
        from rosidl_runtime_py.convert import message_to_ordered_dict
    except Exception as e:
        print("\n[rosbag2_py not available:", e, "]")
        return

    def read_topic(topic, msg_type, times):
        """Return messages nearest to given fractional times in (0,1)."""
        from rcl_interfaces.msg import ParameterType
        storage = rosbag2_py.StorageOptions(uri=BAG, storage_id="sqlite3")
        cdr = rosbag2_py.ConverterOptions("cdr", "cdr")
        reader = rosbag2_py.SequentialReader()
        reader.open(storage, cdr)
        try:
            reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
        except Exception:
            pass
        out = []
        n = 0
        while reader.has_next():
            t, data, _ = reader.read_next()
            msg = deserialize_message(data, msg_type)
            out.append((t, msg))
            n += 1
            if n > 80000:
                break
        return out

    print("\n=== IMU /utlidar/imu samples ===")
    imus = read_topic("/utlidar/imu", Imu, [0.0, 0.5, 1.0])
    if imus:
        accs = []
        for t, m in imus[::max(1, len(imus)//30)]:
            a = m.linear_acceleration
            accs.append([a.x, a.y, a.z])
            if len(accs) <= 5:
                print(f"t={t/1e9:.2f} acc=({a.x:+.3f},{a.y:+.3f},{a.z:+.3f}) orient_cov0={m.orientation_covariance[0] if m.orientation_covariance else 'NA'}")
        accs = np.array(accs)
        print("n_imu:", len(imus), "duration s:", (imus[-1][0]-imus[0][0])/1e9)
        print("mean linear_accel:", np.round(accs.mean(0), 3).tolist())
        g = accs.mean(0)
        print("norm:", float(np.linalg.norm(g)))
        print("=> gravity direction (mean acc, sign of g):", np.round(g/np.linalg.norm(g), 3).tolist())

    print("\n=== Cloud /utlidar/cloud samples ===")
    clouds = read_topic("/utlidar/cloud", PointCloud2, [0.0, 0.5, 1.0])
    if clouds:
        print("n_clouds:", len(clouds))
        # field layout of first
        m = clouds[0][1]
        print("frame0: height=%d width=%d point_step=%d row_step=%d is_dense=%d" % (
            m.height, m.width, m.point_step, m.row_step, m.is_dense))
        print("frame0 fields:", [(f.name, f.datatype, f.offset) for f in m.fields])
        print("frame0 total pts:", m.height * m.width)
        # timestamps span
        t0 = clouds[0][0]; tN = clouds[-1][0]
        print("cloud duration s:", (tN-t0)/1e9, "rate Hz:", len(clouds)/((tN-t0)/1e9))


if __name__ == "__main__":
    main()

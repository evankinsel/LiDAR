import time

import numpy as np
import open3d as o3d
import serial

SERIAL_PORT = "COM3"
BAUD_RATE = 115200
VOXEL_SIZE_METERS = 0.02
TARGET_COUNT = 50000
UPDATE_INTERVAL_SECONDS = 0.20
OUTPUT_FILE = "captured_point_cloud.ply"
REQUIRE_FULL_CALIBRATION = False


def parse_point(line):
    fields = line.strip().split(",")

    if len(fields) != 14 or fields[0] != "POINT":
        return None

    try:
        x_mm = float(fields[7])
        y_mm = float(fields[8])
        z_mm = float(fields[9])
        system_calibration = int(fields[10])
        gyro_calibration = int(fields[11])
        accel_calibration = int(fields[12])
        magnetometer_calibration = int(fields[13])
    except ValueError:
        return None

    if REQUIRE_FULL_CALIBRATION and min(
        system_calibration,
        gyro_calibration,
        accel_calibration,
        magnetometer_calibration,
    ) < 3:
        return None

    return np.array([x_mm, y_mm, z_mm], dtype=np.float64) / 1000.0


def downsample(points):
    raw_cloud = o3d.geometry.PointCloud()
    raw_cloud.points = o3d.utility.Vector3dVector(points)

    cloud = raw_cloud.voxel_down_sample(voxel_size=VOXEL_SIZE_METERS)
    cloud_points = np.asarray(cloud.points)

    if len(cloud_points) <= TARGET_COUNT:
        return cloud

    indices = np.linspace(0, len(cloud_points) - 1, TARGET_COUNT, dtype=int)
    limited_cloud = o3d.geometry.PointCloud()
    limited_cloud.points = o3d.utility.Vector3dVector(cloud_points[indices])
    return limited_cloud


def main():
    serial_connection = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
    time.sleep(2)
    serial_connection.reset_input_buffer()

    visualizer = o3d.visualization.Visualizer()
    visualizer.create_window(
        window_name="ESP32 3D Point Cloud",
        width=1000,
        height=700,
    )

    display_cloud = o3d.geometry.PointCloud()
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.20)

    visualizer.add_geometry(display_cloud)
    visualizer.add_geometry(coordinate_frame)
    visualizer.get_render_option().point_size = 3.0

    captured_points = []
    last_update = time.monotonic()

    print(f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud")
    print("Close the Open3D window or press Ctrl+C to save the cloud.")

    try:
        while visualizer.poll_events():
            raw_line = serial_connection.readline().decode("utf-8", errors="ignore")
            point = parse_point(raw_line)

            if point is not None:
                captured_points.append(point)

            now = time.monotonic()

            if captured_points and now - last_update >= UPDATE_INTERVAL_SECONDS:
                display_cloud = downsample(np.asarray(captured_points))
                visualizer.clear_geometries()
                visualizer.add_geometry(display_cloud)
                visualizer.add_geometry(coordinate_frame)
                visualizer.update_renderer()
                print(
                    f"Raw points: {len(captured_points)} | "
                    f"Displayed points: {len(display_cloud.points)}"
                )
                last_update = now

    except KeyboardInterrupt:
        pass

    finally:
        serial_connection.close()
        visualizer.destroy_window()

    if captured_points:
        final_cloud = downsample(np.asarray(captured_points))
        o3d.io.write_point_cloud(OUTPUT_FILE, final_cloud)
        print(f"Saved {len(final_cloud.points)} points to {OUTPUT_FILE}")
    else:
        print("No valid POINT data was captured.")


if __name__ == "__main__":
    main()

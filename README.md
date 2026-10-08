# ESP32-Based VL53L0X LiDAR 2D & 3D Scanner

**Work in progress**

An experimental desktop LiDAR project built around an ESP32 and a VL53L0X time-of-flight distance sensor. The project is developing from basic distance measurement into 2D scanning and orientation-aware 3D point-cloud capture.

## Current Status

- ESP32 firmware for VL53L0X distance measurements
- OLED output for the 2D scanner prototype
- 2D polar-coordinate conversion
- BNO055-based orientation-aware 3D firmware prototype
- Live serial connection from the ESP32 to Python
- Open3D point-cloud visualization, voxel downsampling, and PLY export

## Hardware

- ESP32 development board
- VL53L0X time-of-flight distance sensor
- SSD1306 OLED display for the 2D prototype
- BNO055 IMU for orientation-aware 3D capture
- Breadboard, jumper wires, and a future pan/tilt or motorized mounting system

## Project Files

| File | Purpose |
| --- | --- |
| `LIDAR_2D_Scanner.ino` | Sweeps a sensor mounted to a servo and outputs 2D Cartesian scan points. |
| `LIDAR_3D_ManualScan.ino` | Manual pan/tilt coordinate-capture prototype. |
| `LIDAR_3D_BNO055.ino` | Reads VL53L0X range data and BNO055 orientation data to output orientation-aware XYZ points. |
| `live_point_cloud.py` | Reads ESP32 serial data, renders the cloud live in Open3D, downsamples it, and saves a PLY file. |

## Setup

Install the required Arduino libraries:

- Adafruit VL53L0X
- Adafruit SSD1306
- Adafruit GFX Library
- Adafruit BNO055
- Adafruit Unified Sensor
- ESP32Servo for the 2D servo-based scanner

Install the Python dependencies:

```bash
pip install open3d pyserial numpy
```

Set the correct ESP32 serial port in `live_point_cloud.py` before running it:

```python
SERIAL_PORT = "COM3"
```

Then run:

```bash
python live_point_cloud.py
```

## Current Limitation

The BNO055 provides the orientation of the sensor, allowing points to be rotated into a shared coordinate frame while the board is tilted or turned. It does not reliably determine the board's changing position in a room. Moving the full device during a scan therefore changes the unknown origin of later points.

## Roadmap

- Build and test the physical scanner structure
- Add controlled pan/tilt motion or a motorized scanning mount
- Calibrate the LiDAR and IMU alignment
- Improve point filtering and visualization
- Add position tracking or scan registration for multi-position mapping
- Develop a complete 2D and 3D desktop scanning workflow

## End Goal

The end goal is a functional desktop 3D scanner built from scratch, with custom ESP32 firmware and Python software that captures, processes, saves, and visualizes LiDAR-derived point clouds.

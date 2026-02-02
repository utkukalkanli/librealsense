# Intel RealSense Tools - macOS Build

> **Note:** All commands require `sudo` on macOS due to USB access permissions.

## Quick Start
```bash
cd /Users/utkukalkanli/Developer/Tez/librealsense
sudo ./build/Release/rs-pointcloud    # Real-time 3D viewer
```

---

## Visualization & Capture Tools

| Tool | Description | Command |
|------|-------------|---------|
| `rs-pointcloud` | **Real-time 3D point cloud viewer** with mouse rotation | `sudo ./build/Release/rs-pointcloud` |
| `rs-capture` | Live camera preview with depth colorization | `sudo ./build/Release/rs-capture` |
| `rs-multicam` | Multi-camera support and viewing | `sudo ./build/Release/rs-multicam` |
| `rs-depth-quality` | Depth accuracy analysis tool | `sudo ./build/Release/rs-depth-quality` |
| `rs-measure` | Distance measurement tool | `sudo ./build/Release/rs-measure` |
| `rs-hdr` | HDR (High Dynamic Range) demo | `sudo ./build/Release/rs-hdr` |

---

## Recording & Playback

| Tool | Description | Command |
|------|-------------|---------|
| `rs-record` | Record depth/color streams to `.bag` file | `sudo ./build/Release/rs-record -t 10 -f output.bag` |
| `rs-record-playback` | Record with live playback GUI | `sudo ./build/Release/rs-record-playback` |
| `rs-convert` | Convert `.bag` files to PLY/PNG/CSV | `sudo ./build/Release/rs-convert -i input.bag -p output_prefix` |
| `rs-rosbag-inspector` | Inspect contents of `.bag` files | `sudo ./build/Release/rs-rosbag-inspector input.bag` |

---

## Basic Examples

| Tool | Description | Command |
|------|-------------|---------|
| `rs-hello-realsense` | Minimal depth streaming example | `sudo ./build/Release/rs-hello-realsense` |
| `rs-enumerate-devices` | List connected cameras and capabilities | `sudo ./build/Release/rs-enumerate-devices` |
| `rs-color` | Basic color stream (C API) | `sudo ./build/Release/rs-color` |
| `rs-depth` | Basic depth stream (C API) | `sudo ./build/Release/rs-depth` |
| `rs-distance` | Print center pixel distance (C API) | `sudo ./build/Release/rs-distance` |
| `rs-infrared` | Infrared stream viewer | `sudo ./build/Release/rs-infrared` |
| `rs-save-to-disk` | Save frames to disk | `sudo ./build/Release/rs-save-to-disk` |

---

## Processing & Alignment

| Tool | Description | Command |
|------|-------------|---------|
| `rs-align` | Align depth to color (basic) | `sudo ./build/Release/rs-align` |
| `rs-align-advanced` | Advanced alignment with settings | `sudo ./build/Release/rs-align-advanced` |
| `rs-post-processing` | Demonstrate depth filters | `sudo ./build/Release/rs-post-processing` |
| `rs-embedded-filters` | On-camera filter demo | `sudo ./build/Release/rs-embedded-filters` |
| `rs-labeled-pointcloud` | Point cloud with segmentation labels | `sudo ./build/Release/rs-labeled-pointcloud` |

---

## Advanced & Calibration

| Tool | Description | Command |
|------|-------------|---------|
| `rs-on-chip-calib` | On-chip calibration | `sudo ./build/Release/rs-on-chip-calib` |
| `rs-sensor-control` | Direct sensor control | `sudo ./build/Release/rs-sensor-control` |
| `rs-motion` | IMU/gyro demonstration | `sudo ./build/Release/rs-motion` |
| `rs-software-device` | Software device emulator | `sudo ./build/Release/rs-software-device` |
| `rs-callback` | Async callback demo | `sudo ./build/Release/rs-callback` |
| `rs-benchmark` | Performance benchmarking | `sudo ./build/Release/rs-benchmark` |
| `rs-data-collect` | Data collection utility | `sudo ./build/Release/rs-data-collect` |

---

## Firmware & System

| Tool | Description | Command |
|------|-------------|---------|
| `rs-fw-update` | Update camera firmware | `sudo ./build/Release/rs-fw-update` |
| `rs-fw-logger` | Firmware debug logger | `sudo ./build/Release/rs-fw-logger` |
| `rs-terminal` | Debug terminal | `sudo ./build/Release/rs-terminal` |
| `rs-eth-config` | Ethernet configuration | `sudo ./build/Release/rs-eth-config` |
| `rs-embed` | Firmware embedding tool | `sudo ./build/Release/rs-embed` |

---

## Custom Python Scripts

| Script | Description | Command |
|--------|-------------|---------|
| `advanced_scan.py` | High-quality scan with filters (High Density preset, Temporal + Spatial filters) | `python3 advanced_scan.py -o scan.ply -t 5` |
| `visualize_ply.py` | View PLY files with optional depth threshold | `python3 visualize_ply.py -t 0.8` |

---

## ⚠️ macOS Limitations

- **`realsense-viewer`** (the full GUI application) is **not supported on macOS** by Intel. This is hardcoded in CMakeLists.txt.
- Alternative: Use `rs-pointcloud` for real-time 3D viewing.

---

## Common Workflows

### 3D Scanning Workflow
```bash
# 1. Record a scan (10 seconds)
sudo ./build/Release/rs-record -t 10 -f my_scan.bag

# 2. Convert to PLY files
sudo ./build/Release/rs-convert -i my_scan.bag -p scan_output/frame

# 3. Visualize
python3 visualize_ply.py scan_output/frame_0.ply
```

### Check Device Status
```bash
sudo ./build/Release/rs-enumerate-devices -s
```

# Raspberry Pi 4 Headless Setup for RealSense D455

Complete guide for setting up a headless Raspberry Pi 4 with Intel RealSense D455 camera.

## Hardware Requirements

- Raspberry Pi 4 (4GB or 8GB RAM recommended)
- 32GB+ microSD card (Class 10 or faster)
- USB-C power supply (5V 3A - important!)
- RealSense D455 camera
- Ethernet cable or WiFi credentials

## Step 1: Flash Raspberry Pi OS

1. Download [Raspberry Pi Imager](https://www.raspberrypi.com/software/)
2. Choose **Raspberry Pi OS (64-bit)** - Lite version for headless
3. Click the **gear icon** ⚙️ before flashing to configure:
   - Enable SSH
   - Set username/password
   - Configure WiFi (optional)
   - Set locale/timezone
4. Flash to SD card

## Step 2: First Boot & SSH

```bash
# Find your Pi's IP (check router or use)
ping raspberrypi.local

# SSH into Pi
ssh pi@raspberrypi.local
# or
ssh pi@<IP_ADDRESS>
```

## Step 3: System Updates

```bash
sudo apt update && sudo apt upgrade -y
sudo reboot
```

## Step 4: Install Dependencies

```bash
# Essential build tools
sudo apt install -y git cmake build-essential

# librealsense dependencies
sudo apt install -y libssl-dev libusb-1.0-0-dev pkg-config
sudo apt install -y libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

# Python dependencies
sudo apt install -y python3-dev python3-pip python3-numpy
pip3 install open3d
```

## Step 5: Build librealsense from Source

```bash
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense

# Setup udev rules (no sudo needed for camera after this!)
./scripts/setup_udev_rules.sh

mkdir build && cd build

# Configure with Python bindings
cmake .. \
    -DBUILD_PYTHON_BINDINGS=ON \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_GRAPHICAL_EXAMPLES=OFF

# Build (takes 30-60 minutes on Pi 4)
make -j3

# Install
sudo make install
sudo ldconfig
```

## Step 6: Configure Python Path

```bash
# Add to ~/.bashrc
echo 'export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.*/dist-packages' >> ~/.bashrc
source ~/.bashrc
```

## Step 7: Test Camera

```bash
# Reboot to apply udev rules
sudo reboot

# After reboot, test (no sudo needed!)
python3 -c "import pyrealsense2 as rs; ctx = rs.context(); print(f'Found {len(ctx.devices)} device(s)')"
```

## Step 8: Copy Your Scripts

From your Mac:
```bash
# Copy scripts to Pi
scp advanced_scan.py hq_capture.py filter_pointcloud.py compare_to_cad.py visualize_ply.py pi@raspberrypi.local:~/
```

## Step 9: Enable High Accuracy Mode (Linux Only!)

On the Pi, you can now use the full camera features:

```python
# In your scripts, these now work on Linux:
depth_sensor.set_option(rs.option.visual_preset, 4)  # High Accuracy
depth_sensor.set_option(rs.option.laser_power, max_power)  # Max laser
```

## Usage

```bash
# SSH into Pi
ssh pi@raspberrypi.local

# Run high-quality capture
python3 hq_capture.py -o scan.ply --stack-count 30 --min-depth 0.4 --max-depth 0.8

# Copy result back to Mac
# On Mac:
scp pi@raspberrypi.local:~/scan.ply .
```

## Tips for Headless Operation

### Auto-start scanning on boot
```bash
# Create a systemd service (optional)
sudo nano /etc/systemd/system/scanner.service
```

### Remote file transfer
```bash
# Use rsync for efficient transfers
rsync -avz pi@raspberrypi.local:~/scans/ ./local_scans/
```

### Check camera status remotely
```bash
ssh pi@raspberrypi.local 'python3 -c "import pyrealsense2 as rs; print(len(rs.context().devices))"'
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| "No device connected" | Check USB 3.0 port (blue), power supply |
| Build fails | Ensure 4GB+ RAM, use `make -j2` instead |
| Camera not detected | Run `./scripts/setup_udev_rules.sh` again |
| Low power warning | Use official 5V 3A power supply |

## Performance Notes

- **Frame stacking:** Reduce to 30 frames (vs 60 on Mac) for speed
- **Resolution:** 1280x720 works well, 848x480 for faster processing
- **Processing:** Point cloud operations are slower; process heavy tasks on Mac

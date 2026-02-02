# Useful librealsense Commands (macOS)

> **Note:** On macOS, direct access to the USB device often requires root privileges. Run these commands with `sudo`.

## 1. Device Verification
Check if the camera is connected and recognized.
```bash
sudo ./build/Release/rs-enumerate-devices
```
For headers and more details:
```bash
sudo ./build/Release/rs-enumerate-devices -s
```

## 2. Basic Streaming Test
Verify that the camera can actually stream data (text-based output).
```bash
sudo ./build/Release/rs-hello-realsense
```

## 3. Visualizations
Open a graphical window to view the camera streams.

**2D Stream (Depth & Color):**
```bash
sudo ./build/Release/rs-capture
```

**3D Point Cloud:**
```bash
sudo ./build/Release/rs-pointcloud
```

## Troubleshooting
If you see "No device connected" or errors after running one command:

1. **Unplug and replug the camera** (This is often required if a program crashed or didn't close clean).
2. Run `sudo killall rs-enumerate-devices` (or other tools) to ensure no background processes are holding the lock.
3. Always use `sudo` on macOS.

If issues persist, check the firmware version using `rs-enumerate-devices`.

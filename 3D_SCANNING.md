# 3D Scanning Workflow

This guide explains how to capture a 3D scan of an object using `rs-record` and `rs-convert`.

## Step 1: Record the Scan (`.bag`)

Use `rs-record` to save the camera stream to a file. This preserves all depth and color data exactly as it was seen.

1.  **Prepare your setup:** Place your object on the turntable.
2.  **Run the recorder:**
    ```bash
    # Records for 15 seconds (adjust -t as needed for one rotation)
    sudo ./build/Release/rs-record -t 15 -f scan.bag
    ```
    *Note: Press `Ctrl+C` to stop early if needed.*

## Step 2: Convert to Point Cloud (`.ply`)

Use `rs-convert` to extract the 3D data from the recording into `.ply` files.

1.  **Create an output folder:**
    ```bash
    mkdir scan_output
    ```
2.  **Run the converter:**
    ```bash
    sudo ./build/Release/rs-convert -i scan.bag -l scan_output/frame
    ```
    *Flags:*
    *   `-i scan.bag`: Input recording.
    *   `-l scan_output/frame`: Output path prefix for PLY files.

## Step 3: View the Results

You will now have a sequence of `.ply` files (e.g., `frame_0.ply`, `frame_1.ply`, etc.) in the `scan_output` folder.

### Option 2: Use the Python Script
We have created a helper script `visualize_ply.py` that uses Open3D to view the files. It can also **clean the background** on the fly!

1.  **View the most recent scan:**
    ```bash
    python3 visualize_ply.py
    ```

2.  **View with Background Removal (e.g., 0.6 meters):**
    ```bash
    python3 visualize_ply.py -t 0.6
    ```
    *This will hide everything further than 60cm away.*

3.  **View a specific file:**
    ```bash
    python3 visualize_ply.py test_output/frame_12345.ply
    ```

### Option 3: External Software
You can view these files in standard 3D software like:
*   **MeshLab** (Free, Open Source)
*   **CloudCompare**
*   **Blender**

## Best Practices for Scanning
To get a clean scan (object only, no background):

1.  **Background:** A **black, matte (non-shiny) background** is excellent.
    *   RealSense cameras often treat black, light-absorbing surfaces as "invalid depth," meaning they naturally disappear from the point cloud.
    *   Avoid shiny/reflective fabrics.
2.  **Distance:** Keep the background far away if possible. You can filter out distant points later using software.
3.  **Lighting:** Ensure the object is well-lit, but avoid direct reflections into the camera.
4.  **Stability:** The camera must remain perfectly still; only the turntable should move.

## Advanced: Automated Cleaning
We can write scripts to automatically remove points that are too far away (background removal).


## Troubleshooting Quality: "Why are there black holes?"
If you see black (empty) spots on your object in the 3D view:

1.  **Too Close:** The D455 camera has a minimum distance of **~40cm (0.4m)**. If the object is closer than that, the camera cannot see it (it's in the blind spot). Move it back!
2.  **Shiny/Black Surfaces:** The camera uses infrared light.
    *   **Black objects** absorb the light (no return signal).
    *   **Shiny/Transparent objects** (glass, metal) reflect the light away.
    *   *Fix:* Coat shiny objects with baby powder or scanning spray.
3.  **Shadows (Occlusion):** The depth sensor and the color camera are side-by-side. Sometimes the camera sees "around a corner" that the depth sensor can't reach, creating a "shadow" with no depth data.

## How to Minimize Shadows

Since "shadows" are caused by the camera's physical viewing angle (one eye sees it, the other doesn't), you physically cannot fix it in a single frame. **You must move the object.**

1.  **Use the Turntable:** Only the turntable allows you to fill these holes.
    *   Take a scan.
    *   Rotate the object 45 degrees.
    *   Take another scan.
    *   *Later:* Merge these scans in software like MeshLab.
2.  **Camera Height:** Position the camera slightly **above** the object, looking down. This reduces shadows blocked by the object's own shape.
3.  **Multiple Heights:** If possible, scan once with the camera low and once with the camera high.



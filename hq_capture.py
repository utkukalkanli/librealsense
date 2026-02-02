#!/usr/bin/env python3
"""
High Quality Single Frame Capture with Frame Stacking
Captures multiple frames at the same position and averages them for higher quality.

Usage:
    sudo python3 hq_capture.py -o high_quality.ply --stack-count 15
"""

import sys
import os
import time
import argparse
import numpy as np

# Add build/Release to path for pyrealsense2
script_dir = os.path.dirname(os.path.abspath(__file__))
build_lib_path = os.path.join(script_dir, "build/Release")
if build_lib_path not in sys.path:
    sys.path.append(build_lib_path)

try:
    import pyrealsense2 as rs
except ImportError:
    print("Error: Could not import pyrealsense2.")
    print("Make sure you built with -DBUILD_PYTHON_BINDINGS=ON")
    sys.exit(1)

try:
    import open3d as o3d
except ImportError:
    print("Error: Open3D not installed. Run: pip3 install open3d")
    sys.exit(1)


def capture_averaged_frame(pipeline, align, spatial, temporal, stack_count, depth_intrinsics):
    """Capture multiple frames and average the depth values."""
    
    print(f"Capturing {stack_count} frames for averaging...")
    
    depth_stack = []
    color_stack = []
    
    for i in range(stack_count):
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        
        if not depth_frame or not color_frame:
            continue
        
        # Apply filters
        filtered_depth = spatial.process(depth_frame)
        filtered_depth = temporal.process(filtered_depth)
        
        # Get numpy arrays
        depth_image = np.asanyarray(filtered_depth.get_data()).astype(np.float32)
        color_image = np.asanyarray(color_frame.get_data()).astype(np.float32)
        
        depth_stack.append(depth_image)
        color_stack.append(color_image)
        
        print(f"  Frame {i+1}/{stack_count}", end='\r')
    
    print()
    
    if len(depth_stack) == 0:
        return None, None
    
    # Stack and compute median (more robust than mean for outliers)
    depth_array = np.stack(depth_stack, axis=0)
    color_array = np.stack(color_stack, axis=0)
    
    # Use median for depth (handles outliers/noise better)
    averaged_depth = np.median(depth_array, axis=0).astype(np.uint16)
    # Use mean for color
    averaged_color = np.mean(color_array, axis=0).astype(np.uint8)
    
    # Count valid measurements per pixel (for quality assessment)
    valid_counts = np.sum(depth_array > 0, axis=0)
    print(f"Average valid measurements per pixel: {np.mean(valid_counts):.1f}")
    
    return averaged_depth, averaged_color


def create_point_cloud_from_arrays(depth_image, color_image, depth_intrinsics):
    """Convert numpy depth+color arrays to Open3D point cloud."""
    
    # Create Open3D images
    o3d_depth = o3d.geometry.Image(depth_image)
    o3d_color = o3d.geometry.Image(color_image)
    
    # Create RGBD image
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d_color, o3d_depth,
        depth_scale=1000.0,  # D455 uses mm
        depth_trunc=3.0,
        convert_rgb_to_intensity=False
    )
    
    # Camera intrinsics
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        depth_intrinsics.width,
        depth_intrinsics.height,
        depth_intrinsics.fx,
        depth_intrinsics.fy,
        depth_intrinsics.ppx,
        depth_intrinsics.ppy
    )
    
    # Create point cloud
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
    
    return pcd


def filter_by_depth(pcd, min_depth, max_depth):
    """Remove points outside the depth range."""
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors) if pcd.has_colors() else None
    
    mask = (points[:, 2] >= min_depth) & (points[:, 2] <= max_depth)
    
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(points[mask])
    if colors is not None:
        filtered_pcd.colors = o3d.utility.Vector3dVector(colors[mask])
    
    return filtered_pcd


def remove_statistical_outliers(pcd, nb_neighbors=20, std_ratio=2.0):
    """Remove isolated noise points."""
    if len(pcd.points) < nb_neighbors:
        return pcd
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    return pcd.select_by_index(ind)


def main():
    parser = argparse.ArgumentParser(
        description="High Quality Capture with Frame Stacking",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  Basic high quality capture:
    sudo python3 hq_capture.py -o scan.ply

  Extra high quality (more frames):
    sudo python3 hq_capture.py -o scan.ply --stack-count 30

  With depth filtering:
    sudo python3 hq_capture.py -o scan.ply --min-depth 0.4 --max-depth 0.8
        """
    )
    
    parser.add_argument("-o", "--output", default="hq_scan.ply", help="Output PLY file")
    parser.add_argument("--stack-count", type=int, default=15, help="Number of frames to average (default: 15)")
    parser.add_argument("--min-depth", type=float, default=0.1, help="Minimum depth in meters")
    parser.add_argument("--max-depth", type=float, default=2.0, help="Maximum depth in meters")
    parser.add_argument("--no-visualize", action="store_true", help="Skip visualization after capture")
    
    args = parser.parse_args()
    
    print("=" * 50)
    print("HIGH QUALITY FRAME STACKING CAPTURE")
    print("=" * 50)
    print(f"Stack count: {args.stack_count} frames")
    print(f"Depth range: {args.min_depth}m - {args.max_depth}m")
    print()
    
    # Configure pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    # Max resolution
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
    
    # Start pipeline
    profile = pipeline.start(config)
    
    print("Using default camera settings (macOS compatibility)")
    
    # Get intrinsics
    depth_profile = profile.get_stream(rs.stream.depth)
    depth_intrinsics = depth_profile.as_video_stream_profile().get_intrinsics()
    
    # Filters
    spatial = rs.spatial_filter()
    spatial.set_option(rs.option.filter_magnitude, 2)
    spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
    spatial.set_option(rs.option.filter_smooth_delta, 20)
    
    temporal = rs.temporal_filter()
    temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
    temporal.set_option(rs.option.filter_smooth_delta, 20)
    
    # Align to color
    align = rs.align(rs.stream.color)
    
    # Warm up the camera and temporal filter
    print("Warming up camera (3 seconds)...")
    for _ in range(90):  # 3 seconds at 30fps
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        depth = aligned.get_depth_frame()
        if depth:
            depth = spatial.process(depth)
            depth = temporal.process(depth)
    
    print("Camera ready. Keep object STILL during capture!")
    print()
    
    # Capture with stacking
    averaged_depth, averaged_color = capture_averaged_frame(
        pipeline, align, spatial, temporal, args.stack_count, depth_intrinsics
    )
    
    pipeline.stop()
    
    if averaged_depth is None:
        print("Error: No valid frames captured!")
        return
    
    # Create point cloud
    print("Creating point cloud...")
    pcd = create_point_cloud_from_arrays(averaged_depth, averaged_color, depth_intrinsics)
    
    # Filter
    pcd = filter_by_depth(pcd, args.min_depth, args.max_depth)
    pcd = remove_statistical_outliers(pcd)
    
    print(f"Final point count: {len(pcd.points)}")
    
    # Save
    print(f"Saving to {args.output}...")
    o3d.io.write_point_cloud(args.output, pcd)
    print("Done!")
    
    # Visualize
    if not args.no_visualize:
        print("Opening viewer...")
        o3d.visualization.draw_geometries([pcd])


if __name__ == "__main__":
    main()

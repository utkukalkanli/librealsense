#!/usr/bin/env python3
"""
Advanced 3D Scanning Script for Intel RealSense D455
Optimized for turntable scanning with background filtering and frame merging.

Usage:
    python3 advanced_scan.py --turntable --duration 30 --min-depth 0.4 --max-depth 1.0 -o scan.ply
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


def create_point_cloud_from_frames(depth_frame, color_frame, depth_intrinsics):
    """Convert RealSense depth+color frames to Open3D point cloud."""
    
    # Get numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    
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
    """Remove points outside the depth range (Z-axis)."""
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors) if pcd.has_colors() else None
    
    # Filter by Z (depth)
    mask = (points[:, 2] >= min_depth) & (points[:, 2] <= max_depth)
    
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(points[mask])
    if colors is not None:
        filtered_pcd.colors = o3d.utility.Vector3dVector(colors[mask])
    
    return filtered_pcd


def filter_by_bounding_box(pcd, bbox_min, bbox_max):
    """Keep only points within the specified 3D bounding box."""
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors) if pcd.has_colors() else None
    
    mask = (
        (points[:, 0] >= bbox_min[0]) & (points[:, 0] <= bbox_max[0]) &
        (points[:, 1] >= bbox_min[1]) & (points[:, 1] <= bbox_max[1]) &
        (points[:, 2] >= bbox_min[2]) & (points[:, 2] <= bbox_max[2])
    )
    
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


def register_point_clouds(source, target, voxel_size=0.005):
    """Register source to target using ICP."""
    
    # Downsample for faster registration
    source_down = source.voxel_down_sample(voxel_size)
    target_down = target.voxel_down_sample(voxel_size)
    
    # Estimate normals
    source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    target_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    
    # ICP registration
    threshold = voxel_size * 1.5
    trans_init = np.eye(4)
    
    reg = o3d.pipelines.registration.registration_icp(
        source_down, target_down, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane()
    )
    
    return reg.transformation


def turntable_scan(args):
    """Perform a turntable scan with multiple frames and merge."""
    
    print("=" * 50)
    print("TURNTABLE SCANNING MODE")
    print("=" * 50)
    print(f"Duration: {args.duration} seconds")
    print(f"Depth range: {args.min_depth}m - {args.max_depth}m")
    print(f"Frame interval: {args.frame_interval}s")
    print()
    
    # Configure pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    # Higher resolution for better detail
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
    
    # Start pipeline
    profile = pipeline.start(config)
    
    # NOTE: On macOS, changing sensor settings (visual preset, laser power) 
    # mid-stream causes segfaults or stream crashes. 
    # Using default camera settings which work well for most cases.
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
    
    # Warm up (shorter, no sensor config to wait for)
    print("Warming up camera...")
    for _ in range(15):
        pipeline.wait_for_frames()
    
    # Collect frames
    collected_clouds = []
    start_time = time.time()
    last_capture = 0
    frame_count = 0
    
    print(f"\nCapturing for {args.duration} seconds...")
    print("(Rotate your turntable now!)")
    print()
    
    while time.time() - start_time < args.duration:
        elapsed = time.time() - start_time
        
        # Capture at intervals
        if elapsed - last_capture >= args.frame_interval:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            
            depth_frame = aligned.get_depth_frame()
            color_frame = aligned.get_color_frame()
            
            if not depth_frame or not color_frame:
                continue
            
            # Apply filters
            filtered_depth = spatial.process(depth_frame)
            filtered_depth = temporal.process(filtered_depth)
            
            # Create point cloud
            pcd = create_point_cloud_from_frames(filtered_depth, color_frame, depth_intrinsics)
            
            if len(pcd.points) == 0:
                continue
            
            # Filter by depth
            pcd = filter_by_depth(pcd, args.min_depth, args.max_depth)
            
            # Filter by height (Y axis) to exclude turntable
            if args.min_height is not None:
                points = np.asarray(pcd.points)
                colors = np.asarray(pcd.colors) if pcd.has_colors() else None
                # In camera frame, Y is typically down, so we filter for Y < -min_height
                mask = points[:, 1] < -args.min_height  # Above the turntable
                filtered_pcd = o3d.geometry.PointCloud()
                filtered_pcd.points = o3d.utility.Vector3dVector(points[mask])
                if colors is not None:
                    filtered_pcd.colors = o3d.utility.Vector3dVector(colors[mask])
                pcd = filtered_pcd
            
            # Optional: bounding box
            if args.bbox_size:
                half = args.bbox_size / 2
                # Centered at depth center
                center_z = (args.min_depth + args.max_depth) / 2
                bbox_min = [-half, -half, args.min_depth]
                bbox_max = [half, half, args.max_depth]
                pcd = filter_by_bounding_box(pcd, bbox_min, bbox_max)
            
            # Remove outliers
            pcd = remove_statistical_outliers(pcd)
            
            if len(pcd.points) > 100:
                collected_clouds.append(pcd)
                frame_count += 1
                print(f"  Frame {frame_count}: {len(pcd.points)} points")
                
                # Save individual frame if requested
                if args.save_frames:
                    # Create frames directory if it doesn't exist
                    frames_dir = args.frames_dir or "frames"
                    os.makedirs(frames_dir, exist_ok=True)
                    frame_path = os.path.join(frames_dir, f"frame_{frame_count:03d}.ply")
                    o3d.io.write_point_cloud(frame_path, pcd)
                    print(f"    Saved: {frame_path}")
            
            last_capture = elapsed
        else:
            # Just consume frames to keep temporal filter updated
            pipeline.wait_for_frames()
    
    pipeline.stop()
    
    print(f"\nCaptured {len(collected_clouds)} frames")
    
    if len(collected_clouds) == 0:
        print("Error: No valid frames captured!")
        return
    
    # Merge point clouds using ICP
    print("\nMerging point clouds with ICP registration...")
    
    merged = collected_clouds[0]
    
    for i in range(1, len(collected_clouds)):
        print(f"  Registering frame {i+1}/{len(collected_clouds)}...")
        
        try:
            # Register current frame to merged cloud
            transform = register_point_clouds(collected_clouds[i], merged)
            
            # Transform and merge
            collected_clouds[i].transform(transform)
            merged += collected_clouds[i]
        except Exception as e:
            print(f"    Warning: Registration failed for frame {i+1}: {e}")
            # Still add the points
            merged += collected_clouds[i]
        
        # Downsample periodically to keep size manageable
        if i % 5 == 0:
            merged = merged.voxel_down_sample(0.002)
    
    # Final cleanup
    print("\nFinal cleanup...")
    merged = merged.voxel_down_sample(0.001)  # 1mm resolution
    merged = remove_statistical_outliers(merged, nb_neighbors=20, std_ratio=2.0)
    
    print(f"Final point count: {len(merged.points)}")
    
    # Save
    output_path = args.output
    print(f"\nSaving to {output_path}...")
    o3d.io.write_point_cloud(output_path, merged)
    print("Done!")
    
    # Visualize
    if not args.no_visualize:
        print("\nOpening viewer...")
        o3d.visualization.draw_geometries([merged])


def single_scan(args):
    """Perform a single high-quality scan."""
    
    print("=" * 50)
    print("SINGLE FRAME MODE")
    print("=" * 50)
    
    pipeline = rs.pipeline()
    config = rs.config()
    # Higher resolution for better quality
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
    
    profile = pipeline.start(config)
    
    # NOTE: On macOS, changing sensor settings causes segfault
    print("Using default camera settings (macOS compatibility)")
    
    depth_profile = profile.get_stream(rs.stream.depth)
    depth_intrinsics = depth_profile.as_video_stream_profile().get_intrinsics()
    
    # Filters
    spatial = rs.spatial_filter()
    temporal = rs.temporal_filter()
    align = rs.align(rs.stream.color)
    
    # Warm up + stabilize temporal filter
    print("Stabilizing (3 seconds)...")
    for _ in range(90):  # 3 seconds at 30fps
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        depth = aligned.get_depth_frame()
        if depth:
            depth = spatial.process(depth)
            depth = temporal.process(depth)
    
    # Capture
    print("Capturing...")
    frames = pipeline.wait_for_frames()
    aligned = align.process(frames)
    
    depth_frame = aligned.get_depth_frame()
    color_frame = aligned.get_color_frame()
    
    depth_frame = spatial.process(depth_frame)
    depth_frame = temporal.process(depth_frame)
    
    pipeline.stop()
    
    # Create point cloud
    pcd = create_point_cloud_from_frames(depth_frame, color_frame, depth_intrinsics)
    
    # Filter
    pcd = filter_by_depth(pcd, args.min_depth, args.max_depth)
    pcd = remove_statistical_outliers(pcd)
    
    print(f"Point count: {len(pcd.points)}")
    
    # Save
    o3d.io.write_point_cloud(args.output, pcd)
    print(f"Saved to {args.output}")
    
    if not args.no_visualize:
        o3d.visualization.draw_geometries([pcd])


def main():
    parser = argparse.ArgumentParser(
        description="Advanced 3D Scanning for Intel RealSense D455",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  Single scan:
    python3 advanced_scan.py -o scan.ply

  Turntable scan (30 seconds, 0.5-0.8m range):
    python3 advanced_scan.py --turntable -d 30 --min-depth 0.4 --max-depth 1.0 -o object.ply
        """
    )
    
    parser.add_argument("-o", "--output", default="scan.ply", help="Output PLY file")
    parser.add_argument("--turntable", action="store_true", help="Turntable mode: capture multiple frames and merge")
    parser.add_argument("-d", "--duration", type=float, default=30, help="Turntable mode: capture duration in seconds")
    parser.add_argument("--frame-interval", type=float, default=1.0, help="Seconds between frame captures")
    parser.add_argument("--min-depth", type=float, default=0.4, help="Minimum depth in meters")
    parser.add_argument("--max-depth", type=float, default=1.5, help="Maximum depth in meters")
    parser.add_argument("--bbox-size", type=float, default=None, help="Optional: bounding box size in meters (cube centered on object)")
    parser.add_argument("--min-height", type=float, default=None, help="Minimum height above turntable in meters (filters out the table)")
    parser.add_argument("--no-visualize", action="store_true", help="Skip visualization after capture")
    parser.add_argument("--save-frames", action="store_true", help="Save individual frames as frame_001.ply, frame_002.ply, etc.")
    parser.add_argument("--frames-dir", type=str, default="frames", help="Directory to save individual frames (default: frames/)")
    
    args = parser.parse_args()
    
    if args.turntable:
        turntable_scan(args)
    else:
        single_scan(args)


if __name__ == "__main__":
    main()

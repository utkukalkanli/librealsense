#!/usr/bin/env python3
"""
Point Cloud Filter Tool
Filter PLY files by color, position, or both.

Usage:
    python3 filter_pointcloud.py input.ply -o filtered.ply --color orange
    python3 filter_pointcloud.py input.ply -o filtered.ply --x-min -0.1 --x-max 0.1
"""

import argparse
import numpy as np
import open3d as o3d
import colorsys


def filter_by_color(pcd, color_name, tolerance=0.15):
    """Filter points by color name (e.g., 'orange', 'red', 'blue')."""
    
    # Define color ranges in HSV (Hue, Saturation, Value)
    color_ranges = {
        'orange': (0.05, 0.12),   # Hue range for orange
        'red': (0.0, 0.05),       # Red wraps around, also 0.95-1.0
        'yellow': (0.12, 0.20),
        'green': (0.25, 0.45),
        'blue': (0.55, 0.70),
        'purple': (0.75, 0.85),
        'white': None,  # Special handling
        'black': None,  # Special handling
    }
    
    if color_name not in color_ranges:
        print(f"Unknown color: {color_name}")
        print(f"Available colors: {list(color_ranges.keys())}")
        return pcd
    
    if not pcd.has_colors():
        print("Point cloud has no colors!")
        return pcd
    
    colors = np.asarray(pcd.colors)
    points = np.asarray(pcd.points)
    
    # Convert RGB to HSV
    hsv_colors = np.array([colorsys.rgb_to_hsv(r, g, b) for r, g, b in colors])
    
    if color_name == 'white':
        # High value, low saturation
        mask = (hsv_colors[:, 2] > 0.8) & (hsv_colors[:, 1] < 0.2)
    elif color_name == 'black':
        # Low value
        mask = hsv_colors[:, 2] < 0.2
    elif color_name == 'red':
        # Red wraps around hue=0
        hue = hsv_colors[:, 0]
        sat = hsv_colors[:, 1]
        mask = ((hue < 0.05) | (hue > 0.95)) & (sat > 0.3)
    else:
        hue_min, hue_max = color_ranges[color_name]
        hue = hsv_colors[:, 0]
        sat = hsv_colors[:, 1]
        val = hsv_colors[:, 2]
        # Looser thresholds to keep more data
        mask = (hue >= hue_min - tolerance) & (hue <= hue_max + tolerance) & (sat > 0.2) & (val > 0.2)
    
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(points[mask])
    filtered_pcd.colors = o3d.utility.Vector3dVector(colors[mask])
    
    return filtered_pcd


def filter_by_position(pcd, x_min=None, x_max=None, y_min=None, y_max=None, z_min=None, z_max=None):
    """Filter points by 3D position (bounding box)."""
    
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors) if pcd.has_colors() else None
    
    mask = np.ones(len(points), dtype=bool)
    
    if x_min is not None:
        mask &= points[:, 0] >= x_min
    if x_max is not None:
        mask &= points[:, 0] <= x_max
    if y_min is not None:
        mask &= points[:, 1] >= y_min
    if y_max is not None:
        mask &= points[:, 1] <= y_max
    if z_min is not None:
        mask &= points[:, 2] >= z_min
    if z_max is not None:
        mask &= points[:, 2] <= z_max
    
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(points[mask])
    if colors is not None:
        filtered_pcd.colors = o3d.utility.Vector3dVector(colors[mask])
    
    return filtered_pcd


def center_point_cloud(pcd):
    """Move point cloud centroid to origin (0, 0, 0)."""
    centroid = pcd.get_center()
    points = np.asarray(pcd.points) - centroid
    
    centered_pcd = o3d.geometry.PointCloud()
    centered_pcd.points = o3d.utility.Vector3dVector(points)
    if pcd.has_colors():
        centered_pcd.colors = pcd.colors
    
    print(f"Centered: moved by ({centroid[0]:.4f}, {centroid[1]:.4f}, {centroid[2]:.4f})")
    return centered_pcd


def main():
    parser = argparse.ArgumentParser(
        description="Filter point cloud by color or position",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  Filter orange objects:
    python3 filter_pointcloud.py scan.ply -o orange_only.ply --color orange

  Filter center region (narrow width):
    python3 filter_pointcloud.py scan.ply -o center.ply --x-min -0.1 --x-max 0.1

  Combine filters:
    python3 filter_pointcloud.py scan.ply -o result.ply --color orange --x-min -0.15 --x-max 0.15 --z-max 0.8
        """
    )
    
    parser.add_argument("input", help="Input PLY file")
    parser.add_argument("-o", "--output", required=True, help="Output PLY file")
    
    # Color filter
    parser.add_argument("--color", type=str, help="Filter by color (orange, red, yellow, green, blue, purple, white, black)")
    parser.add_argument("--color-tolerance", type=float, default=0.15, help="Color matching tolerance (0-0.5)")
    
    # Position filters (X = left/right, Y = up/down, Z = depth)
    parser.add_argument("--x-min", type=float, help="Minimum X (left cutoff in meters)")
    parser.add_argument("--x-max", type=float, help="Maximum X (right cutoff in meters)")
    parser.add_argument("--y-min", type=float, help="Minimum Y (bottom cutoff)")
    parser.add_argument("--y-max", type=float, help="Maximum Y (top cutoff)")
    parser.add_argument("--z-min", type=float, help="Minimum Z (near depth)")
    parser.add_argument("--z-max", type=float, help="Maximum Z (far depth)")
    
    # Centering
    parser.add_argument("--center", action="store_true", help="Move object centroid to origin (0,0,0)")
    
    # Visualization
    parser.add_argument("--no-visualize", action="store_true", help="Skip visualization")
    
    args = parser.parse_args()
    
    # Load
    print(f"Loading {args.input}...")
    pcd = o3d.io.read_point_cloud(args.input)
    print(f"Original points: {len(pcd.points)}")
    
    # Apply color filter
    if args.color:
        print(f"Filtering by color: {args.color}")
        pcd = filter_by_color(pcd, args.color, args.color_tolerance)
        print(f"After color filter: {len(pcd.points)} points")
    
    # Apply position filter
    if any([args.x_min, args.x_max, args.y_min, args.y_max, args.z_min, args.z_max]):
        print("Applying position filter...")
        pcd = filter_by_position(
            pcd,
            x_min=args.x_min, x_max=args.x_max,
            y_min=args.y_min, y_max=args.y_max,
            z_min=args.z_min, z_max=args.z_max
        )
        print(f"After position filter: {len(pcd.points)} points")
    
    # Center the point cloud
    if args.center:
        pcd = center_point_cloud(pcd)
    
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

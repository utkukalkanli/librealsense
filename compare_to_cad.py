#!/usr/bin/env python3
"""
CAD Comparison Tool for Anomaly Detection
Compares a scanned point cloud against a reference CAD model (STL/OBJ/PLY).
Highlights deviations and generates a deviation report.

Usage:
    python3 compare_to_cad.py reference.stl scan.ply -o deviation_report.ply
"""

import argparse
import numpy as np
import open3d as o3d
import json
from datetime import datetime


def load_reference_model(filepath):
    """Load reference model (STL, OBJ, or PLY) and sample points from it."""
    
    print(f"Loading reference model: {filepath}")
    
    # Check file extension
    ext = filepath.lower().split('.')[-1]
    
    if ext in ['stl', 'obj', 'ply', 'off']:
        # Try loading as mesh first
        mesh = o3d.io.read_triangle_mesh(filepath)
        
        if mesh.is_empty():
            # Maybe it's a point cloud PLY
            pcd = o3d.io.read_point_cloud(filepath)
            if not pcd.is_empty():
                print(f"  Loaded as point cloud: {len(pcd.points)} points")
                return pcd, None
            else:
                raise ValueError(f"Could not load {filepath} as mesh or point cloud")
        
        # Sample points from mesh surface
        print(f"  Mesh loaded: {len(mesh.triangles)} triangles")
        
        # Compute normals if missing
        if not mesh.has_vertex_normals():
            mesh.compute_vertex_normals()
        
        # Sample points uniformly from mesh surface
        # More triangles = more sample points for accurate comparison
        num_points = max(100000, len(mesh.triangles) * 10)
        pcd = mesh.sample_points_uniformly(number_of_points=num_points)
        print(f"  Sampled {len(pcd.points)} points from mesh surface")
        
        return pcd, mesh
    else:
        raise ValueError(f"Unsupported file format: {ext}")


def load_scan(filepath):
    """Load scanned point cloud (PLY)."""
    
    print(f"Loading scan: {filepath}")
    pcd = o3d.io.read_point_cloud(filepath)
    
    if pcd.is_empty():
        raise ValueError(f"Could not load scan: {filepath}")
    
    print(f"  Loaded: {len(pcd.points)} points")
    return pcd


def align_point_clouds(source, target, voxel_size=0.005):
    """Align scan to reference using ICP registration."""
    
    print("Aligning scan to reference model...")
    
    # Downsample for faster registration
    source_down = source.voxel_down_sample(voxel_size)
    target_down = target.voxel_down_sample(voxel_size)
    
    # Estimate normals
    source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    target_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    
    # Initial guess (identity)
    trans_init = np.eye(4)
    
    # Coarse registration with point-to-point ICP
    threshold_coarse = voxel_size * 15
    reg_coarse = o3d.pipelines.registration.registration_icp(
        source_down, target_down, threshold_coarse, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    
    # Fine registration with point-to-plane ICP
    threshold_fine = voxel_size * 1.5
    reg_fine = o3d.pipelines.registration.registration_icp(
        source_down, target_down, threshold_fine, reg_coarse.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane()
    )
    
    print(f"  Alignment fitness: {reg_fine.fitness:.3f}")
    print(f"  Alignment RMSE: {reg_fine.inlier_rmse:.4f}m")
    
    # Apply transformation to the original (full resolution) source
    source.transform(reg_fine.transformation)
    
    return source, reg_fine


def compute_deviations(scan_pcd, reference_pcd):
    """Compute point-to-point distances from scan to reference."""
    
    print("Computing deviations...")
    
    # Build KD-tree for reference
    reference_tree = o3d.geometry.KDTreeFlann(reference_pcd)
    
    scan_points = np.asarray(scan_pcd.points)
    distances = np.zeros(len(scan_points))
    
    # Find nearest neighbor for each scan point
    for i, point in enumerate(scan_points):
        [_, idx, dist] = reference_tree.search_knn_vector_3d(point, 1)
        distances[i] = np.sqrt(dist[0])  # Convert squared distance to distance
    
    return distances


def colorize_by_deviation(pcd, distances, threshold_mm=2.0, max_deviation_mm=10.0):
    """Color point cloud by deviation: green=good, yellow=warning, red=bad."""
    
    threshold = threshold_mm / 1000.0  # Convert to meters
    max_dev = max_deviation_mm / 1000.0
    
    colors = np.zeros((len(distances), 3))
    
    for i, dist in enumerate(distances):
        if dist <= threshold:
            # Good: Green
            colors[i] = [0.0, 0.8, 0.0]
        elif dist <= threshold * 2:
            # Warning: Yellow to Orange gradient
            t = (dist - threshold) / threshold
            colors[i] = [1.0, 1.0 - t * 0.5, 0.0]
        else:
            # Bad: Orange to Red gradient
            t = min(1.0, (dist - threshold * 2) / max_dev)
            colors[i] = [1.0, 0.5 * (1 - t), 0.0]
    
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd


def generate_report(distances, threshold_mm=2.0):
    """Generate statistical report of deviations."""
    
    distances_mm = distances * 1000  # Convert to mm
    
    report = {
        "timestamp": datetime.now().isoformat(),
        "total_points": len(distances),
        "statistics": {
            "min_deviation_mm": float(np.min(distances_mm)),
            "max_deviation_mm": float(np.max(distances_mm)),
            "mean_deviation_mm": float(np.mean(distances_mm)),
            "median_deviation_mm": float(np.median(distances_mm)),
            "std_deviation_mm": float(np.std(distances_mm)),
        },
        "quality_assessment": {
            "within_threshold": int(np.sum(distances_mm <= threshold_mm)),
            "warning_zone": int(np.sum((distances_mm > threshold_mm) & (distances_mm <= threshold_mm * 2))),
            "out_of_spec": int(np.sum(distances_mm > threshold_mm * 2)),
        }
    }
    
    total = report["total_points"]
    report["quality_assessment"]["within_threshold_pct"] = round(100 * report["quality_assessment"]["within_threshold"] / total, 1)
    report["quality_assessment"]["warning_zone_pct"] = round(100 * report["quality_assessment"]["warning_zone"] / total, 1)
    report["quality_assessment"]["out_of_spec_pct"] = round(100 * report["quality_assessment"]["out_of_spec"] / total, 1)
    
    return report


def print_report(report, threshold_mm):
    """Print human-readable report."""
    
    print()
    print("=" * 50)
    print("DEVIATION ANALYSIS REPORT")
    print("=" * 50)
    print()
    
    stats = report["statistics"]
    print("Statistics:")
    print(f"  Min deviation:    {stats['min_deviation_mm']:.2f} mm")
    print(f"  Max deviation:    {stats['max_deviation_mm']:.2f} mm")
    print(f"  Mean deviation:   {stats['mean_deviation_mm']:.2f} mm")
    print(f"  Median deviation: {stats['median_deviation_mm']:.2f} mm")
    print(f"  Std deviation:    {stats['std_deviation_mm']:.2f} mm")
    print()
    
    qa = report["quality_assessment"]
    print(f"Quality Assessment (threshold: {threshold_mm} mm):")
    print(f"  ✅ Within spec:   {qa['within_threshold']:6d} points ({qa['within_threshold_pct']:5.1f}%)")
    print(f"  ⚠️  Warning zone:  {qa['warning_zone']:6d} points ({qa['warning_zone_pct']:5.1f}%)")
    print(f"  ❌ Out of spec:   {qa['out_of_spec']:6d} points ({qa['out_of_spec_pct']:5.1f}%)")
    print()
    
    # Overall verdict
    if qa['out_of_spec_pct'] < 1:
        print("RESULT: ✅ PASS - Object matches reference within tolerance")
    elif qa['out_of_spec_pct'] < 5:
        print("RESULT: ⚠️ WARNING - Minor deviations detected")
    else:
        print("RESULT: ❌ FAIL - Significant deviations from reference")


def main():
    parser = argparse.ArgumentParser(
        description="Compare scanned point cloud to CAD reference model",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  Basic comparison:
    python3 compare_to_cad.py reference.stl scan.ply

  With custom threshold:
    python3 compare_to_cad.py reference.stl scan.ply --threshold 1.5

  Save colorized result:
    python3 compare_to_cad.py reference.stl scan.ply -o deviation.ply
        """
    )
    
    parser.add_argument("reference", help="Reference model (STL, OBJ, or PLY)")
    parser.add_argument("scan", help="Scanned point cloud (PLY)")
    parser.add_argument("-o", "--output", help="Output colorized point cloud (PLY)")
    parser.add_argument("--threshold", type=float, default=2.0, help="Acceptable deviation threshold in mm (default: 2.0)")
    parser.add_argument("--no-align", action="store_true", help="Skip automatic alignment (if already aligned)")
    parser.add_argument("--no-visualize", action="store_true", help="Skip visualization")
    parser.add_argument("--save-report", help="Save JSON report to file")
    
    args = parser.parse_args()
    
    # Load models
    reference_pcd, reference_mesh = load_reference_model(args.reference)
    scan_pcd = load_scan(args.scan)
    
    # Align if needed
    if not args.no_align:
        scan_pcd, reg_result = align_point_clouds(scan_pcd, reference_pcd)
    
    # Compute deviations
    distances = compute_deviations(scan_pcd, reference_pcd)
    
    # Generate report
    report = generate_report(distances, args.threshold)
    print_report(report, args.threshold)
    
    # Save JSON report
    if args.save_report:
        with open(args.save_report, 'w') as f:
            json.dump(report, f, indent=2)
        print(f"Report saved to: {args.save_report}")
    
    # Colorize point cloud
    colorized = colorize_by_deviation(scan_pcd, distances, args.threshold)
    
    # Save colorized output
    if args.output:
        o3d.io.write_point_cloud(args.output, colorized)
        print(f"Colorized result saved to: {args.output}")
    
    # Visualize
    if not args.no_visualize:
        print()
        print("Opening viewer...")
        print("  Green = within tolerance")
        print("  Yellow/Orange = warning zone")
        print("  Red = out of spec")
        
        # Show both reference (blue) and colorized scan
        reference_pcd.paint_uniform_color([0.5, 0.5, 0.8])  # Light blue for reference
        o3d.visualization.draw_geometries(
            [colorized, reference_pcd],
            window_name="CAD Comparison - Deviation Analysis"
        )


if __name__ == "__main__":
    main()

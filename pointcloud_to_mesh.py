#!/usr/bin/env python3
"""
Point Cloud to Mesh Converter
Converts PLY point clouds to mesh (STL/OBJ/PLY).

Usage:
    python3 pointcloud_to_mesh.py input.ply -o output.stl
"""

import argparse
import numpy as np
import open3d as o3d


def estimate_normals(pcd, radius=0.01):
    """Estimate normals for point cloud if not present."""
    if not pcd.has_normals():
        print("Estimating normals...")
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30)
        )
        # Orient normals consistently
        pcd.orient_normals_consistent_tangent_plane(k=15)
    return pcd


def poisson_reconstruction(pcd, depth=9):
    """Create mesh using Poisson surface reconstruction."""
    print(f"Running Poisson reconstruction (depth={depth})...")
    
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=depth, linear_fit=True
    )
    
    # Remove low-density vertices (cleanup)
    densities = np.asarray(densities)
    density_threshold = np.quantile(densities, 0.05)  # Remove bottom 5%
    vertices_to_remove = densities < density_threshold
    mesh.remove_vertices_by_mask(vertices_to_remove)
    
    print(f"  Vertices: {len(mesh.vertices)}")
    print(f"  Triangles: {len(mesh.triangles)}")
    
    return mesh


def ball_pivoting_reconstruction(pcd, radii=None):
    """Create mesh using Ball Pivoting Algorithm."""
    print("Running Ball Pivoting reconstruction...")
    
    if radii is None:
        # Auto-calculate radii based on point cloud density
        distances = pcd.compute_nearest_neighbor_distance()
        avg_dist = np.mean(distances)
        radii = [avg_dist * 1.5, avg_dist * 3, avg_dist * 6]
    
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd, o3d.utility.DoubleVector(radii)
    )
    
    print(f"  Vertices: {len(mesh.vertices)}")
    print(f"  Triangles: {len(mesh.triangles)}")
    
    return mesh


def alpha_shape_reconstruction(pcd, alpha=0.03):
    """Create mesh using Alpha Shapes."""
    print(f"Running Alpha Shape reconstruction (alpha={alpha})...")
    
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    
    print(f"  Vertices: {len(mesh.vertices)}")
    print(f"  Triangles: {len(mesh.triangles)}")
    
    return mesh


def main():
    parser = argparse.ArgumentParser(
        description="Convert point cloud to mesh",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  Basic conversion (Poisson):
    python3 pointcloud_to_mesh.py scan.ply -o model.stl

  Ball Pivoting method:
    python3 pointcloud_to_mesh.py scan.ply -o model.obj --method bpa

  Alpha Shapes method:
    python3 pointcloud_to_mesh.py scan.ply -o model.ply --method alpha --alpha 0.02
        """
    )
    
    parser.add_argument("input", help="Input point cloud (PLY)")
    parser.add_argument("-o", "--output", required=True, help="Output mesh (STL, OBJ, or PLY)")
    parser.add_argument("--method", choices=["poisson", "bpa", "alpha"], default="poisson",
                        help="Reconstruction method (default: poisson)")
    parser.add_argument("--depth", type=int, default=9, help="Poisson depth (default: 9)")
    parser.add_argument("--alpha", type=float, default=0.03, help="Alpha shape value (default: 0.03)")
    parser.add_argument("--smooth", action="store_true", help="Apply smoothing filter")
    parser.add_argument("--simplify", type=int, help="Target number of triangles for simplification")
    parser.add_argument("--no-visualize", action="store_true", help="Skip visualization")
    
    args = parser.parse_args()
    
    # Load point cloud
    print(f"Loading {args.input}...")
    pcd = o3d.io.read_point_cloud(args.input)
    print(f"Points: {len(pcd.points)}")
    
    if len(pcd.points) == 0:
        print("Error: Point cloud is empty!")
        return
    
    # Estimate normals (required for most methods)
    pcd = estimate_normals(pcd)
    
    # Reconstruct mesh
    if args.method == "poisson":
        mesh = poisson_reconstruction(pcd, depth=args.depth)
    elif args.method == "bpa":
        mesh = ball_pivoting_reconstruction(pcd)
    elif args.method == "alpha":
        mesh = alpha_shape_reconstruction(pcd, alpha=args.alpha)
    
    # Post-processing
    if args.smooth:
        print("Smoothing mesh...")
        mesh = mesh.filter_smooth_simple(number_of_iterations=5)
        mesh.compute_vertex_normals()
    
    if args.simplify:
        print(f"Simplifying to {args.simplify} triangles...")
        mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=args.simplify)
        mesh.compute_vertex_normals()
    
    # Transfer colors from point cloud if available
    if pcd.has_colors() and len(mesh.vertices) > 0:
        print("Transferring vertex colors...")
        # Build KD-tree for original points
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)
        vertices = np.asarray(mesh.vertices)
        colors = np.zeros((len(vertices), 3))
        pcd_colors = np.asarray(pcd.colors)
        
        for i, vertex in enumerate(vertices):
            [_, idx, _] = pcd_tree.search_knn_vector_3d(vertex, 1)
            colors[i] = pcd_colors[idx[0]]
        
        mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
    
    # Compute normals for nice visualization
    mesh.compute_vertex_normals()
    
    # Save
    print(f"Saving to {args.output}...")
    o3d.io.write_triangle_mesh(args.output, mesh)
    print("Done!")
    
    # Visualize
    if not args.no_visualize:
        print("Opening viewer...")
        o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)


if __name__ == "__main__":
    main()

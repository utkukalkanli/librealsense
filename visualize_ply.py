
import open3d as o3d
import sys
import os
import glob
import argparse
import numpy as np

def visualize_ply(file_path, threshold=None):
    print(f"Loading {file_path}...")
    pcd = o3d.io.read_point_cloud(file_path)
    
    if pcd.is_empty():
        print("Point cloud is empty!")
        return

    if threshold is not None:
        print(f"Applying threshold filter: {threshold} meters")
        points = np.asarray(pcd.points)
        
        # Filter points based on depth (Z axis)
        # We check abs(Z) to cover different coordinate systems
        mask = (np.abs(points[:, 2]) > 0) & (np.abs(points[:, 2]) < threshold)
        
        if np.sum(mask) == 0:
            print("Warning: Threshold removed all points!")
            return
            
        cleaned_points = points[mask]
        print(f"Points remaining: {len(cleaned_points)} / {len(points)}")
        
        # Create a new point cloud with filtered points
        new_pcd = o3d.geometry.PointCloud()
        new_pcd.points = o3d.utility.Vector3dVector(cleaned_points)
        
        if pcd.has_colors():
            colors = np.asarray(pcd.colors)
            new_pcd.colors = o3d.utility.Vector3dVector(colors[mask])
            
        pcd = new_pcd

    print("Visualizing... (Close the window to exit)")
    # Visualize with a coordinate frame for reference
    o3d.visualization.draw_geometries([pcd, o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)])

def main():
    parser = argparse.ArgumentParser(description="Visualize PLY file with optional depth thresholding.")
    parser.add_argument("file", nargs="?", help="Input PLY file")
    parser.add_argument("-t", "--threshold", type=float, help="Max depth distance in meters")
    
    args = parser.parse_args()
    
    file_path = args.file
    
    if not file_path:
        # Default to the most recent ply file in test_output if available
        ply_files = glob.glob("test_output/frame_*.ply")
        if not ply_files:
            print("Usage: python3 visualize_ply.py <path_to_ply_file> [-t threshold]")
            print("No PLY files found in 'test_output/' default path.")
            return
        
        # Sort by modification time to get the last one
        file_path = max(ply_files, key=os.path.getmtime)
        print(f"No file specified. Using most recent: {file_path}")

    visualize_ply(file_path, args.threshold)

if __name__ == "__main__":
    main()

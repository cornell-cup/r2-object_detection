import open3d as o3d

def main():
    cloud = o3d.io.read_point_cloud("1.ply") # Read the point cloud
    o3d.visualization.draw_geometries([cloud]) # Visualize the point cloud

if __name__ == "__main__":
    main()

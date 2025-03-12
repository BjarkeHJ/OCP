import os
import numpy as np
import open3d as o3d

cwd = os.getcwd()
folder = "vis_dev_tools"
file_name = "output.pcd"
pcd_path = os.path.join(cwd, folder, file_name)
if not os.path.exists(pcd_path):
    raise FileNotFoundError(f"File '{pcd_path}' not found!")

class Visualizer:
    def __init__(self):
        self.point_cloud = None
        self.size = 0

    def set_pcd(self, pcd_path):
        pcd = o3d.io.read_point_cloud(pcd_path)
        if pcd.is_empty():
            raise ValueError("Loaded point cloud is empty!")
        self.point_cloud = pcd
        self.get_size()

    def get_size(self):
        if self.point_cloud is None:
            raise ValueError("Point cloud is not set!")
        self.size = len(self.point_cloud.points)  # Get actual point count
        return self.size

    def visualize(self):
        if self.point_cloud is None:
            raise ValueError("Point cloud is not set!")
        o3d.visualization.draw_geometries([self.point_cloud], point_show_normal=True)


pcd_vis = Visualizer()
pcd_vis.set_pcd(pcd_path)
print("Cloud Size: ", pcd_vis.size)
pcd_vis.visualize()
import open3d as o3d
import numpy as np


def get_uniformed_pcd(filename):
    base_pcd = o3d.io.read_point_cloud(filename)
    points = np.asarray(base_pcd.points)

    for i in range(3):
        points[:, i] -= np.min(points[:, i])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = base_pcd.colors

    return pcd

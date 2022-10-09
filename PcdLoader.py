import open3d as o3d
import numpy as np

# 旧式
def get_uniformed_pcd(filename):
    base_pcd = o3d.io.read_point_cloud(filename)
    points = np.asarray(base_pcd.points)

    for i in range(3):
        points[:, i] -= np.min(points[:, i])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = base_pcd.colors

    return pcd


def load_pcd(filename):
    pcd = o3d.io.read_point_cloud(filename)
    points = np.asarray(pcd.points)
    min_vals = np.array([100.0, 100.0, 100.0])
    for p in points:
        for (i, v) in enumerate(p):
            if min_vals[i] > v:
                min_vals[i] = v
    points -= min_vals

    pcd.points = o3d.utility.Vector3dVector(points)

    return pcd, -min_vals


def load_pcd_with_mesh(filename, mesh_filename):
    pcd, trans = load_pcd(filename)

    mesh = o3d.io.read_triangle_mesh(mesh_filename)
    mesh.translate(trans)
    mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)

    scene = o3d.t.geometry.RaycastingScene()
    mesh_id = scene.add_triangles(mesh)

    return pcd, scene, trans, mesh_id

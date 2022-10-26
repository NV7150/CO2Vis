import open3d as o3d
import numpy as np
import json


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


def pcd2json(pcd, s_pos):
    points_list = np.array(pcd.points).tolist()
    colors_list = np.array(pcd.colors).tolist()

    d = {
        "points": [
            {"x": p[0], "y": p[1], "z": p[2]} for p in points_list
        ],
        "colors": [
            {"r": c[0], "g": c[1], "b": c[2]} for c in colors_list
        ],
        "sensors": [
            {"id": s.id, "pos": {"x": s.pos[0], "y": s.pos[1], "z": s.pos[2]}} for s in s_pos
        ]
    }

    return json.dumps(d)

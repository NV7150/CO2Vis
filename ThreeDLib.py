from FrameLoader import Frame
import numpy as np
import open3d as o3d


# 画像のcenterのポジションからz行ったところの三次元座標を返す
def get_3dpos_frame(center, frame: Frame, z=1, trans=None):
    if trans is None:
        trans = np.zeros(shape=3)

    inv_int = np.linalg.inv(frame.intrinsics)
    image_pos = np.array([[center[0]], [center[1]], [1]]) * z

    position_camera_pos = np.dot(inv_int, image_pos)
    position_camera_pos = np.concatenate([position_camera_pos, np.array([[1]])])

    position_global_pos = np.dot(frame.pose, position_camera_pos)
    position_global_pos = position_global_pos.flatten()[:3]

    return -(position_global_pos - frame.pos) + frame.pos + trans


def raycast2dest_mesh(frame: Frame, destination, mesh_scene, trans=None):
    if trans is None:
        trans = np.arange([0, 0, 0])

    origin_pos = frame.pos + trans

    vec_to_dest = (origin_pos - destination)
    vec_l = np.linalg.norm(vec_to_dest)
    direction = vec_to_dest / vec_l

    ray_vec = np.concatenate([origin_pos, -direction])
    ray = o3d.core.Tensor([ray_vec], dtype=o3d.core.Dtype.Float32)
    ans = mesh_scene.cast_rays(ray)

    hit_dist = ans['t_hit'].numpy()[0]
    hit_pos = -direction * hit_dist + origin_pos

    return hit_pos



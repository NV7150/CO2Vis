import datetime

import open3d as o3d
import open3d.visualization.gui as gui
import numpy as np
import random
from scipy.spatial import Voronoi, ConvexHull
import matplotlib.pyplot as plt
import math
import time
import threading
import glob

from VisLibrary import add_point
from ColorMapper import *
from PcdLoader import get_uniformed_pcd, load_pcd_with_mesh
from GuiAdmin import *
from QrCodeDefiner import define_qr_pos
from RequestHandler import get_current_data, convert_jsons


def parse_dict_to_points(dic):
    points = []
    for key, val in dic.items():
        points.append(Point(np.array(val), str(key).strip()))
    return points


def visualize(
        pcd_path,
        mesh_path,
        folder_path,
        time_th=-1,
        reflesh_rate=10,
        random_data=False,
        voxel=-1
):
    pcd, scene, trans, mesh_id = load_pcd_with_mesh(pcd_path, mesh_path)

    if voxel != -1:
        pcd = pcd.voxel_down_sample(voxel_size=voxel)

    pos_dict = define_qr_pos(
        glob.glob(f"{folder_path}/frame*.json"),
        pcd,
        mesh_scene=scene,
        trans=trans
    )

    print("define qr pos completed")

    points = parse_dict_to_points(pos_dict)

    # 下４桁固定
    points = [Point(p.pos, p.id[-4:]) for p in points]
    print(points)

    color = ColorMapper(500, 700, 1000)
    mapper = SoftmaxMapper(pcd, points, color, cut_th=0)

    def update_tick(mapper_ins, gui_ins):
        while True:
            values = -1
            if not random_data:
                values = get_current_data()
            else:
                values = {"data": [
                    {
                        "sensorid": p.id,
                        "timeline": datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                        "senco2": int(random.uniform(500, 1000))
                    }
                    for p in points
                ]}

            if values == -1:
                print("connection timed out")
                continue

            datas = convert_jsons(values, time_th)

            mapper_ins.update_values(datas)

            n_pcd = mapper_ins.export_pcd()

            for p in points:
                add_point(n_pcd, p.pos, color=[1, 0, 1])

            labels = []

            for p in mapper.sensor_points:
                label = LabelData()
                label.pos = p.pos
                label.label = f"{math.floor(mapper.values[p.id])}"
                labels.append(label)

            gui_ins.update_geometry(n_pcd)
            gui_ins.update_labels(labels)

            time.sleep(reflesh_rate)

    gui_wrapper = GuiWrapper()

    th = threading.Thread(target=update_tick, args=[mapper, gui_wrapper])

    th.start()

    flag = True

    while flag:
        flag = gui_wrapper.show_in_tick()

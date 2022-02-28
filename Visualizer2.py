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


def visualize(pcd_path, mesh_path, folder_path, time_th=-1, reflesh_rate=10):
    pcd, scene, trans, mesh_id = load_pcd_with_mesh(pcd_path, mesh_path)

    pos_dict = define_qr_pos(
        glob.glob(f"{folder_path}/frame*.json"),
        pcd,
        mesh_scene=scene,
        trans=trans
    )

    points = parse_dict_to_points(pos_dict)

    color = ColorMapper(500, 700, 1000)
    mapper = SoftmaxMapper(pcd, points, color, cut_th=0)

    def update_tick(mapper_ins, gui_ins):
        while True:
            values = get_current_data()

            if values == -1:
                print("connection timed out")
                continue

            print(values.keys())
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
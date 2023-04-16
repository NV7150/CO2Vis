import datetime
import json

# import keyboard
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
import websockets
import asyncio

from VisLibrary import add_point
from ColorMapper import *
from PcdLoader import get_uniformed_pcd, load_pcd_with_mesh, pcd2json
from GuiAdmin import *
from QrCodeDefiner import define_qr_pos
from RequestHandler import get_current_data, convert_jsons, delete_mutiple
from Visualizer2 import parse_dict_to_points, save_points, load_points
from sampling import farthest_point_sample_np


async def up_server(
        pcd_path,
        mesh_path,
        folder_path,
        time_th=-1,
        reflesh_rate=10,
        random_data=False,
        voxel=-1,
        save_file="posSave.json",
        new_api=False,
        search_bus="SKK",
        ws_host='localhost',
        ws_port=8765,
        sample=1024,
        resource_con=None,
        virtual_server=None
):
    print("started")
    pcd, scene, trans, mesh_id = load_pcd_with_mesh(pcd_path, mesh_path)
    print("pcd loaded")

    if voxel != -1:
        pcd = pcd.voxel_down_sample(voxel_size=voxel)

    if sample > 0:
        s_points = np.array(pcd.points)
        s_colors = np.array(pcd.colors)
        sample_idx = farthest_point_sample_np(s_points, sample)
        s_points = s_points[sample_idx]
        s_colors = s_colors[sample_idx]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(s_points)
        pcd.colors = o3d.utility.Vector3dVector(s_colors)

    if len(glob.glob(save_file)) <= 0:
        pos_dict = define_qr_pos(
            glob.glob(f"{folder_path}/frame*.json"),
            pcd,
            mesh_scene=scene,
            trans=trans
        )

        print("define qr pos completed")

        points = parse_dict_to_points(pos_dict)

        points = [Point(p.pos, p.id) for p in points]
        print(points)
        save_points(points, save_file)
    else:
        print("file ", save_file, " found")
        points = load_points(save_file)

    color = ColorMapper(500, 1000, 4000)
    mapper = SoftmaxMapper(pcd, points, color, cut_th=0, blend_rate=0.3)

    async def update_tick(ws):
        while True:
            values = -1
            if not (virtual_server is None):
                values = virtual_server.next_data()

            else:
                if not random_data:
                    if resource_con is None:
                        values = get_current_data(new_api=new_api, search=search_bus)
                    else:
                        values = resource_con.read()
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
                time.sleep(reflesh_rate)
                continue

            datas = delete_mutiple(convert_jsons(values, time_th, new_api=new_api))
            print([d.__str__() for d in datas])

            mapper.update_values(datas)

            n_pcd = mapper.export_pcd()

            for p in points:
                add_point(n_pcd, p.pos, color=[1, 0, 1])

            labels = []

            for p in mapper.sensor_points:
                if p.id not in mapper.values.keys():
                    continue
                label = LabelData()
                label.pos = p.pos
                label.label = f"{math.floor(mapper.values[p.id])}"
                labels.append(label)
            content = pcd2json(n_pcd, points)
            await ws.send(content)
            await asyncio.sleep(reflesh_rate)

            # if keyboard.is_pressed('escape'):
            #     break
    print('starting server')
    # await update_tick(None)
    async with websockets.serve(update_tick, ws_host, ws_port):
        await asyncio.Future()

import datetime

import numpy as np
from Visualizer2 import visualize
from VirtualCo2Server import VirtualServer
from RequestHandler import SensorData, convert_csv_data


def kernel(img):
    # img_colors = np.array(img)
    # avr_binaried = [np.mean(pix) for pix in img_colors]
    # black = np.min(avr_binaried)
    # white = np.max(avr_binaried)
    #
    # img_colors -= black
    # img_colors = img_colors *

    return img


if __name__ == "__main__":
    # s_datas = []
    # with open("resources/20220706.csv") as f:
    #     head = True
    #     for row in f:
    #         if head:
    #             head = False
    #             continue
    #         s_datas.append(convert_csv_data(row))
    #
    # virtual_server = VirtualServer(datetime.datetime(2022, 7, 6, 12, 0, 0), s_datas, timescale=18)
    # visualize(
    #     "sampleData/car.ply",
    #     "sampleData/car.obj",
    #     "sampleData/car",
    #     voxel=0.1,
    #     reflesh_rate=0.1,
    #     save_file="sampleData/ex-0706.json",
    #     virtual_server=virtual_server
    # )
    visualize(
        "sampleData/bus.ply",
        "sampleData/bus2.obj",
        "sampleData/bus2",
        # voxel=0.1,
        # reflesh_rate=0.1,
        save_file="sampleData/bus2-pos.json",
        random_data=True
        # virtual_server=virtual_server
    )

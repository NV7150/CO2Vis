import random

import numpy as np
import open3d as o3d

from ColorMapper import HeatMapper, ColorMapper, Point
from VisLibrary import add_point
from PcdLoader import get_uniformed_pcd


def visualize_test():
    pcd = get_uniformed_pcd("sampleData/sampleScan.ply")
    pcd = pcd.voxel_down_sample(voxel_size=0.1)

    points = [
        Point(
            np.array([random.uniform(-0.5, 3.5), random.uniform(0.5, 3.5), random.uniform(0.5, 10.0)]),
            str(i)
        )
        for i in range(30)
    ]

    color = ColorMapper(500, 700, 1000)

    mapper = HeatMapper(pcd, points, color)

    values = [
        (str(i), random.uniform(700, 1000)) for i in range(30)
    ]

    for i, v in values:
        mapper.calculate(i, v)

    # mapper.recal_all()

    n_pcd = mapper.export_pcd()
    lines = mapper.export_lines()

    for p in points:
        add_point(n_pcd, p.pos, color=[1, 0, 1])

    o3d.visualization.draw_geometries([n_pcd, lines])


visualize_test()

import math

import numpy as np
import open3d as o3d
from scipy.spatial import Delaunay


class ColorMapper:
    def __init__(self, safe, warn, danger, safe_color=None, warn_color=None, danger_color=None):
        if safe_color is None:
            safe_color = np.array([0, 1, 0], dtype=float)

        if warn_color is None:
            warn_color = np.array([1, 1, 0], dtype=float)

        if danger_color is None:
            danger_color = np.array([1, 0, 0], dtype=float)

        if not (safe <= warn <= danger):
            raise Exception("invalid range")

        self.safe = safe
        self.warn = warn
        self.danger = danger

        self.safe_color = safe_color
        self.warn_color = warn_color
        self.danger_color = danger_color

    def __call__(self, v):
        if v <= self.safe:
            return np.array(self.safe_color)
        elif self.safe <= v <= self.warn:
            d = v - self.safe
            prop = d / float(self.warn - self.safe)
            return self.safe_color * (1 - prop) + self.warn_color * prop
        elif self.warn <= v <= self.danger:
            d = v - self.warn
            prop = d / float(self.danger - self.warn)
            return self.warn_color * (1 - prop) + self.danger_color * prop

        return np.array(self.danger_color)


class Point:
    pos: np.ndarray
    id: str

    def __init__(self, pos, id):
        self.pos = pos
        self.id = id


class Tetrahedron:
    def __init__(self, points: list[Point]):
        self.points = points

    def get_scale(self):
        o = self.points[0].pos
        oa = self.points[1].pos - o
        ob = self.points[2].pos - o
        oc = self.points[3].pos - o

        return abs(np.dot(np.cross(oa, ob), oc)) / 6

    def is_in(self, point):
        tris = [
            Tetrahedron(points=[
                Point(point, -1),
                self.points[i],
                self.points[i + 1 if i + 1 < 4 else 0],
                self.points[i + 2 if i + 2 < 4 else i - 2]
            ])
            for i in range(4)
        ]

        scale_sum = 0
        for tri in tris:
            scale_sum += tri.get_scale()

        return abs(scale_sum - self.get_scale()) <= 1e-4


class HeatMapper:
    def __init__(self, pcd, sensor_points: list[Point], color_mapper: ColorMapper):

        self.pcd2tri = {}
        self.sensor_id2pcdi = {}

        p_arr = np.array([p.pos for p in sensor_points])
        self.sensor_poses = p_arr

        d = Delaunay(p_arr)

        tri_points = d.simplices

        self.lines = []

        for p in tri_points:
            for i, p1 in enumerate(p):
                for j in range(len(p) - (i + 1)):
                    ind = j + i + 1
                    if ind >= len(p):
                        break
                    p2 = p[ind]
                    self.lines.append([p1, p2])

        tris = [
            Tetrahedron([sensor_points[p_i] for p_i in p]) for p in tri_points
        ]

        pcd_points = np.asarray(pcd.points)

        for i, p in enumerate(pcd_points):
            for tri in tris:
                if tri.is_in(p):
                    self.pcd2tri.setdefault(i, tri)

                    for t_p in tri.points:
                        if t_p.id not in self.sensor_id2pcdi.keys():
                            self.sensor_id2pcdi.setdefault(t_p.id, [])

                        self.sensor_id2pcdi[t_p.id].append(i)
                    break

        for s_p in sensor_points:
            if s_p.id not in self.sensor_id2pcdi.keys():
                self.sensor_id2pcdi.setdefault(s_p.id, [])

        self.current_colors = [color_mapper(0) for i in range(len(pcd_points))]
        self.c = color_mapper
        self.points = pcd_points

        self.values = {}
        for p in sensor_points:
            self.values.setdefault(p.id, 0)

    def cal_point(self, i):
        if i not in self.pcd2tri.keys():
            return self.c(0)

        tri = self.pcd2tri[i]
        pos = self.points[i]

        distances = []

        for p in tri.points:
            distances.append(np.linalg.norm(p.pos - pos))

        def softmax(x):
            y = np.exp(x)
            f_x = y / np.sum(np.exp(x))
            return f_x

        distances = softmax(np.array(distances))
        s = 0
        for i, d in enumerate(distances):
            s_id = tri.points[i].id
            s += self.values[s_id] * d

        print(self.c(s), " ", s, " ", distances)
        return self.c(s)

    def calculate(self, sensor_id, value):
        self.values[sensor_id] = value

        points = self.sensor_id2pcdi[sensor_id]
        for p in points:
            self.current_colors[p] = self.cal_point(p)

    def recal_all(self):
        for p in range(len(self.points)):
            self.current_colors[p] = self.cal_point(p)

    def export_pcd(self):
        pcd = o3d.geometry.PointCloud()

        pcd.points = o3d.utility.Vector3dVector(self.points)
        pcd.colors = o3d.utility.Vector3dVector(self.current_colors)

        return pcd

    def export_lines(self, color=None):
        if color is None:
            color = [0, 0, 1]

        line = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(self.sensor_poses),
            lines=o3d.utility.Vector2iVector(self.lines)
        )
        line.colors = o3d.utility.Vector3dVector([color for i in range(len(self.points))])
        return line


class SimplyHeatMapper:
    def __init__(self, pcd, c: ColorMapper):
        self.pcd = pcd
        self.c = c
        self.pos_map = {}

    def add_point(self, pos, val, s_id):
        if s_id not in self.pos_map:
            self.pos_map.setdefault(s_id, {"pos": pos, "val": val})
        self.pos_map[s_id]["pos"] = np.array(pos)
        self.pos_map[s_id]["val"] = val

    def export_pcd(self):
        points = np.asarray(self.pcd.points)
        colors = []

        for p in points:
            scores = []
            vals = []
            for i, prop in self.pos_map.items():
                d = np.linalg.norm(prop["pos"] - p)
                scores.append(1 / math.pow(d, 2))
                vals.append(prop["val"])

            def softmax(x):
                y = np.exp(x)
                f_x = y / np.sum(np.exp(x))
                return f_x

            scores = np.array(scores)
            vals = np.array(vals)
            if scores.sum() >= 1.0:
                scores = softmax(scores)
            score = np.array([scores[j] * vals[j] for j in range(len(vals))]).sum()

            colors.append(self.c(score))

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        return pcd

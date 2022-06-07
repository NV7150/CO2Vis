import numpy as np

def add_point(pcd, point, scale=0.005, color=None):
    if color is None:
        color = [255, 0, 0]

    for x in range(-5, 6):
        for y in range(-5, 6):
            for z in range(-5, 6):
                curr_p = np.array(point, dtype=float)
                curr_p[0] += x * scale
                curr_p[1] += y * scale
                curr_p[2] += z * scale
                pcd.points.append(curr_p)
                pcd.colors.append(np.array(color, dtype=float))

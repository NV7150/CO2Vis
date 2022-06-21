import cv2
import numpy as np
import open3d as o3d
from CameraPosDefiner import *
from PcdLoader import *
from VisLibrary import *

targ_img_name = "sampleData/2022_06_04_12_18_45/frame_00673.jpg"
source_img_name = "sampleData/room3/IMG_2934.JPG"
c_img = cv2.imread(targ_img_name) # scan
s_img = cv2.imread(source_img_name) # camera
match, s_kp, t_kp = get_match_points(s_img, c_img)
h, res = get_match_matrix(match, s_kp, t_kp, match_rate=0.15)
# height, width, channels = s_img.shape
# img = cv2.warpPerspective(c_img, h, (width, height))
# cv2.namedWindow("img")
# cv2.imshow("img", img)
# cv2.imshow("img2", s_img)
# cv2.waitKey(0)
outer = cal_viewpoint(targ_img_name, h)
frame = Frame.from_json(targ_img_name.replace(".jpg", ".json"))
pos = frame.pose


pcd, offset = load_pcd("sampleData/room3.ply")
# new_pos = np.dot(outer, pos)
# new_pos = pos
# rot_matrix = new_pos[:,:-1][:3]
# move_matrix = new_pos[:,-1][:3]
#

new_o = transpose_with_outer(targ_img_name, outer)

# rot_matrix = np.dot(outer[:,:-1][:3],pos[:,:-1][:3])
# move_matrix = outer[:,-1][:3] + pos[:,-1][:3]
rot_matrix = new_o[:,:-1][:3]
move_matrix = new_o[:,-1][:3]

fx = frame.intrinsics[0,0]
fy = frame.intrinsics[1,1]
center = frame.intrinsics[:,2][:2]
rot_inv = np.transpose(rot_matrix)
camera_pos = move_matrix.flatten()

def transpose_pos(pos_m_in):
    pos_m = np.array(pos_m_in).reshape(3,1)
    x = (pos_m[0,0] - center[0]) / float(fx)
    y = (pos_m[1,0] - center[1]) / float(fy)
    print(x, " ", y)
    pos_m[0,0] = x
    pos_m[1,0] = y
    pos_m = np.dot(rot_inv, pos_m - move_matrix.reshape(3,1)).flatten()\
    # pos_m = np.dot(pos[:,:-1][:3], pos_m - move_matrix.reshape(3, 1)).flatten() + offset
    print(pos_m)
    pos_m = 2 * camera_pos - pos_m
    pos_m += offset
    return pos_m

s_i_read = cv2.imread(source_img_name)
width = np.array(s_i_read).shape[0]
height = np.array(s_i_read).shape[1]

ps = [
    [transpose_pos([float(i),float(j),1.0]) for i in range(0,width, int(width // 10))] for j in range(0,height,int(height // 10))
]

add_point(pcd, camera_pos + offset)
for i, row in enumerate(ps):
    for j, point_pos in enumerate(row):
        # print(p)
        add_point(pcd, point_pos, color=[0,(float(i) / len(ps)),(float(j) / len(row))])

add_point(pcd, np.array([0,1,0]), color=[1,1,0])
add_point(pcd, np.array([0,0,0]), color=[1,1,0])

o3d.visualization.draw_geometries([pcd])

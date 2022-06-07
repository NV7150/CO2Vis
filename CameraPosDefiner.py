import json
import open3d as o3d

import cv2
import numpy as np

from ThreeDLib import *
from FrameLoader import Frame


def get_keypoint(img, max_pts=500):
    detector = cv2.ORB_create(max_pts)
    return detector.detectAndCompute(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), None)


def get_match_points_knn(source_img, targ_img, max_pts=500, th=0.7, k=2):
    source_keypoints, source_desc = get_keypoint(source_img, max_pts)
    targ_keypoints, targ_desc = get_keypoint(targ_img, max_pts)

    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    match = bf.knnMatch(source_desc, targ_desc, k=k)
    match = [f for f, s in match if f.distance / s.distance < th]
    return match, source_keypoints, targ_keypoints


def get_match_points(source_img, targ_img, max_pts=500):
    source_keypoints, source_desc = get_keypoint(source_img, max_pts)
    targ_keypoints, targ_desc = get_keypoint(targ_img, max_pts)

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    match = bf.match(source_desc, targ_desc)
    return match, source_keypoints, targ_keypoints


def get_match_matrix(matches, source_pt, targ_pt, match_rate=0.15, minimum_use=10):
    matches = sorted(matches, key=lambda x: x.distance)
    use_points = matches[:int(len(matches) * match_rate)]

    if len(use_points) <= minimum_use:
        return np.zeros((3, 3))

    source_points = np.float32(
        [source_pt[match.queryIdx].pt for match in matches]
    ).reshape(-1, 1, 2)
    targ_points = np.float32(
        [targ_pt[match.queryIdx].pt for match in matches]
    ).reshape(-1, 1, 2)
    h, mask = cv2.findHomography(targ_points, source_points, cv2.RANSAC)

    return h


def cal_viewpoint(camera_file, h, obj):
    frame = Frame.from_json(f"{camera_file.split('.')[0]}.json")
    inv_a = np.linalg.inv(frame.intrinsics)
    myu = 1 / np.linalg.norm(np.dot(inv_a, h[:, :1]))
    r1 = myu * np.dot(inv_a, h[:, :1])
    r2 = myu * np.dot(inv_a, h[:, 1:2])
    r3 = np.cross(r1, r2)
    t =
    


    # img = cv2.imread(camera_file)
    # row, col, ch = img.shape
    # img = cv2.warpPerspective(img, h, (row, col))
    # dest = get_3dpos_frame(np.array([0, 0]), frame)
    # base_m = raycast2dest_mesh(frame, dest, obj)
    # new_m = np.dot(np.linalg.inv(frame.intrinsics), np.dot(h, ))


def check_camera_points(camera_imgs, scan_imgs, knn=True):
    for c_img in camera_imgs:
        for s_img in scan_imgs:
            h = 0
            if knn:
                match, s_kp, t_kp = get_match_points_knn(c_img, s_img)
                h = get_match_matrix(match, s_kp, t_kp, match_rate=1.0)
            else:
                match, s_kp, t_kp = get_match_points(c_img, s_img)
                h = get_match_matrix(match, s_kp, t_kp)






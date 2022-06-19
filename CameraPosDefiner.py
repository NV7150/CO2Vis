import json
import open3d as o3d

import cv2
import numpy as np

from collections import defaultdict

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

    if len(use_points) < minimum_use:
        return np.zeros((3, 3))

    source_points = np.float32(
        [source_pt[match.queryIdx].pt for match in use_points]
    ).reshape(-1, 1, 2)
    targ_points = np.float32(
        [targ_pt[match.trainIdx].pt for match in use_points]
    ).reshape(-1, 1, 2)
    h, mask = cv2.findHomography(targ_points, source_points, cv2.RANSAC)

    return h


def cal_viewpoint(camera_file, h):
    frame = Frame.from_json(f"{camera_file.split('.')[0]}.json")
    inv_ah = np.dot(np.linalg.inv(frame.intrinsics), h)
    myu = 1 / np.linalg.norm(np.dot(np.linalg.inv(frame.intrinsics), h[0]))
    myu2 = 1 / np.linalg.norm(np.dot(np.linalg.inv(frame.intrinsics), h[1]))
    myu = (myu + myu2) / 2
    r1 = inv_ah[:, 0].flatten()
    r2 = inv_ah[:, 1].flatten()
    r3 = np.cross(r1, r2)
    r1 /= np.linalg.norm(r1)
    r2 /= np.linalg.norm(r2)
    r3 /= np.linalg.norm(r3)
    t = myu * inv_ah[:, 2].flatten()
    # myu = 1 / np.linalg.norm(np.dot(inv_a, h[0]))
    # myu2 = 1 / np.linalg.norm(np.dot(inv_a, h[1]))
    # myu = (myu + myu2) / 2
    # r1 = myu * np.dot(inv_a, h[0])
    # r2 = myu * np.dot(inv_a, h[1])
    # r3 = np.cross(r1, r2, axis=0)
    # t = myu * np.dot(inv_a, h[2])

    # r1 = np.dot(inv_a, h[0])
    # r1 /= np.linalg.norm(r1)
    # r2 = np.dot(inv_a, h[1])
    # r2 /= np.linalg.norm(r2)
    # r3 = np.cross(r1, r2)
    # t = np.dot(inv_a, h[2])
    # t /= np.linalg.norm(t)

    # outer_matrix = np.concatenate([np.array([r1,r2,r3]), t.reshape(3,1)], axis=1)
    outer_matrix = np.concatenate([np.transpose(np.array([r1,r2,r3,t])), np.array([[0,0,0,1]])], axis=0)

    return outer_matrix

    # img = cv2.imread(camera_file)
    # row, col, ch = img.shape
    # img = cv2.warpPerspective(img, h, (row, col))
    # dest = get_3dpos_frame(np.array([0, 0]), frame)
    # base_m = raycast2dest_mesh(frame, dest, obj)
    # new_m = np.dot(np.linalg.inv(frame.intrinsics), np.dot(h, ))


def check_camera_points(camera_imgs, scan_imgs, knn=False):
    camera_points = defaultdict(object)
    for c_img in camera_imgs:
        hs = []
        for s_img in scan_imgs:
            if knn:
                match, s_kp, t_kp = get_match_points_knn(c_img, s_img)
                h = get_match_matrix(match, s_kp, t_kp, match_rate=1.0)
            else:
                match, s_kp, t_kp = get_match_points(c_img, s_img)
                h = get_match_matrix(match, s_kp, t_kp)
            h = np.dot()
            hs.append(h)
        camera_points[c_img] = np.mean(np.array(hs), axis=0)
    return dict(camera_points)






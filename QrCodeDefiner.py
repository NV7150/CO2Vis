import numpy as np

from FrameLoader import Frame
from QrCodeDecoder import QRCode, segment_image
from QrPointer import ray_cast_qr, get_qr_from_frame
from ThreeDLib import raycast2dest_mesh


def define_qr_pos(frame_pathes, pcd, trans=None, mesh_scene=None, debug=False):
    if trans is None:
        trans = np.array([0, 0, 0])

    found_qrs = {}
    found_counts = {}
    for path in frame_pathes:
        frame = Frame.from_json(path)
        if not frame.has_image:
            continue

        qrcodes = segment_image(frame.image_path)

        if len(qrcodes) < 1:
            continue

        for qrcode in qrcodes:
            qr_pos = get_qr_from_frame(qrcode, frame, trans=trans)

            if mesh_scene is not None:
                true_point, dist = raycast2dest_mesh(frame.pos + trans, qr_pos, mesh_scene)
            else:
                (rc_pos, rc_dist, true_point) = ray_cast_qr(frame, qr_pos, pcd, step=-0.02, th=0.01)

            if debug:
                if qrcode.value not in found_qrs.keys():
                    found_qrs.setdefault(qrcode.value, [])
                info = {
                    "point": true_point,
                    "origin": frame.pos + trans
                }
                found_qrs[qrcode.value].append(info)

                continue

            if qrcode.value not in found_qrs.keys():
                found_qrs.setdefault(qrcode.value, true_point)
                found_counts.setdefault(qrcode.value, 1)
            else:
                found_counts[qrcode.value] += 1
                count = found_counts[qrcode.value]
                avr = found_qrs[qrcode.value]
                new_avr = (avr * (count - 1) + true_point) / count
                found_qrs[qrcode.value] = new_avr

    return found_qrs

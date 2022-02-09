import cv2
import numpy as np
from pyzbar.pyzbar import decode, ZBarSymbol


class QRCode:
    def __init__(self, value, pos=None, size=None, points=None):
        self.value = value
        self.pos = pos
        self.size = size
        self.points = []

        if points is not None:
            for point in points:
                self.points.append(np.array([float(point.x), float(point.y)]))
            self.points = np.array(self.points, dtype=float)

    def get_point(self):
        if self.size is not None:
            return self.pos + self.size / 2
        else:
            # 重心を計算
            g1 = (self.points[0] + self.points[1] + self.points[2]) / 3
            g2 = (self.points[0] + self.points[2] + self.points[3]) / 3

            grad1 = (g1[1] - g2[1]) / (g1[0] - g2[0])
            int1 = -g1[0] * grad1 + g1[1]

            grad2 = (self.points[0, 1] - self.points[2, 1]) / (self.points[0, 0] - self.points[2, 0])
            int2 = -self.points[0, 0] * grad2 + self.points[0, 1]

            x = (int2 - int1) / (grad1 - grad2)
            y = grad1 * x + int1

            return np.array([x, y])
            # print(self.points)
            # c_o_m = ndimage.measurements.center_of_mass(self.points)

            # return np.array(c_o_m)


def decode_image(path: str):
    img = cv2.imread(path)
    value = decode(img, symbols=[ZBarSymbol.QRCODE])

    objs = []
    if value:
        for qrcode in value:
            x, y, w, h = qrcode.rect
            dec_inf = qrcode.data.decode('utf-8')
            objs.append(QRCode(dec_inf, pos=np.array([x, y]), size=np.array([w, h])))

    return objs


def segment_image(path: str):
    img = cv2.imread(path)
    value = decode(img, symbols=[ZBarSymbol.QRCODE])

    if not value:
        return []

    objs = []
    for qrcode in value:
        dec_inf = qrcode.data.decode('utf-8')
        objs.append(QRCode(dec_inf, points=qrcode.polygon))

    return objs

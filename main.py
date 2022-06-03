import numpy as np
from Visualizer2 import visualize


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
    visualize(
        "sampleData/Lab.ply",
        "sampleData/Lab.obj",
        "sampleData/Lab/2022_03_02_14_20_48",
        random_data=True
    )

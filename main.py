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
        "sampleData/bus.ply",
        "sampleData/bus2/textured_output.obj",
        "sampleData/bus2",
        voxel=0.1,
        reflesh_rate=5,
        save_file="sampleData/bus2.json",
        new_api=True,
        search_bus="EX_HADANO"
        # random_data=True
    )

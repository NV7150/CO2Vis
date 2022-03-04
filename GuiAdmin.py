import open3d.visualization.gui as gui
import open3d as o3d
import numpy as np

from ThreadingTools import Locker


class LabelData:
    pos: np.ndarray
    label: str


class GuiWrapper:
    def __init__(self):
        app = gui.Application.instance
        app.initialize()

        w = app.create_window("Open3D - 3D Labels", 1024, 768)
        widget3d = gui.SceneWidget()
        w.add_child(widget3d)
        widget3d.scene = o3d.visualization.rendering.Open3DScene(w.renderer)

        self.app = app
        self.window = w
        self.widget = widget3d

        mat = o3d.visualization.rendering.MaterialRecord()
        mat.shader = "defaultUnlit"
        mat.point_size = 5 * w.scaling
        mat.base_color = (1, 1, 1, 0.7)

        self.default_mat = mat

        self.labels = []

        self.locker = Locker()

    def update_geometry(self, pcd):
        with self.locker:
            self.widget.scene.clear_geometry()
            self.widget.scene.add_geometry("Points", pcd, self.default_mat)

    def update_labels(self, labels):
        with self.locker:
            for label in self.labels:
                self.widget.remove_3d_label(label)

            self.labels = []
            for n_label in labels:
                self.labels.append(self.widget.add_3d_label(n_label.pos, n_label.label))

    def show_in_tick(self):
        with self.locker:
            return self.app.run_one_tick()

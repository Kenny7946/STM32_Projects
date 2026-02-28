import sys
import numpy as np
from PyQt6 import QtWidgets, QtCore
import pyqtgraph.opengl as gl
import pyqtgraph as pg


# ==============================
# Beispiel: Dummy Pose Provider
# ==============================
def example_pose():
    # Hier später deine compute_hand_pose() einsetzen
    return {
        "thumb":  [np.array([0,0,0]),
                   np.array([0.01,0.03,0.01]),
                   np.array([0.02,0.05,0.02])],

        "index":  [np.array([0.02,0,0]),
                   np.array([0.02,0.04,0.01]),
                   np.array([0.02,0.07,0.02]),
                   np.array([0.02,0.09,0.03])],

        "middle": [np.array([0,0,0]),
                   np.array([0,0.05,0.01]),
                   np.array([0,0.09,0.02]),
                   np.array([0,0.12,0.03])],

        "ring":   [np.array([-0.02,0,0]),
                   np.array([-0.02,0.04,0.01]),
                   np.array([-0.02,0.07,0.02]),
                   np.array([-0.02,0.09,0.03])],

        "pinky":  [np.array([-0.04,0,0]),
                   np.array([-0.04,0.03,0.01]),
                   np.array([-0.04,0.05,0.02]),
                   np.array([-0.04,0.07,0.03])]
    }


# ==============================
# Hand Visualizer
# ==============================
class HandVisualizer(gl.GLViewWidget):

    def __init__(self):
        super().__init__()

        self.setCameraPosition(distance=0.3)
        self.opts['center'] = pg.Vector(0, 0.05, 0)

        # Koordinatensystem
        axis = gl.GLAxisItem()
        axis.setSize(0.1, 0.1, 0.1)
        self.addItem(axis)

        self.finger_lines = {}

    def update_hand(self, pose):
        for finger, joints in pose.items():

            points = np.array(joints)

            if finger not in self.finger_lines:
                line = gl.GLLinePlotItem(
                    pos=points,
                    width=3,
                    antialias=True
                )
                self.finger_lines[finger] = line
                self.addItem(line)
            else:
                self.finger_lines[finger].setData(pos=points)


# ==============================
# Main Window
# ==============================
class MainWindow(QtWidgets.QMainWindow):

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Hand Tracking 3D")
        self.resize(900, 700)

        self.viewer = HandVisualizer()
        self.setCentralWidget(self.viewer)

        # Timer für Live-Update (60 FPS)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_scene)
        self.timer.start(16)

    def update_scene(self):
        pose = example_pose()  # später: compute_hand_pose(...)
        self.viewer.update_hand(pose)


# ==============================
# Start
# ==============================
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
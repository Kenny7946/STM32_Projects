import sys
import numpy as np
from PyQt6 import QtWidgets, QtCore
import pyqtgraph.opengl as gl
import pyqtgraph as pg
from scipy.spatial.transform import Rotation as R

# -------------------------------
# Beispiel: Dummy Sensor Daten
# -------------------------------
def example_hand_sensor():
    """
    Gibt eine Euler-Handrotation zurück.
    Roll, Pitch, Yaw in Grad.
    """
    import time
    t = time.time()
    # Roll=±30°, Pitch=±20°, Yaw=±45° sinusförmig
    roll = 90.0 * np.sin(t)
    pitch = 0.0 * np.sin(t/2)
    yaw = 0.0 * np.sin(t/3)
    return {"euler": [roll, pitch, yaw]}


# -------------------------------
# Visualizer nur für Handrotation
# -------------------------------
class HandFrameVisualizer(gl.GLViewWidget):
    def __init__(self):
        super().__init__()
        self.setCameraPosition(distance=0.3)
        self.opts['center'] = pg.Vector(0, 0, 0)

        # Koordinatensystem der Welt
        # Koordinatensystem der Welt (X=Rot, Y=Grün, Z=Blau)
        axis_length = 0.1
        origin = np.array([0, 0, 0])

        # X-Achse = Rot
        x_axis = gl.GLLinePlotItem(
            pos=np.array([origin, origin + np.array([axis_length, 0, 0])]),
            color=(1, 0, 0, 1),
            width=3,
            antialias=True
        )
        self.addItem(x_axis)

        # Y-Achse = Grün
        y_axis = gl.GLLinePlotItem(
            pos=np.array([origin, origin + np.array([0, axis_length, 0])]),
            color=(0, 1, 0, 1),
            width=3,
            antialias=True
        )
        self.addItem(y_axis)

        # Z-Achse = Blau
        z_axis = gl.GLLinePlotItem(
            pos=np.array([origin, origin + np.array([0, 0, axis_length])]),
            color=(0, 0, 1, 1),
            width=3,
            antialias=True
        )
        self.addItem(z_axis)

        # Handachsen
        self.hand_axes = []
        colors = [(1,0,0,1),(0,1,0,1),(0,0,1,1)]  # X, Y, Z
        for c in colors:
            axis = gl.GLLinePlotItem(pos=np.zeros((2,3)), width=3, antialias=True, color=c)
            self.hand_axes.append(axis)
            self.addItem(axis)

    def update_hand_rotation(self, euler):
        """
        euler = [roll, pitch, yaw] in Grad
        """
        roll, pitch, yaw = euler
        rot = R.from_euler("XYZ", [roll, pitch, yaw], degrees=True)

        origin = np.array([0,0,0])
        axis_vectors = [np.array([0.05,0,0]), np.array([0,0.05,0]), np.array([0,0,0.05])]  # X,Y,Z
        for i, vec in enumerate(axis_vectors):
            rotated = rot.apply(vec)
            self.hand_axes[i].setData(pos=np.array([origin, origin + rotated]))


# -------------------------------
# Qt Main Window
# -------------------------------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Hand Rotation Debug")
        self.resize(600,600)

        self.viewer = HandFrameVisualizer()
        self.setCentralWidget(self.viewer)

        # Timer für Updates
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_scene)
        self.timer.start(16)  # ~60 FPS

    def update_scene(self):
        sensor = example_hand_sensor()
        self.viewer.update_hand_rotation(sensor["euler"])


# -------------------------------
# Start
# -------------------------------
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
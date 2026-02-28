import sys
import numpy as np
from PyQt6 import QtWidgets, QtCore
import pyqtgraph.opengl as gl
import pyqtgraph as pg


class Hand3DViewer(gl.GLViewWidget):
    """
    OpenGL Viewer für Handtracking.
    Erwartet Pose-Format:
    {
        "finger": [np.array([x,y,z]), ...]
    }
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        # Kamera
        self.setCameraPosition(distance=0.3)
        self.opts['center'] = pg.Vector(0, 0.05, 0)

        # Koordinatenachsen
        axis = gl.GLAxisItem()
        axis.setSize(0.1, 0.1, 0.1)
        self.addItem(axis)

        self.finger_lines = {}

    def update_hand(self, pose: dict):
        """Aktualisiert die Darstellung."""
        if pose is None:
            return

        for finger, joints in pose.items():
            points = np.array(joints)

            if finger not in self.finger_lines:
                line = gl.GLLinePlotItem(
                    pos=points,
                    width=4,
                    antialias=True,
                )
                self.finger_lines[finger] = line
                self.addItem(line)
            else:
                self.finger_lines[finger].setData(pos=points)


class HandTrackingWindow(QtWidgets.QMainWindow):
    """
    Fertiges Fenster für Live-Handtracking.

    pose_provider: Funktion oder Callable, das eine Pose zurückgibt.
    """

    def __init__(self, pose_provider=None, update_hz=60):
        super().__init__()

        self.setWindowTitle("Hand Tracking 3D")
        self.resize(900, 700)

        self.viewer = Hand3DViewer()
        self.setCentralWidget(self.viewer)

        self.pose_provider = pose_provider

        # Timer für Live-Updates
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._update_scene)
        self.timer.start(int(1000 / update_hz))

    def _update_scene(self):
        if self.pose_provider is None:
            return

        pose = self.pose_provider()
        self.viewer.update_hand(pose)


# ---------------------------------------------------------
# OPTIONAL: Viewer in bestehende Qt-App integrieren
# ---------------------------------------------------------
def start_viewer(pose_provider):
    """
    Startet den Viewer als eigenständige Anwendung.
    """
    app = QtWidgets.QApplication(sys.argv)
    window = HandTrackingWindow(pose_provider)
    window.show()
    sys.exit(app.exec())
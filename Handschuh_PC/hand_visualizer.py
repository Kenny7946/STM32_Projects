import sys
import numpy as np
from PyQt6 import QtWidgets, QtCore
import pyqtgraph.opengl as gl
import pyqtgraph as pg
import config

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

        self.logger = None

        # === Zentrales Widget + Layout ===
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)

        layout = QtWidgets.QVBoxLayout()
        central_widget.setLayout(layout)

        # === 3D Viewer ===
        self.viewer = Hand3DViewer()
        layout.addWidget(self.viewer)

        # === Debug Button ===
        self.debug_button = QtWidgets.QPushButton("Start Logging")
        self.debug_button.setCheckable(True)
        self.debug_button.clicked.connect(self._toggle_logging)

        self.source_button = QtWidgets.QPushButton("LIVE")
        self.source_button.setCheckable(True)
        self.source_button.clicked.connect(self.toggle_source)        

        toolbar = QtWidgets.QToolBar()
        toolbar.addWidget(self.debug_button)
        toolbar.addWidget(self.source_button)
        self.addToolBar(toolbar)

        # Pose Provider
        self.pose_provider = pose_provider

        # Timer für Live-Updates
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._update_scene)
        self.timer.start(int(1000 / update_hz))

    # ==========================
    # Button Callback
    # ==========================
    def _on_debug_button_clicked(self):
        print("🔘 Debug Button wurde gedrückt!")

    def toggle_source(self, checked):
        if checked:
            config.MODE = "replay"   # "live" oder "replay"
        else:
            config.MODE = "live"   # "live" oder "replay"            

        self.source_button.setText(config.MODE)
        print(f"Toggled data provider to {config.MODE}")

    # ==========================
    # Update Loop
    # ==========================
    def _update_scene(self):
        if self.pose_provider is not None:
            pose = self.pose_provider()
            if pose is not None:
                self.viewer.update_hand(pose)

    def set_logger(self, logger):
        self.logger = logger


    def _toggle_logging(self, checked):
        if self.logger is None:
            print("Kein Logger gesetzt!")
            return

        if checked:
            # Logging läuft gerade → Button gedrückt, Logging stoppen
            self.logger.set_enabled(False)
            self.debug_button.setText("Logging pausiert…")

            # Dialog für neuen Dateinamen
            filename, ok = QtWidgets.QInputDialog.getText(
                self, "Neues Logfile", "Dateiname für neues Log (optional):"
            )

            # Neues File starten
            if not ok or filename == "" or filename == " ":
                filename = None  # Abbruch → automatisch Zeitstempel
            self.logger.start_new_file(filename)

            # Button wieder aktiv anzeigen
            self.debug_button.setChecked(True)
            self.debug_button.setText("Logging läuft…")
        else:
            # Falls Button deaktiviert → Logging stoppen
            self.logger.set_enabled(False)
            self.debug_button.setText("Neues Log starten")
            


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
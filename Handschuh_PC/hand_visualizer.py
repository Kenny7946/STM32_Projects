import sys
import numpy as np
from PyQt6 import QtWidgets, QtCore
from PyQt6.QtWidgets import QFileDialog
import pyqtgraph.opengl as gl
import pyqtgraph as pg
from pathlib import Path
import config
from replay_controller import ReplayController
from hand_pose_estimator import HandPoseEstimator
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
        self.replay = None
        self.estimator = HandPoseEstimator()

        # === Zentrales Widget + Layout ===
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)

        layout = QtWidgets.QVBoxLayout()
        central_widget.setLayout(layout)

        # === 3D Viewer ===
        self.viewer = Hand3DViewer()

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
        

        # --- Media Controls ---
        self.play_button = QtWidgets.QPushButton("▶")
        self.step_back_button = QtWidgets.QPushButton("⏮")
        self.step_fwd_button = QtWidgets.QPushButton("⏭")

        self.slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(1000)

        # --- Sensor Graph ---
        self.graph = pg.PlotWidget()
        self.graph.setBackground("k")
        self.graph.showGrid(x=True, y=True)

        self.graph.addLegend()

        layout.addWidget(self.graph)

        self.gyro_curve_x = self.graph.plot(pen=pg.mkPen('r', width=1), name="Gyro X")
        self.gyro_curve_y = self.graph.plot(pen=pg.mkPen('g', width=1), name="Gyro Y")
        self.gyro_curve_z = self.graph.plot(pen=pg.mkPen('b', width=1), name="Gyro Z")

        self.accel_curve_x = self.graph.plot(pen=pg.mkPen('y', width=1), name="Accel X")
        self.accel_curve_y = self.graph.plot(pen=pg.mkPen('m', width=1), name="Accel Y")
        self.accel_curve_z = self.graph.plot(pen=pg.mkPen('c', width=1), name="Accel Z")

        # vertikale Frame Linie
        self.frame_line = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen('w'))
        self.graph.addItem(self.frame_line)

        self.time_label = QtWidgets.QLabel("00:00 / 00:00")

        toolbar.addWidget(self.step_back_button)
        toolbar.addWidget(self.play_button)
        toolbar.addWidget(self.step_fwd_button)
        toolbar.addWidget(self.slider)
        toolbar.addWidget(self.time_label)

        self.play_button.clicked.connect(self.toggle_play)
        self.step_fwd_button.clicked.connect(self.step_forward)
        self.step_back_button.clicked.connect(self.step_backward)
        self.slider.sliderMoved.connect(self.seek_position)

        layout.addWidget(self.viewer, stretch=3)
        layout.addWidget(self.graph, stretch=1)

        # Pose Provider
        self.pose_provider = pose_provider

        # Timer für Live-Updates
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._update_scene)
        self.timer.start(int(1000 / update_hz))

        self.addToolBar(toolbar)

    # ==========================
    # Button Callback
    # ==========================
    def _on_debug_button_clicked(self):
        print("🔘 Debug Button wurde gedrückt!")

    def toggle_source(self, checked):
        if checked:
            self.switch_to_replay()
        else:
            self.switch_to_live()       

        self.source_button.setText(config.MODE)
        print(f"Toggled data provider to {config.MODE}")

    # ==========================
    # Update Loop
    # ==========================
    def _update_scene(self):

        # -------- LIVE --------
        if config.MODE == "live":
            if self.pose_provider:
                pose = self.pose_provider()
                if pose:
                    self.viewer.update_hand(pose)
            return


        # -------- REPLAY --------
        if config.MODE == "replay" and self.replay:

            dt = self.timer.interval() / 1000.0
            self.replay.update(dt)

            sensors = self.replay.get_current_sensors()
            pose = self.estimator.compute_pose(sensors)

            self.viewer.update_hand(pose)

            # Slider Update
            self.slider.blockSignals(True)
            self.slider.setValue(self.replay.current_index)
            self.slider.blockSignals(False)

            self.update_time_label()

            self.frame_line.setValue(self.replay.current_index)

    def update_time_label(self):
        if not self.replay:
            return

        current_ms = self.replay.get_current_time_ms()
        total_ms = self.replay.get_total_time_ms()

        current_frame, total_frames = self.replay.get_frame_info()

        def fmt(ms):
            seconds = ms // 1000
            milliseconds = ms % 1000
            minutes = seconds // 60
            seconds = seconds % 60
            return f"{minutes:02}:{seconds:02}.{milliseconds:03}"

        time_str = f"{fmt(current_ms)} / {fmt(total_ms)}"
        frame_str = f"Frame {current_frame + 1} / {total_frames}"

        self.time_label.setText(f"{time_str}   |   {frame_str}")

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

    def toggle_play(self):
        if not self.replay:
            return

        if self.replay.playing:
            self.replay.pause()
            self.play_button.setText("▶")
        else:
            self.replay.play()
            self.play_button.setText("⏸")

    def step_forward(self):
        if self.replay:
            self.replay.step_forward()

    def step_backward(self):
        if self.replay:
            self.replay.step_backward()

    def seek_position(self, value):
        if not self.replay:
            return

        self.replay.current_index = value

    def select_replay_file(self):
        base_dir = Path(__file__).resolve().parent
        logs_dir = base_dir / "logs"

        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Replay-Datei auswählen",
            str(logs_dir),
            "JSONL Dateien (*.jsonl)"
        )

        if filename:
            config.REPLAY_FILE = filename
            print("Replay-Datei gesetzt:", filename)
            return filename

        return None
    
    def switch_to_replay(self):
        filename = self.select_replay_file()
        if not filename:
            return

        self.replay = ReplayController(filename)
        self._load_graph_data()
        self.slider.setMinimum(0)
        self.slider.setMaximum(self.replay.get_total_frames() - 1)
        config.MODE = "replay"

        self.source_button.setText("REPLAY")
        print("Modus: REPLAY")
        print(f"REPLAY_FILENAME: {config.REPLAY_FILENAME}")

    def switch_to_live(self):
            config.MODE = "live"
            self.source_button.setText("LIVE")
            self.replay = None   

    def _load_graph_data(self):
        if not self.replay:
            return
        print(self.replay.data[0])

        frames = len(self.replay.data)

        gyro_x = []
        gyro_y = []
        gyro_z = []

        accel_x = []
        accel_y = []
        accel_z = []

        for entry in self.replay.data:
            sensors = entry["sensors"]

            gx, gy, gz = sensors["gyro"]
            ax, ay, az = sensors["accel"]

            gyro_x.append(gx)
            gyro_y.append(gy)
            gyro_z.append(gz)

            accel_x.append(ax)
            accel_y.append(ay)
            accel_z.append(az)

        x_axis = list(range(frames))

        self.gyro_curve_x.setData(x_axis, gyro_x)
        self.gyro_curve_y.setData(x_axis, gyro_y)
        self.gyro_curve_z.setData(x_axis, gyro_z)

        self.accel_curve_x.setData(x_axis, accel_x)
        self.accel_curve_y.setData(x_axis, accel_y)
        self.accel_curve_z.setData(x_axis, accel_z)

        self.graph.setXRange(0, frames)
                            


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
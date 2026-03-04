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
from sensor_plot_widget import SensorPlotWidget
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

        self.sensor_values = None

        self.max_live_samples = 200
        self.live_index = 0

        self.live_buffer = {
            "gyro_x": [],
            "gyro_y": [],
            "gyro_z": [],
            "accel_x": [],
            "accel_y": [],
            "accel_z": []
        }

        # === Zentrales Widget + Layout ===
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QtWidgets.QVBoxLayout()
        central_widget.setLayout(main_layout)

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

        h_layout = QtWidgets.QHBoxLayout()
        main_layout.addLayout(h_layout, stretch=1)
        h_layout.addWidget(self.viewer, stretch=3)
        
        self.sensor_widget = SensorPlotWidget()
        h_layout.addWidget(self.sensor_widget, stretch=2)

        # Sensoren hinzufügen
        self.sensor_widget.add_sensor_curve("Gyro X", 'r')
        self.sensor_widget.add_sensor_curve("Gyro Y", 'g')
        self.sensor_widget.add_sensor_curve("Gyro Z", 'b')
        self.sensor_widget.add_sensor_curve("Accel X", 'y')
        self.sensor_widget.add_sensor_curve("Accel Y", 'm')
        self.sensor_widget.add_sensor_curve("Accel Z", 'c')

        def on_graph_click(frame, values):
            print(f"Frame {frame}: {values}")

        self.sensor_widget.click_callback(on_graph_click)


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

        sensors = None

        # -------- LIVE --------
        if config.MODE == "live":
            if self.pose_provider:
                pose = self.pose_provider()
                
                if pose:
                    self.viewer.update_hand(pose)
            
            sensors = self.sensor_values
            if sensors:
                # --- Live Buffer füllen ---
                self.live_buffer["gyro_x"].append(sensors["gyro"][0])
                self.live_buffer["gyro_y"].append(sensors["gyro"][1])
                self.live_buffer["gyro_z"].append(sensors["gyro"][2])

                self.live_buffer["accel_x"].append(sensors["accel"][0])
                self.live_buffer["accel_y"].append(sensors["accel"][1])
                self.live_buffer["accel_z"].append(sensors["accel"][2])

                # Älteste Werte entfernen wenn > max_live_samples
                for key in self.live_buffer:
                    if len(self.live_buffer[key]) > self.max_live_samples:
                        self.live_buffer[key].pop(0)

                # X-Achse erzeugen (0..N-1)
                x_vals = list(range(len(self.live_buffer["gyro_x"])))

                # --- Sensorplots aktualisieren ---
                self.sensor_widget.update_data("Gyro X", x_vals, self.live_buffer["gyro_x"])
                self.sensor_widget.update_data("Gyro Y", x_vals, self.live_buffer["gyro_y"])
                self.sensor_widget.update_data("Gyro Z", x_vals, self.live_buffer["gyro_z"])

                self.sensor_widget.update_data("Accel X", x_vals, self.live_buffer["accel_x"])
                self.sensor_widget.update_data("Accel Y", x_vals, self.live_buffer["accel_y"])
                self.sensor_widget.update_data("Accel Z", x_vals, self.live_buffer["accel_z"])

            return 
    
        # -------- REPLAY --------
        elif config.MODE == "replay" and self.replay:

            dt = self.timer.interval() / 1000.0
            self.replay.update(dt)
            frame = self.replay.current_index
            self.sensor_widget.set_cursor(frame)

            sensors = self.replay.get_current_sensors()
            pose = self.estimator.compute_pose(sensors)
            self.viewer.update_hand(pose)
            sensors = self.replay.data[frame]["sensors"]

            # Gyro & Accel aktualisieren
            self.sensor_widget.update_data("Gyro X", list(range(len(self.replay.data))),  [s["sensors"]["gyro"][0] for s in self.replay.data])
            self.sensor_widget.update_data("Gyro Y", list(range(len(self.replay.data))),  [s["sensors"]["gyro"][1] for s in self.replay.data])
            self.sensor_widget.update_data("Gyro Z", list(range(len(self.replay.data))),  [s["sensors"]["gyro"][2] for s in self.replay.data])
            self.sensor_widget.update_data("Accel X", list(range(len(self.replay.data))), [s["sensors"]["accel"][0] for s in self.replay.data])
            self.sensor_widget.update_data("Accel Y", list(range(len(self.replay.data))), [s["sensors"]["accel"][1] for s in self.replay.data])
            self.sensor_widget.update_data("Accel Z", list(range(len(self.replay.data))), [s["sensors"]["accel"][2] for s in self.replay.data])        

            # Slider Update
            self.slider.blockSignals(True)
            self.slider.setValue(self.replay.current_index)
            self.slider.blockSignals(False)

            self.update_time_label()


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
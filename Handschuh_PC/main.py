import sys
import asyncio
from queue import Queue

from PyQt6 import QtWidgets

from hand_ble import HandBLEReceiver
from hand_pose_estimator import HandPoseEstimator
from hand_visualizer import HandTrackingWindow
from data_logger import HandTrackingLogger
from pathlib import Path
from datetime import datetime
from live_provider import LiveSensorProvider
from replay_provider import LogReplayProvider
import config




# -------------------------------
# Shared Queue für Sensoren
# -------------------------------
sensor_queue = Queue(maxsize=10)  # Puffer für BLE-Daten


# -------------------------------
# BLE Callback
# -------------------------------
def handle_sensor_data(sensors):
    """
    BLE Callback: speichert Sensor-Daten in Queue.
    """
    if config.MODE == "live":
        if sensor_queue.full():
            sensor_queue.get_nowait()
        sensor_queue.put(sensors)


# -------------------------------
# Pose Provider für Visualizer
# -------------------------------
estimator = HandPoseEstimator()
base_dir = Path(__file__).resolve().parent
log_dir = base_dir / "logs"
log_dir.mkdir(exist_ok=True)

logger = HandTrackingLogger(log_dir="C:/Markus/Coding/STM32/Handschuh_PC/logs")

live_sensor_provider = LiveSensorProvider(sensor_queue)

def pose_provider():
    try:
        if config.MODE != "live":
            return None

        sensors = live_sensor_provider.get_next()
        if sensors is None:
            return None

        pose = estimator.compute_pose(sensors)

        logger.log(
            sensors=sensors,
            pose=pose,
            position=None
        )

        return pose

    except Exception as e:
        print("Pose Error:", e)
        return None


# -------------------------------
# BLE Async-Loop in Thread
# -------------------------------
def start_ble_loop():
    """
    Async BLE loop wird in eigenem Thread gestartet.
    """
    async def run():
        ble = HandBLEReceiver(name="XX-STM32")  # optional: address="AA:BB:CC:DD:EE:FF"
        await ble.find_device()
        try:
            await ble.start(handle_sensor_data)
        except asyncio.CancelledError:
            print("BLE Loop cancelled")
        finally:
            await ble.stop()  # <- disconnect beim Exit
            print("BLE sauber getrennt")

    asyncio.run(run())


# -------------------------------
# Qt App
# -------------------------------
def main():
    import threading

    app = QtWidgets.QApplication(sys.argv)
    app.aboutToQuit.connect(logger.close)

    # Visualizer starten
    window = HandTrackingWindow(pose_provider)
    window.set_logger(logger)
    window.show()

    # BLE Loop in separatem Thread
    ble_thread = threading.Thread(target=start_ble_loop, daemon=True)
    ble_thread.start()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
import sys
import asyncio
from queue import Queue

from PyQt6 import QtWidgets

from hand_ble import HandBLEReceiver
from hand_pose_estimator import HandPoseEstimator
from hand_visualizer import HandTrackingWindow


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
    if sensor_queue.full():
        sensor_queue.get_nowait()
    sensor_queue.put(sensors)


# -------------------------------
# Pose Provider für Visualizer
# -------------------------------
estimator = HandPoseEstimator()

def pose_provider():
    """
    Liest neueste Sensor-Daten aus Queue und berechnet Pose.
    """
    try:
        sensors = sensor_queue.get_nowait()
        pose = estimator.compute_pose(sensors)
        return pose
    except:
        #print("Konnte Pose nicht bestimmen")
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
        await ble.start(handle_sensor_data)

    asyncio.run(run())


# -------------------------------
# Qt App
# -------------------------------
def main():
    import threading

    app = QtWidgets.QApplication(sys.argv)

    # Visualizer starten
    window = HandTrackingWindow(pose_provider)
    window.show()

    # BLE Loop in separatem Thread
    ble_thread = threading.Thread(target=start_ble_loop, daemon=True)
    ble_thread.start()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
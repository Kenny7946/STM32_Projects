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
import http.server
import socketserver
import threading
import asyncio
import websockets
import json

async_loop = None

connected_clients = set()

# Thread-sicher broadcasten
def send_data_threadsafe(data):
    global async_loop
    if async_loop is None:
        return

    # run_coroutine_threadsafe gibt ein Future zurück, kann ignoriert werden
    asyncio.run_coroutine_threadsafe(broadcast(data), async_loop)

async def ws_handler(websocket):
    print("Browser verbunden")
    connected_clients.add(websocket)

    try:
        async for _ in websocket:
            pass
    finally:
        connected_clients.remove(websocket)
        print("Browser getrennt")


async def broadcast(data):
    if not connected_clients:
        return

    message = json.dumps(data)

    await asyncio.gather(
        *[client.send(message) for client in connected_clients],
        return_exceptions=True
    )

def start_async_server():
    global async_loop
    async_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(async_loop)

    async def runner():
        server = await websockets.serve(ws_handler, "0.0.0.0", 8080)
        print("WS Server läuft auf ws://localhost:8080")
        await server.wait_closed()

    async_loop.run_until_complete(runner())
    async_loop.run_forever()


import threading
threading.Thread(target=start_async_server, daemon=True).start()

# HTTP-Handler
class Handler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        # Debug-Ausgabe optional
        print(format % args)

# Server starten
httpd = socketserver.TCPServer(("", 8000), Handler)

def serve():
    print(f"Serving at port {8000}")
    httpd.serve_forever()

threading.Thread(target=serve, daemon=True).start()

# -------------------------------
# Shared Queue für Sensoren
# -------------------------------
sensor_queue = Queue(maxsize=10)  # Puffer für BLE-Daten
window = None

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
        sensors = live_sensor_provider.get_next()
        if sensors is None:
            return None, None
        
        if config.MODE == "live":
            window.sensor_values = sensors

        pose, angles = estimator.compute_pose(sensors)

        logger.log(
            sensors=sensors,
            angles=angles,
            pose=pose,
            position=None
        )

        return pose, angles

    except Exception as e:
        print("Pose Error:", e)
        return None, None


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
    global window
    window = HandTrackingWindow(pose_provider, send_data_threadsafe)
    window.set_logger(logger)
    window.show()

    # BLE Loop in separatem Thread
    ble_thread = threading.Thread(target=start_ble_loop, daemon=True)
    ble_thread.start()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
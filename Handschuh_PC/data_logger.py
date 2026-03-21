import json
import time
from datetime import datetime
from pathlib import Path
import numpy as np

class HandTrackingLogger:
    """
    Logger für Handtracking-Daten.
    - JSON Lines Format
    - Neues File kann jederzeit gestartet werden
    """

    def __init__(self, log_dir="logs"):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.file = None
        self.enabled = False
        self.current_filepath = None

    def start_new_file(self, filename=None):
        """
        Öffnet ein neues Logfile. 
        Optional kann ein eigener Dateiname angegeben werden.
        """
        if self.file:
            self.file.close()

        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"handtracking_{timestamp}.jsonl"
        elif not filename.endswith(".jsonl"):
            filename += ".jsonl"

        self.current_filepath = self.log_dir / filename
        self.file = open(self.current_filepath, "a", buffering=1)
        print(f"[Logger] Neues Logfile erstellt: {self.current_filepath}")
        self.enabled = True

    def set_enabled(self, state: bool):
        self.enabled = state
        status = "aktiv" if state else "inaktiv"
        print(f"[Logger] Logging ist jetzt {status}")

    def _serialize_pose(self, pose):
        if pose is None:
            return None
        serialized = {}
        for finger, joints in pose.items():
            serialized[finger] = [j.tolist() if isinstance(j, np.ndarray) else j for j in joints]
        return serialized

    def log(self, sensors, angles, pose=None, position=None):
        if not self.enabled or self.file is None:
            return

        entry = {
            "timestamp": datetime.utcnow().isoformat(),
            "unix_time": time.time(),
            "position": position.tolist() if isinstance(position, np.ndarray) else position,
            "sensors": sensors,
            "pose": self._serialize_pose(pose),
            "angles": angles
        }
        self.file.write(json.dumps(entry) + "\n")

    def close(self):
        if self.file:
            self.file.close()
            self.file = None
            self.enabled = False
            print("[Logger] Logfile geschlossen")
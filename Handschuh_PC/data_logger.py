import json
import time
from datetime import datetime
import numpy as np
from pathlib import Path


class HandTrackingLogger:
    """
    Speichert Handtracking-Daten fortlaufend als JSON Lines.

    Jede Zeile enthält:
        timestamp
        position
        sensors
        pose
    """

    def __init__(self, filepath="handtracking_log.jsonl"):
        self.filepath = Path(filepath)
        self.filepath.parent.mkdir(parents=True, exist_ok=True)

        self.file = open(self.filepath, "a", buffering=1)  # line buffered

    def _serialize_pose(self, pose):
        """Konvertiert numpy arrays → Listen."""
        if pose is None:
            return None

        serialized = {}
        for finger, joints in pose.items():
            serialized[finger] = [j.tolist() if isinstance(j, np.ndarray) else j for j in joints]
        return serialized

    def log(self, sensors, pose=None, position=None):
        entry = {
            "timestamp": datetime.utcnow().isoformat(),
            "unix_time": time.time(),
            "position": position.tolist() if isinstance(position, np.ndarray) else position,
            "sensors": sensors,
            "pose": self._serialize_pose(pose),
        }

        self.file.write(json.dumps(entry) + "\n")

    def close(self):
        self.file.close()
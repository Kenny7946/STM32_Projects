from pathlib import Path

MODE = "live"   # "live" oder "replay"

REPLAY_PATH = Path(__file__).resolve().parent / "logs"
REPLAY_FILENAME = "temp123.jsonl"
REPLAY_FILE = REPLAY_PATH / REPLAY_FILENAME
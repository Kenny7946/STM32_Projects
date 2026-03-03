import json
import bisect

class ReplayController:
    def __init__(self, logfile):
        self.data = []
        self.timestamps = []
        self.current_index = 0
        self.playing = False
        self.speed = 1.0

        self._load(logfile)

    def _load(self, logfile):
        with open(logfile, "r") as f:
            for line in f:
                entry = json.loads(line)
                self.data.append(entry)
                self.timestamps.append(entry["unix_time"])

        self.start_time = self.timestamps[0]
        self.end_time = self.timestamps[-1]
        self.duration = self.end_time - self.start_time

    # ---------- Playback ----------

    def play(self):
        self.playing = True

    def pause(self):
        self.playing = False

    def step_forward(self):
        self.current_index = min(self.current_index + 1, len(self.data) - 1)

    def step_backward(self):
        self.current_index = max(self.current_index - 1, 0)

    def seek_time(self, t_normalized):
        """
        t_normalized: 0.0 - 1.0
        """
        absolute_time = self.start_time + t_normalized * self.duration
        self.current_index = bisect.bisect_left(self.timestamps, absolute_time)

    def update(self, dt):
        """
        dt in seconds (vom Qt Timer)
        """
        if not self.playing:
            return

        current_time = self.timestamps[self.current_index]
        target_time = current_time + dt * self.speed

        new_index = bisect.bisect_left(self.timestamps, target_time)
        self.current_index = min(new_index, len(self.data) - 1)

    def get_current_sensors(self):
        return self.data[self.current_index]["sensors"]

    def get_progress(self):
        current_time = self.timestamps[self.current_index]
        return (current_time - self.start_time) / self.duration
    
    def get_current_time_ms(self):
        current = self.timestamps[self.current_index] - self.start_time
        return int(current * 1000)

    def get_total_time_ms(self):
        return int(self.duration * 1000)

    def get_frame_info(self):
        return self.current_index, len(self.data)

    def get_total_frames(self):
        return len(self.data)
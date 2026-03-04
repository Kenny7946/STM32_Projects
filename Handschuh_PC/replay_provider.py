import json
import time

class LogReplayProvider:
    def __init__(self, logfile, realtime=True, loop=False):
        self.logfile = logfile
        self.realtime = realtime
        self.loop = loop

        self.data = []
        self.index = 0

        self._load()

    def setReplayFile(self, replay_file):
        if(self.logfile == replay_file):
            return
        
        self.logfile = replay_file
        self._load()
        self.index = 0
        

    def _load(self):
        self.data = []

        with open(self.logfile, "r") as f:
            for line in f:
                self.data.append(json.loads(line))

        if len(self.data) < 2:
            self.realtime = False

    def get_next(self):
        #print(f"Next: {self.index}")
        if self.index >= len(self.data):
            if self.loop:
                self.index = 0
            else:
                return None

        entry = self.data[self.index]
        self.index += 1

        if self.realtime and self.index < len(self.data):
            t0 = entry["unix_time"]
            t1 = self.data[self.index]["unix_time"]
            dt = max(0, t1 - t0)
            time.sleep(dt)

        return entry["sensors"]
from queue import Queue

class LiveSensorProvider:
    def __init__(self, queue: Queue):
        self.queue = queue

    def get_next(self):
        try:
            return self.queue.get_nowait()
        except:
            return None
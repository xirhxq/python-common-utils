# core/TimeBuffer.py

from collections import deque

class TimeBuffer:
    def __init__(self, name='Buffer', max_age=0.15, time_func=None):
        self.buffer = deque()
        self.name = name
        self.max_age = max_age
        self.time_func = time_func or self.default_time

    @staticmethod
    def default_time():
        import time
        return time.time()

    @property
    def is_empty(self):
        return len(self.buffer) == 0

    def add_message(self, msg):
        self.buffer.append((self.time_func(), msg))

    def get_message(self):
        current_time = self.time_func()
        target_time = current_time - self.max_age

        last_valid_message = None
        for timestamp, message in self.buffer:
            if timestamp < target_time:
                last_valid_message = message
            else:
                break

        while not self.is_empty and self.buffer[0][0] < target_time:
            self.buffer.popleft()

        return last_valid_message
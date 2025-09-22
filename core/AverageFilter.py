# core/AverageFilter.py

from collections import deque


class AverageFilter:
    def __init__(self, name="AverageFilter", max_age=0.15, time_func=None):
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

    def add_value(self, value):
        self.buffer.append((self.time_func(), value))

    def get_average(self):
        if self.is_empty:
            return None

        current_time = self.time_func()
        target_time = current_time - self.max_age

        # Remove expired values
        while not self.is_empty and self.buffer[0][0] < target_time:
            self.buffer.popleft()

        if self.is_empty:
            return None

        # Calculate average of remaining values
        sum_values = sum(value for timestamp, value in self.buffer)
        count = len(self.buffer)
        
        return sum_values / count if count > 0 else None
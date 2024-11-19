# examples/python38/example_time_buffer_simulated_time_with_class.py

from core.TimeBuffer import TimeBuffer


class TimeBufferManager:
    def __init__(self, max_age=0.15, start_time=0.0):
        self.time = start_time
        self.time_buffer = TimeBuffer(max_age=max_age, time_func=lambda: self.time)

    def advance_time(self, delta):
        self.time += delta

    def run(self):
        self.time_buffer.add_message("Message 1")
        self.advance_time(0.05)
        self.time_buffer.add_message("Message 2")
        self.advance_time(0.05)
        self.time_buffer.add_message("Message 3")

        print("Last valid message before time limit:", self.time_buffer.get_message())

        self.advance_time(0.1)
        print("Last valid message after time limit:", self.time_buffer.get_message())


if __name__ == "__main__":
    manager = TimeBufferManager()
    manager.run()

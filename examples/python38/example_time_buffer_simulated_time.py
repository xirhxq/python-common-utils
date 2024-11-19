# examples/python38/example_time_buffer_simulated_time.py

from core.TimeBuffer import TimeBuffer


class SimulatedTime:
    def __init__(self, start_time=0.0):
        self.time = start_time

    def get_time(self):
        return self.time

    def advance_time(self, delta):
        self.time += delta


def example_timebuffer_simulated_time():
    simulated_time = SimulatedTime()
    time_buffer = TimeBuffer(max_age=0.15, time_func=simulated_time.get_time)

    time_buffer.add_message("Message 1")
    simulated_time.advance_time(0.05)
    time_buffer.add_message("Message 2")
    simulated_time.advance_time(0.05)
    time_buffer.add_message("Message 3")

    print("Last valid message before time limit:", time_buffer.get_message())

    simulated_time.advance_time(0.1)
    print("Last valid message after time limit:", time_buffer.get_message())


if __name__ == "__main__":
    example_timebuffer_simulated_time()

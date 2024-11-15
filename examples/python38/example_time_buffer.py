# examples/python38/example_time_buffer.py

import time
from core.TimeBuffer import TimeBuffer


def example_timebuffer():
    time_buffer = TimeBuffer(max_age=0.15, time_func=time.time)

    time_buffer.add_message("Message 1")
    time.sleep(0.05)
    time_buffer.add_message("Message 2")
    time.sleep(0.05)
    time_buffer.add_message("Message 3")

    print("Last valid message before time limit:", time_buffer.get_message())

    time.sleep(0.1)
    print("Last valid message after time limit:", time_buffer.get_message())


if __name__ == "__main__":
    example_timebuffer()

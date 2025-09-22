# examples/python38/example_average_filter_system_time.py

import time
from core.AverageFilter import AverageFilter


def example_average_filter_system_time():
    avg_filter = AverageFilter(name="SystemTimeFilter", max_age=0.15, time_func=time.time)

    # Add some values with real time delays
    values = [10, 20, 30, 40, 50]
    
    for value in values:
        time.sleep(0.05)  # Sleep for 50ms
        avg_filter.add_value(value)
        avg = avg_filter.get_average()
        print(f"Added value: {value}, Current average: {avg}")

    print(f"Final average: {avg_filter.get_average()}")


if __name__ == "__main__":
    example_average_filter_system_time()
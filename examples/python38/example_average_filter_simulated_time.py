# examples/python38/example_average_filter_simulated_time.py

from core.AverageFilter import AverageFilter


class SimulatedTime:
    def __init__(self, start_time=0.0):
        self.time = start_time

    def get_time(self):
        return self.time

    def advance_time(self, delta):
        self.time += delta


def example_average_filter_simulated_time():
    simulated_time = SimulatedTime()
    avg_filter = AverageFilter(
        name="TestFilter", max_age=0.15, time_func=simulated_time.get_time
    )

    # Add some values
    values = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
    
    for i, value in enumerate(values):
        simulated_time.advance_time(0.05)  # Advance by 50ms each time
        avg_filter.add_value(value)
        avg = avg_filter.get_average()
        print(
            f"Added value: {value}, Current average: {avg}, Time: {simulated_time.get_time()}"
        )

    print(f"Final average: {avg_filter.get_average()}")


if __name__ == "__main__":
    example_average_filter_simulated_time()
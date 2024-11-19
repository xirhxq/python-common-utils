# examples/python38/example_pid_simulated_time.py

from core.PID import PID


class SimulatedTime:
    def __init__(self, start_time=0.0):
        self.time = start_time

    def get_time(self):
        return self.time

    def advance_time(self, delta):
        self.time += delta


def example_pid_simulated_time():
    simulated_time = SimulatedTime()
    pid = PID(kp=1.0, ki=0.1, kd=0.05, timeFunc=simulated_time.get_time, intMax=10.0, intMin=-10.0)

    error_values = [0.5, 1.0, 1.5, 1.0, 0.5, 0.0, -0.5, -1.0, -1.5, -1.0]

    pid_outputs = []

    for error in error_values:
        simulated_time.advance_time(0.1)
        output = pid.compute(error)
        pid_outputs.append(output)
        print(f"Error: {error}, PID Output: {output}, Time: {simulated_time.get_time()}")

    pid.clearIntResult()
    print(f"Integral result after clearing: {pid._PID__errIntegral}")


if __name__ == "__main__":
    example_pid_simulated_time()
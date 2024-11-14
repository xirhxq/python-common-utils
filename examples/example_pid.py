# examples/example_pid.py

import time
from core.PID import PID


def example_pid():
    pid = PID(kp=1.0, ki=0.1, kd=0.05, timeFunc=time.time, intMax=10.0, intMin=-10.0)

    error_values = [0.5, 1.0, 1.5, 1.0, 0.5, 0.0, -0.5, -1.0, -1.5, -1.0]

    pid_outputs = []

    for error in error_values:
        output = pid.compute(error)
        pid_outputs.append(output)
        print(f"Error: {error}, PID Output: {output}")

    pid.clearIntResult()
    print(f"Integral result after clearing: {pid._PID__errIntegral}")


if __name__ == "__main__":
    example_pid()

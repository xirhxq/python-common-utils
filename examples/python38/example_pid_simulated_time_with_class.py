# examples/python38/example_pid_simulated_time_with_class.py

from core.PID import PID


class PIDController:
    def __init__(
        self, kp=1.0, ki=0.1, kd=0.05, int_max=10.0, int_min=-10.0, start_time=0.0
    ):
        self.time = start_time
        self.pid = PID(
            kp=kp,
            ki=ki,
            kd=kd,
            timeFunc=lambda: self.time,
            intMax=int_max,
            intMin=int_min,
        )

    def advance_time(self, delta):
        self.time += delta

    def run(self):
        error_values = [0.5, 1.0, 1.5, 1.0, 0.5, 0.0, -0.5, -1.0, -1.5, -1.0]
        pid_outputs = []

        for error in error_values:
            self.advance_time(0.1)
            output = self.pid.compute(error)
            pid_outputs.append(output)
            print(f"Error: {error}, PID Output: {output}, Time: {self.time}")

        self.pid.clearIntResult()
        print(f"Integral result after clearing: {self.pid._PID__errIntegral}")


if __name__ == "__main__":
    controller = PIDController()
    controller.run()

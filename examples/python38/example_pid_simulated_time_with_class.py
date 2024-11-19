# examples/python38/example_pid_simulated_time_with_class.py

from core.PID import PID

class Controller:
    def __init__(self, kp, ki, kd, intMax=None, intMin=None, start_time=0.0):
        self.time = start_time
        self.pid = PID(kp=kp, ki=ki, kd=kd, timeFunc=self.get_time, intMax=intMax, intMin=intMin)

    def get_time(self):
        return self.time

    def update_time(self, dt):
        self.time += dt

    def compute_pid(self, error):
        return self.pid.compute(error)

    def reset_pid(self):
        self.pid.clearIntResult()


def example_pid_simulated_time_with_class():
    controller = Controller(kp=1.0, ki=0.1, kd=0.05, intMax=10.0, intMin=-10.0, start_time=0.0)

    error_values = [0.5, 1.0, 1.5, 1.0, 0.5, 0.0, -0.5, -1.0, -1.5, -1.0]

    pid_outputs = []

    for i, error in enumerate(error_values):
        controller.update_time(0.1)
        output = controller.compute_pid(error)
        pid_outputs.append(output)
        print(f"Error: {error}, PID Output: {output}, Time: {controller.get_time()}")

    controller.reset_pid()
    print(f"Integral result after clearing: {controller.pid._PID__errIntegral}")


if __name__ == "__main__":
    example_pid_simulated_time_with_class()
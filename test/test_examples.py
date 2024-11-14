# test/test_examples.py

import unittest
import subprocess
import sys
import os
import time

EXAMPLES = [
    "examples/example_pid.py",
    "examples/example_pid_ros.py",
    "examples/example_timebuffer.py",
    "examples/example_timebuffer_ros.py",
]


class TestExamples(unittest.TestCase):

    def run_example(self, example_path, start_roscore=False):
        """Runs a given example file and checks for errors."""
        try:
            if start_roscore:
                roscore_process = subprocess.Popen(
                    ["roscore"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                )
                time.sleep(5)

            result = subprocess.run(
                [sys.executable, "-m", example_path], capture_output=True, text=True
            )
            result.check_returncode()

            if start_roscore:
                roscore_process.terminate()
                roscore_process.wait()

            return True
        except subprocess.CalledProcessError as e:
            print(f"Error running {example_path}: {e.stderr}")
            return False
        except Exception as e:
            print(f"Unexpected error running {example_path}: {str(e)}")
            return False

    def test_example_pid(self):
        self.assertTrue(self.run_example("examples.example_pid"))

    def test_example_pid_ros(self):
        self.assertTrue(
            self.run_example("examples.example_pid_ros", start_roscore=True)
        )

    def test_example_timebuffer(self):
        self.assertTrue(self.run_example("examples.example_time_buffer"))

    def test_example_timebuffer_ros(self):
        self.assertTrue(
            self.run_example("examples.example_time_buffer_ros", start_roscore=True)
        )


if __name__ == "__main__":
    unittest.main()

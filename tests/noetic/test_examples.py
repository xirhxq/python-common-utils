# tests/noetic/test_examples.py

import unittest
import subprocess
import os
import glob
import sys
import time


EXAMPLES_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../../examples/noetic")
)
EXAMPLES = glob.glob(os.path.join(EXAMPLES_DIR, "example_*.py"))


class TestExamplesNoetic(unittest.TestCase):

    def run_example(self, example_module, start_roscore=False):
        """Runs a given example module and checks for errors."""
        try:
            if start_roscore:
                roscore_process = subprocess.Popen(
                    ["roscore"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                )
                time.sleep(5)

            result = subprocess.run(
                [sys.executable, "-m", example_module], capture_output=True, text=True
            )
            result.check_returncode()

            if start_roscore:
                roscore_process.terminate()
                roscore_process.wait()

            return True
        except subprocess.CalledProcessError as e:
            print(f"Error running {example_module}: {e.stderr}")
            return False

    def test_examples(self):
        for example_path in EXAMPLES:
            example_module = example_path_to_module(example_path, "examples.noetic")
            with self.subTest(example=example_module):
                self.assertTrue(self.run_example(example_module, start_roscore=True))


def example_path_to_module(example_path, base_package):
    relative_path = os.path.relpath(example_path, os.path.dirname(EXAMPLES_DIR))
    module_path = os.path.splitext(relative_path)[0].replace(os.path.sep, ".")
    return f"{base_package}.{module_path.split('.', 1)[-1]}"


if __name__ == "__main__":
    unittest.main()

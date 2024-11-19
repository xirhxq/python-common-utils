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


def generate_test(example_path):
    def test(self):
        example_module = example_path_to_module(example_path, "examples.noetic")
        try:
            roscore_process = subprocess.Popen(
                ["roscore"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            time.sleep(5)

            result = subprocess.run(
                [sys.executable, "-m", example_module], capture_output=True, text=True
            )
            result.check_returncode()

            roscore_process.terminate()
            roscore_process.wait()

        except subprocess.CalledProcessError as e:
            self.fail(f"Error running {example_module}: {e.stderr}")

    return test


def example_path_to_module(example_path, base_package):
    relative_path = os.path.relpath(example_path, os.path.dirname(EXAMPLES_DIR))
    module_path = os.path.splitext(relative_path)[0].replace(os.path.sep, ".")
    return f"{base_package}.{module_path.split('.', 1)[-1]}"


class TestExamplesNoetic(unittest.TestCase):
    pass


for example_path in EXAMPLES:
    example_name = os.path.splitext(os.path.basename(example_path))[0]
    test_name = f"test_{example_name}"
    test_case = generate_test(example_path)
    setattr(TestExamplesNoetic, test_name, test_case)


if __name__ == "__main__":
    unittest.main()

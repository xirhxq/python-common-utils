# tests/python38/test_examples.py

import unittest
import subprocess
import os
import glob
import sys


EXAMPLES_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../../examples/python38")
)
EXAMPLES = glob.glob(os.path.join(EXAMPLES_DIR, "example_*.py"))


class TestExamplesPython(unittest.TestCase):

    def run_example(self, example_module):
        """Runs a given example module and checks for errors."""
        try:
            result = subprocess.run(
                [sys.executable, "-m", example_module], capture_output=True, text=True
            )
            result.check_returncode()
            return True
        except subprocess.CalledProcessError as e:
            print(f"Error running {example_module}: {e.stderr}")
            return False

    def test_examples(self):
        for example_path in EXAMPLES:
            example_module = example_path_to_module(example_path, "examples.python38")
            with self.subTest(example=example_module):
                self.assertTrue(self.run_example(example_module))


def example_path_to_module(example_path, base_package):
    relative_path = os.path.relpath(example_path, os.path.dirname(EXAMPLES_DIR))
    module_path = os.path.splitext(relative_path)[0].replace(os.path.sep, ".")
    return f"{base_package}.{module_path.split('.', 1)[-1]}"


if __name__ == "__main__":
    unittest.main()

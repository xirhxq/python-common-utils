# python-common-utils

`python-common-utils` is a utility library designed to support various engineering and experimental tasks. Each functionality is implemented as a separate, object-oriented module, making it easy to integrate into different projects. This library is **compatible with Python (>=3.8)**, **ROS (Noetic)**, and **ROS 2 (Foxy)** environments, allowing it to be used flexibly across different platforms.

## Features

This library offers the following functionalities:

- **Simple PID Controller Implementation**: Provides a **PID** controller for system control and feedback loops, suitable for various control engineering applications.
- **Time-based Data Buffering**: Implements the **TimeBuffer** for storing and retrieving messages based on time constraints, ensuring efficient message handling in time-sensitive systems.

## Directory Structure

- `core/`: Contains the main source code, including the PID controller and TimeBuffer utilities.
- `examples/`: Provides example usage scripts to demonstrate how to use the modules in `core/`. These scripts are designed to work in Python, ROS, and ROS 2 environments.
- `tests/`: Contains unit tests to verify the correctness of the modules in `core/`. The tests ensure expected behavior across supported platforms.
- `docker/`: Includes Docker-related scripts for building and running tests in isolated environments.
- `README.md`: Documentation that outlines the library's purpose, structure, and functionality.

## Usage

The primary purpose of this library is to provide utility modules that can be easily integrated into different projects. You can find example usage scripts in the `examples/` directory, which demonstrate how to use the `core/` modules in different environments.

### Environment Compatibility

`python-common-utils` is designed to work seamlessly in the following environments:
- **Python**: Compatible with Python 3.8 and above.
- **ROS**: Tested with ROS Noetic.
- **ROS 2**: Tested with ROS 2 Foxy.

This library is **ROS-agnostic**, meaning you can use it in ROS or ROS 2 environments, or in standalone Python projects without any ROS dependencies.

## Examples

### PID Controller Example

In a ROS 2 environment, you can run the `examples/foxy/example_pid_ros2.py` script to see the PID controller in action:

```bash
python3 -m examples.foxy.example_pid_ros2
```

This example demonstrates how to set up and use the PID controller for system control.

### TimeBuffer Example

In a ROS 2 environment, you can run the `examples/foxy/example_time_buffer_ros2.py` script:

```bash
python3 -m examples.foxy.example_time_buffer_ros2
```

This script demonstrates how to use the `TimeBuffer` to manage time-sensitive messages.

For detailed examples and usage, refer to the `examples/` directory.

## Testing

To run tests for the library, including example scripts, follow these steps:

1. Ensure the Docker scripts are executable:
   ```bash
   chmod +x docker/build_docker.sh docker/run_docker.sh
   ```

2. Build the Docker image:
   ```bash
   ./docker/build_docker.sh
   ```

3. Run tests inside a Docker container:
   ```bash
   ./docker/run_docker.sh
   ```

The `run_docker.sh` script will execute all unit tests and example scripts in the `examples/` directory.

## Contributing

Contributions are welcome! Please ensure that your code passes all tests before submitting a pull request (PR). If you would like to propose new features or improvements, feel free to create an issue.

## License

`python-common-utils` is licensed under the [GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.html). You are free to use, modify, and distribute this software, provided you comply with the terms of the license.
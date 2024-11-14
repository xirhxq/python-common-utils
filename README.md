# python-common-utils

`python-common-utils` is a utility library designed to provide common Python functionalities to help developers with general-purpose programming tasks. It includes a variety of modules, such as control algorithms and data buffering utilities, to assist with common programming needs.

## Features

This library offers the following functionalities:

- **Simple PID Controller Implementation**: Includes the **PID** controller for system control and feedback loops.
- **Time-based Data Buffering**: Provides the **TimeBuffer** for storing and retrieving messages based on time constraints.

## Directory Structure

- `core/`: Contains the main source code, including the PID controller and TimeBuffer utilities.
- `examples/`: Provides example usage scripts to demonstrate how to use the modules in `core/`. These serve as references to understand the intended functionality.
- `tests/`: Contains unit tests to verify the correctness of the modules in `core/`. The tests ensure expected behavior.
- `docker/`: Includes Docker-related scripts for building and running tests in an isolated environment.
- `README.md`: Documentation that outlines the library's purpose, structure, and functionality.

## Usage

The primary purpose of this library is to provide examples in the `examples/` directory, which demonstrate how to use the modules in `core/`. You can refer to these examples as guides for implementing the functionality in your own projects.

## Testing

To set up and run tests, including example files, within a Docker container:

1. Ensure the scripts are executable:
   ```bash
   chmod +x docker/build_docker.sh docker/run_docker.sh
   ```

2. Build the Docker image:
   ```bash
   ./docker/build_docker.sh
   ```

3. Run tests within the Docker container:
   ```bash
   ./docker/run_docker.sh
   ```

The `run_docker.sh` script will execute all unit tests, including examples from the `examples/` directory.

## Contributing

Contributions are welcome! Please ensure that your code passes all tests before submitting a pull request (PR).

## License

`python-common-utils` is licensed under the [GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.html). You are free to use, modify, and distribute this software, but you must comply with the terms of the GNU v3 license.

# python-common-utils

`python-common-utils` is a utility library for common Python functionalities, including control algorithms (such as a PID controller). This library aims to assist developers in efficiently handling general-purpose programming tasks.

## Directory Structure

```
python-common-utils/
├── core/                 # Source code directory
│   └── PID.py            # PID controller implementation
├── test/                 # Test files directory
│   └── test_pid.py       # Unit tests for the PID controller
├── docker/               # Docker-related scripts
│   ├── Dockerfile        # Dockerfile for containerized testing
│   ├── build_docker.sh   # Script to build Docker container
│   └── run_docker.sh     # Script to run tests in Docker container
└── README.md             # Library documentation
```

## Usage

Below is an example of how to use the PID controller module in `core/`.

### Using the PID Controller

```python
import time
from core.PID import PID

pid = PID(kp=1.0, ki=0.1, kd=0.05, timeFunc=time.time)
output = pid.compute(1.5)
print("PID output:", output)
```

## Testing

To set up and run the tests in a Docker container:

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

The `run_docker.sh` script will automatically execute all unit tests within the container.

## Contributing

Contributions are welcome! Please ensure your code passes all tests before submitting a pull request (PR).

## License

`python-common-utils` is licensed under the [GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.html). You are free to use, modify, and distribute this software, but you must comply with the terms of the GNU v3 license.

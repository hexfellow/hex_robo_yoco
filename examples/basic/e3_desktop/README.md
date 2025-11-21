# Mujoco E3 Desktop Example

## Description

This example shows how to use Mujoco simulation for E3 Desktop robot. It simulates the E3 Desktop robot in Mujoco physics engine and provides position, velocity, torque control, as well as RGB and depth image feedback.

## Structure

```bash
mujoco_e3_desktop/
├── cli.py     # client code (working code)
├── cli.json   # client configuration
├── launch.py  # launch script
└── README.md  # this file
```

## Dependencies

### Hardware

None.

### Environment

None.

## Usage

- Assuming you have installed the library from source code, and your `working directory` is `hex_zmq_servers/examples/basic/mujoco_e3_desktop`, you can run the example by:

    ```bash
    source ../../../.venv/bin/activate
    python3 launch.py
    ```

- The Mujoco visualization window will open automatically. Press 'q' in the image window to quit.


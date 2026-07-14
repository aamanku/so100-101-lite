# so100-101-lite

A small, Torch-free SO100/SO101 follower-arm controller for Linux. It includes direct keyboard examples, a safety-limited ZMQ robot server, and a telemetry GUI.

This is an independent, unofficial project built from selected hardware code from [Hugging Face LeRobot](https://github.com/huggingface/lerobot) and control examples from [XLeRobot](https://github.com/Vector-Wangel/XLeRobot). It does **not** install the full LeRobot package, PyTorch, TorchVision, or WandB.

## Features

- Direct keyboard joint and end-effector control.
- ZMQ position-command server with:
  - calibrated position limits and endpoint margin;
  - per-joint velocity and acceleration limiting;
  - smooth trajectory generation;
  - command watchdog and hold behavior;
  - latched over-temperature, over-current, and serial-error faults;
  - emergency torque disable;
  - gripper torque and commands disabled by default.
- Tk GUIs with:
  - desired-position sliders;
  - a single-slider heading client with piecewise-linear keypose interpolation;
  - live measured-position markers on every slider;
  - damping mode that sends measured positions back as desired positions;
  - position, target, smoothed goal, speed, load, voltage, temperature, current, movement, and status telemetry;
  - hold, fault reset, and emergency-stop controls.
- Runs without Torch or the full LeRobot distribution.

> [!WARNING]
> This software controls physical hardware and is not safety-certified. Keep people and objects outside the arm workspace, begin with the arm unloaded, use conservative limits, and remain ready to disconnect power. You are responsible for validating all limits and current conversions for your hardware.

## Requirements

- Linux
- Python 3.12 (Miniforge recommended)
- SO100 or SO101 follower arm using Feetech STS3215 motors
- USB serial device such as `/dev/ttyACM0`
- Desktop session for the GUI; X11 is recommended for the keyboard examples

## Installation

Clone the repository and enter its root:

```bash
git clone https://github.com/aamanku/so100-101-lite.git
cd so100-101-lite
```

Create the Python 3.12 environment from the supplied dependency file:

```bash
conda env create -f environment.yml
conda activate so100_101_lite
```

Equivalent manual installation:

```bash
conda create -n so100_101_lite -c conda-forge python=3.12 pip tk -y
conda activate so100_101_lite
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Confirm that the local lightweight `lerobot/` copy is loaded and Torch is absent:

```bash
python -c "import importlib.util, lerobot; print(lerobot.__file__); assert importlib.util.find_spec('torch') is None"
```

Do not install the full `lerobot` PyPI distribution in this environment. Run the programs from the repository root so Python imports the included `lerobot/` directory.

## USB serial permissions

Find the controller port:

```bash
ls -l /dev/ttyACM*
```

If opening the port reports `Permission denied`, this temporary permission change may be required:

```bash
sudo chmod 666 /dev/ttyACM0
```

Replace `/dev/ttyACM0` with the actual device name. This grants every local user read/write access and normally resets after reconnecting the device or rebooting.

A more persistent and restrictive solution is to add your user to the serial-port group, then log out and back in:

```bash
sudo usermod -aG dialout "$USER"
```

Do not run the controller itself with `sudo`.

## ZMQ server and GUI

### 1. Start the robot server

```bash
conda activate so100_101_lite
python 2_s100_keyboard_joint_control_zmq.py --port /dev/ttyACM0
```

The server listens on `tcp://127.0.0.1:5555`. It owns the serial port and performs all command validation and safety limiting. It sends no movement goal until the first valid ZMQ target arrives.

The gripper is disabled by default: its torque is turned off and gripper targets are ignored. Enable it explicitly when required:

```bash
python 2_s100_keyboard_joint_control_zmq.py \
  --port /dev/ttyACM0 \
  --enable-gripper
```

Run this to inspect all configurable thresholds:

```bash
python 2_s100_keyboard_joint_control_zmq.py --help
```

#### Tune the motion constants

Before using a different payload, supply voltage, arm revision, or mounting orientation, review the user-tunable constants near the top of [`2_s100_keyboard_joint_control_zmq.py`](2_s100_keyboard_joint_control_zmq.py):

- `DEFAULT_MAX_SPEED` — maximum trajectory speed for each joint in degrees/second; gripper uses percent/second.
- `DEFAULT_MAX_ACCELERATION` — maximum trajectory acceleration for each joint in degrees/second²; gripper uses percent/second².
- `CURRENT_AMPS_PER_RAW_UNIT` — telemetry conversion only; change it only after checking the motor and firmware documentation.

Start with values lower than the defaults and test every joint unloaded. Temperature limits, current fault threshold, watchdog timeout, position-limit margin, and loop rates are command-line options rather than global constants; inspect them with `--help`.

### 2. Start the GUI

In another terminal:

```bash
conda activate so100_101_lite
python 21_gui.py
```

The GUI initially observes the robot without sending movement commands.

- **Enable Command Streaming:** sends slider positions.
- **Damping Mode:** sends each active joint's latest measured position as its desired position.
- **Hold Current Position:** asks the server to stop the smoothed trajectory at its current commanded goal.
- **Stop Sending:** exits slider or damping mode and invokes hold.
- **Reset Fault / Enable Torque:** clears a latched fault only if temperature and current checks pass.
- **EMERGENCY STOP:** disables torque and latches a server fault.
- **Faint slider marker:** shows the measured joint position; the slider thumb is the desired position.

Closing the GUI does not stop the server. If GUI commands disappear, the server watchdog transitions to hold.

### Heading keypose client

[`3_heading_zmq.py`](3_heading_zmq.py) is a smaller client with no telemetry table and one **shoulder pan** slider. The slider directly sets only the desired pan target. Targets for the remaining joints come from the user-editable `KEYPOSES` list near the top of the file and are interpolated from the server's latest measured pan position.

Before running it, replace the example poses with positions validated on your arm. Every keypose must contain the same joints, and `shoulder_pan` values must be strictly increasing:

- the first keypose defines the minimum pan angle;
- the last keypose defines the maximum pan angle;
- with two keyposes, non-pan joint targets are interpolated between those endpoints as measured pan progresses;
- with three or more keyposes, interpolation is piecewise between the two adjacent keyposes around the measured pan angle;
- the desired slider value remains the pan target and is not used to advance the other joints;
- if measured pan stops moving, the interpolated targets for the other joints stop advancing;
- omitted joints retain their existing server targets.

The supplied examples command every non-gripper arm joint. Review and validate them before enabling commands. Add `gripper` to every keypose only if the server is deliberately running with `--enable-gripper`.

Start the robot server first, then run:

```bash
python 3_heading_zmq.py
```

The client observes the server without moving the arm until **Enable Pose Streaming** is clicked. It intersects the keypose pan range with the server's calibrated pan limits. **Stop / Hold**, connection loss, command rejection, closing the window, and the server watchdog stop the command stream; **EMERGENCY STOP** disables torque and latches a fault.

## Keyboard examples

Only run one process that owns the serial port at a time.

```bash
# Direct joint control
python 0_so100_keyboard_joint_control.py

# Analytical end-effector control
python 1_so100_keyboard_ee_control.py
```

The keyboard examples print their key mappings after connecting. They move the arm toward zero during startup, unlike the ZMQ server, so keep the workspace clear before launching them.

## Calibration

LeRobot-compatible calibration data is loaded automatically. If no calibration is found, the robot server may prompt for calibration during connection. Follow the prompts carefully and support the arm as required.

Use a consistent `--robot-id` if you maintain multiple calibration files:

```bash
python 2_s100_keyboard_joint_control_zmq.py \
  --port /dev/ttyACM0 \
  --robot-id my_so101
```

## ZMQ API

The server uses a JSON ZMQ `REP` socket. Clients must send a request and receive its response before sending the next request.

A target request may include all active joints or a subset:

```json
{
  "type": "set_targets",
  "positions": {
    "shoulder_pan": 10.0,
    "shoulder_lift": -15.0,
    "elbow_flex": 20.0,
    "wrist_flex": 0.0,
    "wrist_roll": 30.0
  }
}
```

Supported request types:

| Type | Purpose |
|---|---|
| `get_state` | Read telemetry and controller state without refreshing the command watchdog. |
| `set_targets` | Validate and update desired positions; refreshes the watchdog. |
| `hold` | Stop the trajectory at the latest smoothed goal. |
| `estop` | Disable torque and latch a fault. |
| `reset_fault` | Re-check telemetry and re-enable torque for active joints. |

Every response contains `ok` and a `state` object. Failed requests also contain `error`. The state includes measured positions, raw velocity/load/status, voltage, temperature, estimated current, requested targets, smoothed goals, trajectory velocity, calibrated limits, watchdog state, warnings, and fault state.

Clients should resend targets faster than the configured watchdog timeout. The supplied GUI does this automatically.

## Network use

The default localhost binding prevents other machines from commanding the arm. For remote use on a trusted network:

```bash
# Robot machine
python 2_s100_keyboard_joint_control_zmq.py \
  --port /dev/ttyACM0 \
  --bind tcp://0.0.0.0:5555

# GUI machine
python 21_gui.py --connect tcp://ROBOT_IP:5555
```

ZMQ authentication and encryption are not configured. Do not expose the server to an untrusted network or the public internet.

## Telemetry notes

- Position units are degrees, except gripper position, which is percent.
- Voltage is converted from the Feetech raw register in 0.1 V units.
- Current uses the STS3215 nominal scale of 6.5 mA per raw unit. Treat the displayed amperage as an estimate and verify it against your motor model and firmware.
- Temperature is read in degrees Celsius.
- Velocity, load, movement, and status retain their Feetech register representation where indicated as raw.

## Repository layout

```text
.
├── 0_so100_keyboard_joint_control.py
├── 1_so100_keyboard_ee_control.py
├── 2_s100_keyboard_joint_control_zmq.py   # robot-side ZMQ server
├── 3_heading_zmq.py                       # one-slider keypose client
├── 21_gui.py                              # full telemetry GUI client
├── .python-version                        # Python 3.12 for pyenv-compatible tools
├── lerobot/                               # copied minimal hardware subset
├── environment.yml                        # Miniforge/Python 3.12 environment
├── requirements.txt                       # pip runtime dependencies
├── NOTICE
└── LICENSE
```

## Attribution and license

Selected LeRobot modules and the original XLeRobot examples are distributed under the Apache License 2.0. See [`NOTICE`](NOTICE) for source attribution and [`LICENSE`](LICENSE) for the license text.

The included `lerobot/` directory is intentionally incomplete and only supports the functionality used by this repository. It is not a replacement for the full LeRobot project.

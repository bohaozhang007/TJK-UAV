# TJK-UAV

[中文说明](READMECN.md)

TJK-UAV is a single-target perception and flight-control framework for drones. It searches for a specified object, selects a suitable view, tracks and approaches the object, aligns it near the image center, captures a front view, and flies around it to capture additional views.

The project separates drone-specific control from Agent logic:

- Robot Server runs beside the drone SDK, simulator, or ROS system.
- Robot Client connects the Agent to the selected Robot Server.
- Agent performs search, detection, view selection, tracking, approach, centering, and multi-view capture.

Currently registered Robot backends are `owl`, `tello`, and `ue`.

## Quick start

Start the Robot Server first. Start the Agent only after the server is reachable and the drone is ready.

### 1. Robot side

For the OWL drone, run this from the repository on the drone computer:

```bash
chmod +x run_owl_console.sh
./run_owl_console.sh
```

The script loads ROS, starts the OWL Robot Server on port `8765`, and opens the command-line console. The usual manual workflow is:

```text
init
takeoff
health
```

The Agent detects an already airborne drone and does not repeat takeoff.

To start a backend directly, add `src` to `PYTHONPATH` and select it with `--robot`:

```bash
export PYTHONPATH="$PWD/src"
python3 -m robot.server --robot owl --host 0.0.0.0 --port 8765
```

Switch the backend by replacing `owl` with `tello` or `ue`:

```bash
python3 -m robot.server --robot tello --host 0.0.0.0 --port 8765
python3 -m robot.server --robot ue --host 0.0.0.0 --port 8765
```

### 2. Agent side

On Windows, start Agent v17 with `run_agent.bat`.

Text-prompt example:

```powershell
.\run_agent.bat --use_da3 --det yolo --text "bottle" --robot owl --server-host 192.168.2.20 --config owl\v17_2.yaml
```

SAM3 visual-exemplar example:

```powershell
.\run_agent.bat --use_da3 --det sam3 --img assets\bottle.jpg --box assets\bottle.txt --robot owl --server-host 192.168.2.20 --config owl\v17_2.yaml
```

`--box` points to a text file containing four space-separated `xyxy` pixel coordinates.

To switch robots, use the same backend on both sides:

```powershell
--robot owl
--robot tello
--robot ue
```

Use `--server-host 127.0.0.1` when Agent and Robot Server run on the same computer. Config paths are resolved relative to `src/agent/config`. Runtime logs and visualizations are written under the repository-level `logs/` directory and are ignored by Git.

> Real-drone tests should begin in a safe area with manual takeover available.

## Repository structure

```text
TJK-UAV/
├── assets/                  Reference images and exemplar boxes
├── logs/                    Runtime logs and visualizations (Git-ignored)
├── src/
│   ├── agent/
│   │   ├── config/          Versioned and robot-specific YAML configs
│   │   └── tjk/             Versioned Agent implementations
│   ├── robot/
│   │   ├── controllers/     Control logic, coordinate conversion, and safety
│   │   ├── hardware/        Raw SDK, ROS, or simulator communication
│   │   └── server.py        Unified HTTP Robot Server and console
│   ├── robot_client/        Agent-facing clients for each Robot backend
│   ├── third_party/         Detector, tracker, and depth adapters/services
│   └── utils.py
├── run_agent.bat            Windows Agent launcher
└── run_owl_console.sh       OWL Robot Server launcher
```

## Installation and customization with Codex

Hardware SDKs, ROS workspaces, models, and Agent dependencies differ between deployments. This repository therefore does not prescribe one universal dependency installation. Most adaptations can be completed by asking Codex to inspect the existing implementation and add only what the selected platform needs.

### 1. Customize a Robot

Give Codex the new robot's SDK or ROS documentation, available topics/services, coordinate conventions, camera interface, pose source, motion commands, and operating environment. Ask it to follow the existing `owl`, `tello`, and `ue` implementations and:

1. Add `src/robot/hardware/<robot>.py` for raw hardware, SDK, ROS, or simulator communication.
2. Add `src/robot/controllers/<robot>.py` for coordinate conversion, safety checks, blocking motion completion, and motion tolerances.
3. Add `src/robot_client/<robot>.py` for the common Agent-facing interface.
4. Register the new name in the Robot Server, Agent client builder, and `--robot` choices.
5. Verify image capture, pose, health, takeoff/landing, relative XYZ/yaw motion, and completion behavior before running an Agent mission.

Example Codex request:

```text
Read src/robot, src/robot_client, and the existing OWL adapter. Add a new
<robot-name> backend from the attached SDK/ROS documentation. Keep hardware
limited to raw communication, put control and safety logic in the controller,
implement the common client API, register --robot <robot-name>, and verify the
Robot Server endpoints without changing unrelated backends.
```

### 2. Customize an Agent

Tell Codex the mission behavior, selected robot, detector/tracker inputs, required outputs, and available runtime environment. It may design the Agent functions and choose/install the dependencies needed by that design. Ask it to:

1. Read the latest implementation under `src/agent/tjk/` and reuse the common Robot Client interface.
2. Create a new versioned Agent file instead of overwriting older experimental versions when appropriate.
3. Add a matching YAML file under `src/agent/config/<robot>/`.
4. Update the launcher or CLI only when the new Agent, model service, or input format requires it.
5. Keep logs and visual results under the repository-level `logs/` directory.

Example Codex request:

```text
Read the latest Agent and its YAML config. Create a new version for
<mission description> using robot <robot-name>. You may design the required
Agent stages and install the model/runtime dependencies you choose. Preserve
older Agent versions, add a matching robot-specific config, update the launcher
if needed, and verify the complete mission with mocked inputs before flight.
```

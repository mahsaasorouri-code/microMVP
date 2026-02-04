# MVP3 

A multi-agent path planning and coordination system with real-time vision processing and simulation capabilities.

## Project Overview

This project is developed and tested under **Python 3.12**. It provides a comprehensive system for multi-robot coordination, featuring both simulation and real-world modes with real-time camera vision processing.

It supports two execution modes:
- **Simulation Mode** - Virtual environment for testing algorithms without hardware
- **Real-World Mode** - Live camera input with actual robot coordination

---

## System Requirements

- **Python 3.12**
- **Camera** (for real-world mode): 720  1280 resolution
- **Operating System**: Windows

### Dependencies

All required packages are listed in 
equirements.txt. Key dependencies include:
- PyQt6 - GUI framework
- OpenCV - Vision processing
- ZMQ - Network communication
# MVP3 Revamp

MVP3 Revamp is a prototype system for multi-robot path planning, vision-based localization, and coordinated control. It supports both simulation and real-hardware modes and is intended for algorithm validation and small-robot integration.

Key features
- Multi-robot path planning and collision avoidance (MRPP, RVO2, etc.)
- Real-time ArUco-based localization from a camera, published over ZMQ
- PyQt6 GUI with simulation and real-hardware modes
- Serial communication and robot command framing

Quick Start
1. Create and activate a Python environment (recommended with conda):

```bash
conda create -n MVP3_1 python=3.12 -y
conda activate MVP3_1
pip install -r requirements.txt
```

2. Run the GUI in simulation mode (no hardware required):

```bash
python qt_gui.py -s
```

3. Run the GUI in real-hardware mode (camera and robots required):

```bash
python qt_gui.py
```
Real-hardware setup (XIAO ESP32C6)

For real-world deployments we use Seeed XIAO ESP32C6 boards. One XIAO should act as the central signal sender and be connected directly to the PC via USB; flash that board with xiao_ap_ESP_NOW.ino from the firmware/ folder. The remaining XIAO boards are installed on the robots — flash each robot's XIAO with xiao_1_8_ESP_NOW.ino. When flashing multiple robots, ensure you modify the CAR_ID in the xiao_1_8_ESP_NOW.ino file to assign a unique ID to each individual robot. Adjust pin mappings in the sketches to match your hardware wiring. The system has been tested with around 7 robots operating concurrently. After the XIAO boards are flashed and the robots are powered, connect the camera and run the GUI (python qt_gui.py) to start the real-hardware experiment.


Project layout (summary)
- `qt_gui.py`: main GUI application (simulation and hardware modes)
- `vision_pub.py`: camera capture and ArUco detection, publishes positions via ZMQ
- `positionZMQSub.py`: subscribes to vision data and maintains robot states
- `comm.py`: serial communication and command framing
- `DDR.py`: differential-drive control logic
- `algorithms/`: path planning and avoidance algorithms (`mrpp.py`, `rvo2.py`, ...)
- `patterns/`: predefined movement patterns and examples
- `firmware/`: microcontroller firmware examples
- `projects/`: example projects and research code

Dependencies
- See `requirements.txt` for the full list (OpenCV, PyQt6, pyzmq, numpy, pyserial, etc.).

Optional Dependency: Gurobi (MRPP only)

- Gurobi Optimizer **13.0.1**
- Required only for MRPP (ILP-based multi-robot path planning)
- Not required for vision, GUI control, or formation demos

Development and contribution
- To add a new algorithm, place the module under `algorithms/` and add a test entry in the GUI.
- Issues and PRs are welcome—please include reproduction steps and configuration details.

References
- Main GUI: [qt_gui.py](qt_gui.py)
- Vision publisher: [vision_pub.py](vision_pub.py)
- Communication: [comm.py](comm.py)


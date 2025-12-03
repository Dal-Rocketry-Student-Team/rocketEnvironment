# Rocket IMU Visualizer (PyQt6 + pyqtgraph)

A lightweight 3D visualization for real-time rocket attitude and motion using a 6-axis IMU on an STM32.
It listens to a serial port (USB CDC/Virtual COM), renders a simple rocket model, overlays a heads-up display (HUD), and (optionally) integrates accelerometer data to show translational drift.

# Quick Start
### 1) clone and enter
git clone https://github.com/Dal-Rocketry-Student-Team/rocketEnvironment.git

### 2) create and activate a virtual environment

#### Windows (PowerShell)
python -m venv .venv
.venv\Scripts\Activate.ps1

#### macOS / Linux
python3 -m venv .venv
source .venv/bin/activate

### 3) install dependencies
pip install -r requirements.txt

### 4) connect your board (note the serial port: e.g., COM7, /dev/ttyACM0, /dev/tty.usbmodem*)

### 5) run the viewer
python rocket_viz.py

### 6) Set your serial port & baud:

Open rocket_viz.py and find where the serial worker is created, e.g.:
worker = SerialWorker(port="COM7", baud=115200)
Change to your port:
Windows: COM3, COM7, …
Linux: /dev/ttyACM0 or /dev/ttyUSB0
macOS: /dev/tty.usbmodem* or /dev/tty.usbserial*

### (optional) 7) quaternion simulator:
This script simulates an IMU by generating synthetic quaternion, gyro, and
accelerometer data, and streams it over a serial port in the same format as the stm32.

A virtual com port connection NEEDS to be setup for this to be
[ quaternion_simulator.py ]  ->  COM[number1]  ===virtual cable===  COM[number2]  ->  [ your viewer ]

quaternion_simulator.py writes to COM[number1] and the rocket scene script reads from COM[number2]

This can be setup using a virtual com port tool like com0com, install through this video: https://www.youtube.com/watch?v=5RIcez2Vpdk

The script can be started with various arguments.
Examples:

python quaternion_simulator.py --port COM[number] --mode rot --rate-hz 100 --yaw-rate 30 --roll-rate 10

python quaternion_simulator.py --port COM[number] --mode trans --a-amp-g 0.05 --a-freq 0.5 --a-dir x

python quaternion_simulator.py --port COM[number] --mode both --yaw-rate 20 --roll-rate 10 --a-amp-g 0.03 --a-freq 0.7 --a-dir y

# What this is for

Visual check of IMU fusion (quaternion/Euler) and axis alignment.

Quick bring-up of STM32 + LSM6DSR (or similar) without heavy ground software.

Teaching/diagnostics: see how accelerometer, gyro, and gravity interact.

Demo-grade inertial translation (double integration) to illustrate drift.

Not a flight controller and not for mission-critical GNC. It’s a visualization and learning tool.

# Features

- 3D rocket (body, nose, nozzle, fins) with body-axes arrows.

- Live HUD with roll/pitch/yaw (deg), render FPS, packet age (ms), ω (dps), a (g).

- Serial reader runs on a Qt background thread (no frozen UI).

- Quaternion → rotation with incremental mesh rotation (no per-frame rebuilds).

- Optional translation: rotate accel to world, subtract gravity, integrate to v and p (with light damping for demo stability).

- Should work on Windows / macOS / Linux (untested).

- Data format (from the MCU)

- Send newline-terminated ASCII lines like:

- IMU,<tick_ms>, q0, q1, q2, q3, gx, gy, gz, ax, ay, az

- Added new quaternion simulator tool to debug the virtual environment

### Where:

- tick_ms = e.g., HAL_GetTick() (ms).

- q0..q3 = unit quaternion in [w, x, y, z] order.

- gx,gy,gz = gyro in deg/s.

- ax,ay,az = accel in g.

STM32 printf example:
printf("IMU,%lu,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f\r\n",
       HAL_GetTick(), q0, q1, q2, q3,
       gyro_dps[0], gyro_dps[1], gyro_dps[2],
       accel_g[0],  accel_g[1],  accel_g[2]);

# How it works (high level)

### Threads

- UI thread: PyQt6 event loop, 3D rendering (GLViewWidget), 60 Hz frame timer.

- Serial worker: QThread that blocks on readline() and emits signals:

- newQuat(w,x,y,z) → updates the current rotation matrix.

- newIMU(gx,gy,gz, ax,ay,az) (and dt) → updates HUD and optional translation.

### Orientation

- Quaternion → 3×3 rotation matrix.

- Frame update blends toward the target matrix (exponential smoothing), re-orthonormalizes (SVD), computes delta vs last drawn, and rotates all meshes by that delta (keeps your fixed translates for nose/fins intact).

### Translation (optional)

- Rotate accel (g) from body → world, convert to m/s², subtract gravity, integrate to velocity and position (with small damping). Demonstrates drift; not an INS.

# Configuration knobs (in code)

- Smoothing (display): self.smooth_alpha (0..1; higher = snappier).

- HUD font/position: self.hud.setStyleSheet(...), _place_hud().

- Axes: make_axes(L=...).

- Fins: make_fin_mesh(...) and _add_fins(n=..., span=..., sweep=..., thickness=...).

# Requirements

Python 3.9–3.13 (64-bit recommended)

pip, venv

GPU/driver that can handle OpenGL (Qt can fall back to software if needed)


## Rocket IMU Visualizer (PyQt6 + pyqtgraph)

A lightweight 3D visualization for real-time rocket attitude and motion using a 6-axis IMU on an STM32.
It listens to a serial port (USB CDC/Virtual COM), renders a simple rocket model, overlays a heads-up display (HUD), and (optionally) integrates accelerometer data to show translational drift.




What this is for

Visual check of IMU fusion (quaternion/Euler) and axis alignment.

Quick bring-up of STM32 + LSM6DSR (or similar) without heavy ground software.

Teaching/diagnostics: see how accelerometer, gyro, and gravity interact.

Demo-grade inertial translation (double integration) to illustrate drift.

Not a flight controller and not for mission-critical GNC. It’s a visualization and learning tool.

Features

3D rocket (body, nose, nozzle, fins) with body-axes arrows.

Live HUD with roll/pitch/yaw (deg), render FPS, packet age (ms), ω (dps), a (g).

Serial reader runs on a Qt background thread (no frozen UI).

Quaternion → rotation with incremental mesh rotation (no per-frame rebuilds).

Optional translation: rotate accel to world, subtract gravity, integrate to v and p (with light damping for demo stability).

Works on Windows / macOS / Linux.

Data format (from the MCU)

Send newline-terminated ASCII lines like:

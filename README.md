# EECS-473-MDE-Project
YOLOv5 person-following robot: Pi 5 + Teensy (RTOS), PID diff-drive, ESP32 wrist controller.

# Auto-Follow Luggage Robot (Raspberry Pi 5 + Teensy)

Real-time human detection/following at **~17 FPS** using YOLOv5 on Raspberry Pi 5, with a Teensy microcontroller
handling motor control via a PID differential drive. Teensy runs RTOS-style tasks (TeensyThreads) for sensor fusion,
control loops, and UART comms. An ESP32 wrist controller provides user input over Bluetooth.

## Features
- Person detection (YOLOv5), C++ inference on Pi 5.
- PID velocity + heading control on Teensy with wheel encoders and LiDAR range.
- RTOS task layout: `sensing`, `fusion`, `control`, `telemetry`, `ui`.
- Bluetooth/UART link: ESP32 → Teensy (start/stop, follow distance).
- Latency-aware command smoothing for stable following.

## Hardware (reference build)
- Raspberry Pi 5 + camera module
- Teensy 4.x, dual motor driver, wheel encoders
- 2D LiDAR / distance sensor (any UART/I2C unit)
- ESP32 wrist remote (buttons/rotary for setpoint)

## Repository Structure


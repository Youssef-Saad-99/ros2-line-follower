# 🤖 line_follower

<div align="center">

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badge&logo=ros)
![ATmega32](https://img.shields.io/badge/MCU-ATmega32-orange?style=for-the-badge)
![Python](https://img.shields.io/badge/Python-3.10+-green?style=for-the-badge&logo=python)
![C](https://img.shields.io/badge/Firmware-C%2FAVR-lightgrey?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-purple?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Active-brightgreen?style=for-the-badge)

**A fully autonomous differential-drive line-following robot powered by ROS2 Humble and ATmega32.**

*Real-time PID control · Bluetooth UART bridge · Encoder odometry · IR array sensing*

---

[Overview](#-overview) • [Team](#-team-vega) • [Architecture](#-system-architecture) • [Hardware](#-hardware-components) • [Setup](#-bluetooth-setup-linux) • [Run](#-running-the-project) • [Topics](#-ros2-topics) • [Troubleshooting](#-troubleshooting)

</div>

---

## 📋 Overview

The **line_follower** project implements a fully autonomous mobile robot capable of tracking a high-contrast black line on a white floor surface without human intervention. The system integrates an **ATmega32 microcontroller** running hard-real-time embedded firmware with a **ROS2 Humble** software stack running on a host PC, bridged over **Bluetooth UART** via an HC-05 module.

### 🎯 Project Objectives

- Implement a closed-loop PID line-following controller using a 5-element IR sensor array
- Establish reliable bidirectional UART communication between ATmega32 and ROS2
- Compute real-time wheel odometry using differential-drive kinematics and optical encoders
- Design a modular, node-based ROS2 architecture separating sensing, control, actuation, and localisation
- Achieve stable autonomous navigation including 90° sharp corner handling and line-loss recovery

### 🛠️ Technologies

| Category | Technology | Version / Detail |
|----------|-----------|-----------------|
| Robotics Middleware | ROS2 | Humble Hawksbill |
| Microcontroller | ATmega32 | 16 MHz, AVR-GCC |
| Communication | UART over Bluetooth | 9600 baud, 8N1 |
| Bluetooth Module | HC-05 | Slave mode, paired |
| Sensing | 5× IR Reflectance Array | ADC-based, threshold 600 |
| Localisation | Optical Wheel Encoders | 20 PPR per wheel |
| Control | PID Controller | Kp=0.5, Kd=0.05 |
| Drive System | Differential Drive | L298N H-Bridge |
| Language (ROS2) | Python | 3.10+ |
| Language (Firmware) | C | AVR-libc |

---

## 👥 Team VEGA

| # |       Name        |             Role           |           Email          |
|---|-------------------|----------------------------|--------------------------|
| 1 | Mostafa Abdelaziz | Embedded Engineer - Leader | mostafaafifi497@gmail.com   |
| 2 | Youssef Saad      | Embedded Engineer          | youssefsaad306@gmail.com |
| 3 | Omar Ahmed        | ROS2 Developer             | omarelnady144@gmail.com   |
| 4 | Mostafa Shabib    | ROS2 Developer             | mostafashabib51@gmail.com   |
| 5 | Ahmed Alaa        | Embedded Engineer          | ahmedelkholy.com1@gmail.com   |
| 6 | Rahma Elramdy     | ROS2 Developer             | rahmaelramady5@gmail.com   |
| 7 | Shahd ALi         | ROS2 Developer             | shahdalihussein072@gmail.com   |
| 8 | Sally Faragallah  | ROS2 Developer             | Sallyfaragalla1@gmail.com   |
| 9 | Aya Rezq          | ROS2 Developer             | ayarezq144@gmail.com   |
|10 | Fatma El-Zahraa  | ROS2 Developer             | mohamedzharaa73@gmail.com  |

> **Team VEGA**

---

## 🏗️ System Architecture

### Node Graph

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         PHYSICAL ROBOT                                  │
│                                                                         │
│  [5× IR Sensors] ──ADC──► [ATmega32] ──UART──► S:XXXXX\n               │
│  [Left Encoder]  ──INT0──► [ATmega32] ──UART──► E:L,R\n                │
│  [Right Encoder] ──INT1──► [ATmega32]                                   |
│  [DC Motor L]    ◄──PWM── [ATmega32] ◄──UART── F7\n                    │
│  [DC Motor R]    ◄──PWM── [ATmega32]                                   │
└─────────────────────────────┬───────────────────────────────────────────┘
                              │ HC-05 Bluetooth  (9600 baud)
                              │
┌─────────────────────────────▼───────────────────────────────────────────┐
│                      ROS2 HUMBLE GRAPH                                  │
│                                                                         │
│  ┌────────────────────────────────────────────────────────────────┐     │
│  │  serial_comm_node      /dev/rfcomm0                            │     │
│  └──────────┬──────────────────────────────────┬─────────────────┘     │
│             │ /ir_raw (String)                  │ /encoder_raw (String) │
│             ▼                                   ▼                       │
│  ┌──────────────────┐                ┌──────────────────────────┐      │
│  │ line_sensor_node │                │ encoder_odometry_node     │      │
│  │ centroid → error │                │ ticks → x, y, θ          │      │
│  └──────────┬───────┘                └──────────────┬───────────┘      │
│             │ /line_error (Float32)                 │ /odom             │
│             ▼                                       │ (Odometry + TF)   │
│  ┌──────────────────┐                               ▼                   │
│  │line_controller   │                    ┌──────────────────┐           │
│  │ PID  50 Hz loop  │                    │   RViz2 / Nav2   │           │
│  └──────────┬───────┘                    └──────────────────┘           │
│             │ /cmd_vel (Twist)                                           │
│             ▼                                                            │
│  ┌──────────────────┐                                                   │
│  │motor_driver_node │                                                   │
│  │ Twist → 'F7'     │                                                   │
│  └──────────┬───────┘                                                   │
│             │ /motor_cmd → serial_comm_node → UART → ATmega32           │
└─────────────────────────────────────────────────────────────────────────┘
```

### Data Flow Pipeline

```
IR Sensors → ADC → ATmega32 → UART/BT → serial_comm_node
    → /ir_raw → line_sensor_node
        → /line_error → line_controller_node (PID @ 50 Hz)
            → /cmd_vel → motor_driver_node
                → /motor_cmd → serial_comm_node → UART/BT → ATmega32
                    → H-Bridge → DC Motors → Robot Motion
                        → Encoders → INT0/INT1 → ATmega32
                            → /encoder_raw → encoder_odometry_node
                                → /odom  (dead-reckoning loop closes here)
```

### UART Packet Protocol

| Direction | Frame Format | Example | Meaning |
|-----------|-------------|---------|---------|
| AVR → PC | `S:XXXXX\n` | `S:00100\n` | IR array — line under sensor 2 |
| AVR → PC | `E:L,R\n` | `E:120,118\n` | Encoder ticks (left, right) |
| PC → AVR | `XY\n` | `F7\n` | Motor command — Forward 70% |
| PC → AVR | `PING\n` | `PING\n` | Handshake request |
| AVR → PC | `PONG\n` | `PONG\n` | Handshake response |

> **Single-frame protocol:** Direction and speed are combined into one frame (e.g. `F7`) to prevent the AVR ISR from dropping the speed byte while `data_ready=1` is pending from the direction byte.

---

## ⚙️ Hardware Components

| Component | Model | Quantity | Role |
|-----------|-------|----------|------|
| Microcontroller | ATmega32 @ 16 MHz | 1 | Main embedded controller — ADC, UART, PWM, interrupts |
| Bluetooth Module | HC-05 | 1 | Transparent UART bridge to host PC |
| IR Sensor Array | TCRT5000 × 5 | 1 array | Reflectance-based line detection (ADC Ch0–4) |
| Motor Driver | L298N H-Bridge | 1 | Dual H-bridge for differential drive motor control |
| DC Motors + Gearbox | TT Gear Motor | 2 | Drive wheels — PWM speed + direction control |
| Optical Encoders | 20 PPR slotted disc | 2 | Wheel tick counting via INT0 / INT1 |
| Power Supply | 3S LiPo / 12V | 1 | Motor supply (12V); 5V regulator for logic |
| Robot Chassis | Differential drive | 1 | Two driven wheels + passive front caster |
| Wiring / PCB | Custom | — | Signal routing, decoupling capacitors |

### Wiring Summary

```
ATmega32 Pin     →   Connected To
─────────────────────────────────────────
PA0–PA4 (ADC)    →   IR Sensor outputs (0–4)
PD2 (INT0)       →   Left encoder signal
PD3 (INT1)       →   Right encoder signal
PD0 (RXD)        →   HC-05 TX
PD1 (TXD)        →   HC-05 RX
PB3 (OC0)        →   L298N ENA (Left PWM)
PBx (GPIO)       →   L298N IN1, IN2 (Left direction)
PBx (GPIO)       →   L298N IN3, IN4 (Right direction)
PBx (OC)         →   L298N ENB (Right PWM)
```

---

## 📡 Bluetooth Setup (Linux)

Follow these steps **once** to pair and bind the HC-05 module on any Linux host.

### Step 1 — Start Bluetooth Service

```bash
sudo systemctl start bluetooth
sudo systemctl enable bluetooth     # optional: auto-start on boot
sudo systemctl status bluetooth     # verify it is running
```

### Step 2 — Enter bluetoothctl and Scan

```bash
sudo bluetoothctl
```

Inside the bluetoothctl prompt:

```
[bluetooth]# power on
[bluetooth]# agent on
[bluetooth]# default-agent
[bluetooth]# scan on
```

Wait for the HC-05 to appear (usually listed as `HC-05` or `LinvorA`). Note the MAC address `XX:XX:XX:XX:XX:XX`.

```
[bluetooth]# scan off
```

### Step 3 — Pair, Trust, and Connect

```bash
# Replace XX:XX:XX:XX:XX:XX with your HC-05 MAC address
[bluetooth]# pair XX:XX:XX:XX:XX:XX
# Enter PIN when prompted: 1234  (or 0000)

[bluetooth]# trust XX:XX:XX:XX:XX:XX
[bluetooth]# connect XX:XX:XX:XX:XX:XX
[bluetooth]# exit
```

### Step 4 — Bind to Serial Port

```bash
# Bind HC-05 to /dev/rfcomm0
sudo rfcomm bind /dev/rfcomm0 XX:XX:XX:XX:XX:XX

# Grant serial port permissions
sudo chmod 666 /dev/rfcomm0

# Add your user to the dialout group (persistent, requires re-login)
sudo usermod -aG dialout $USER
```

### Step 5 — Verify Connection

```bash
# List bound rfcomm devices
rfcomm show

# Quick serial test (should see S: and E: frames from the robot)
cat /dev/rfcomm0
```

### Finding the HC-05 Port Automatically

```bash
# List all Bluetooth serial devices
ls /dev/rfcomm*

# Or check dmesg after connecting
dmesg | grep rfcomm
```

### Common Bluetooth Problems and Fixes

| Problem | Likely Cause | Fix |
|---------|-------------|-----|
| `rfcomm: No such file or directory` | rfcomm kernel module not loaded | `sudo modprobe rfcomm` |
| `Permission denied: /dev/rfcomm0` | User not in dialout group | `sudo chmod 666 /dev/rfcomm0` or `sudo usermod -aG dialout $USER` |
| HC-05 not appearing in scan | HC-05 in AT mode / not discoverable | Power cycle HC-05; press button briefly |
| `Connection refused` | HC-05 not in slave mode | Re-flash HC-05 AT config: `AT+ROLE=0` |
| Keeps disconnecting | Weak power supply to HC-05 | Use stable 3.3V regulated supply |
| `No PONG received` | Baud rate mismatch | Verify HC-05 AT baud matches firmware (9600) |
| rfcomm device disappears after reconnect | rfcomm not released | `sudo rfcomm release /dev/rfcomm0` then rebind |

---

## 🏗️ Workspace Setup

### Prerequisites

```bash
# ROS2 Humble must be installed and sourced
source /opt/ros/humble/setup.bash

# Required Python packages
pip install pyserial
```

### Create and Build

```bash
# Create workspace (skip if it already exists)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the project
git clone https://github.com/VEGA-team/line_follower.git

# Build
cd ~/ros2_ws
colcon build --packages-select line_follower

# Source the workspace
source install/setup.bash
```

### Add to Shell Profile (recommended)

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Package Structure

```
line_follower/
├── line_follower/
│   ├── __init__.py
│   ├── serial_comm_node.py          # Hardware UART gateway
│   ├── line_sensor_node.py          # IR → error calculation
│   ├── line_controller_node.py      # PID control loop
│   ├── motor_driver_node.py         # Twist → AVR command
│   └── encoder_odometry_node.py     # Dead-reckoning odometry
├── launch/
│   └── line_follow.launch.py        # Single-command launcher
├── resource/
│   └── line_follower
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

---

## 🚀 Running the Project

### Pre-flight Checklist

- [ ] HC-05 is powered and blinking (fast blink = discoverable, slow blink = connected)
- [ ] `/dev/rfcomm0` is bound: `rfcomm show`
- [ ] Workspace is sourced: `source ~/ros2_ws/install/setup.bash`
- [ ] Robot is placed on the track with sensors over the line
- [ ] Battery voltage is adequate (check motor supply)

### Step 1 — Prepare Bluetooth

```bash
sudo rfcomm bind /dev/rfcomm0 XX:XX:XX:XX:XX:XX
sudo chmod 666 /dev/rfcomm0
```

### Step 2 — Build and Source

```bash
cd ~/ros2_ws
colcon build --packages-select line_follower
source install/setup.bash
```

### Step 3 — Launch the Robot

```bash
ros2 launch line_follower line_follow.launch.py
```

You should see:

```
[serial_comm_node]: Waiting for device on /dev/rfcomm0...
[serial_comm_node]: Port opened — sending PING...
[serial_comm_node]: ✅ Device connected — response: PONG
[line_sensor_node]: Line Sensor Node is ready, listening to /ir_raw...
[line_controller_node]: LineControllerNode started | PID: Kp=0.5 Ki=0.0 Kd=0.05
[motor_driver_node]: MotorDriverNode started | Turn speed increased | Max Error safety active
[encoder_odometry_node]: EncoderOdometryNode ready | r=0.033 m  L=0.160 m  PPR=20
```

The robot will begin moving autonomously once the handshake is confirmed.

### Step 4 — Monitor Topics

```bash
# List all active topics
ros2 topic list

# Watch IR sensor data
ros2 topic echo /ir_raw

# Watch the lateral error in real time
ros2 topic echo /line_error

# Watch velocity commands
ros2 topic echo /cmd_vel

# Watch motor commands sent to the AVR
ros2 topic echo /motor_cmd

# Watch odometry
ros2 topic echo /odom

# Check publish rates
ros2 topic hz /line_error
ros2 topic hz /odom
```

### Optional — Visualise in RViz2

```bash
rviz2
# Add: Odometry display → /odom
# Add: TF display (odom → base_link transform)
```

### Record a Bag File

```bash
# Record all relevant topics for offline analysis
ros2 bag record /ir_raw /line_error /cmd_vel /motor_cmd /encoder_raw /odom -o run_001

# Play back later
ros2 bag play run_001/ --clock
```

---

## 📊 ROS2 Topics

| Topic | Message Type | Publisher | Subscriber | Description |
|-------|-------------|-----------|-----------|-------------|
| `/ir_raw` | `std_msgs/String` | `serial_comm_node` | `line_sensor_node` | Raw 5-bit IR binary frame from AVR. Format: `S:XXXXX` where each `X` is `0` (white) or `1` (black/line). Example: `S:00100` means line under centre sensor. |
| `/line_error` | `std_msgs/Float32` | `line_sensor_node` | `line_controller_node` | Normalised lateral deviation ∈ `[−1.0, +1.0]`. `0.0` = perfectly centred. `+1.0` = line at rightmost sensor. `−1.0` = line at leftmost sensor. |
| `/cmd_vel` | `geometry_msgs/Twist` | `line_controller_node` | `motor_driver_node` | Target robot velocity. `linear.x` = forward speed (m/s). `angular.z` = yaw rate (rad/s, positive = left). |
| `/motor_cmd` | `std_msgs/String` | `motor_driver_node` | `serial_comm_node` | Single combined AVR command string. `F7` = Forward 70%, `L4` = Left turn 40%, `R6` = Right turn 60%, `S` = Stop. |
| `/encoder_raw` | `std_msgs/String` | `serial_comm_node` | `encoder_odometry_node` | Cumulative encoder tick counts from AVR. Format: `E:left,right`. Example: `E:120,118` = 120 left ticks ≈ 1.25 m, 118 right ticks ≈ 1.22 m. |
| `/odom` | `nav_msgs/Odometry` | `encoder_odometry_node` | RViz2 / Nav2 | Full dead-reckoning pose estimate. Fields: `pose.pose.position.{x,y}` (metres), `pose.pose.orientation` (quaternion from yaw θ), `twist.twist.linear.x` (m/s), `twist.twist.angular.z` (rad/s). |

### Topic Example Values

```bash
# /ir_raw — line centred under sensor 2
data: "S:00100"

# /line_error — line slightly to the right
data: 0.5

# /cmd_vel — moving forward with slight right correction
linear:
  x: 0.10
angular:
  z: -0.38

# /motor_cmd — right turn at speed level 6
data: "R6"

# /encoder_raw — ~1.2 m driven
data: "E:116,114"

# /odom — robot at x=0.45m, y=0.12m, heading 18°
pose:
  position: {x: 0.45, y: 0.12, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.156, w: 0.988}
```

---

## 🧠 Controller Logic

### 1. IR Line Detection

The ATmega32 reads five IR sensors (ADC Ch0–4) every 50 ms and applies a binary threshold:

```
ADC value < 600  →  1  (dark surface — line detected)
ADC value ≥ 600  →  0  (light surface — floor)
```

Result is transmitted as `S:XXXXX\n` over UART.

### 2. Weighted-Centroid Error Calculation

```python
weights  = [-2, -1, 0, +1, +2]          # sensor position weights
centroid = Σ(weight_i × reading_i) / Σ(reading_i)
error    = clamp(centroid / 2.0, -1.0, +1.0)
```

| Pattern | Sensor State | Error | Meaning |
|---------|-------------|-------|---------|
| `00100` | ░░▓░░ | `0.00` | Line centred — no correction needed |
| `00010` | ░░░▓░ | `+0.50` | Line right — turn right |
| `00001` | ░░░░▓ | `+1.00` | Line far right — hard right |
| `01000` | ░▓░░░ | `−0.50` | Line left — turn left |
| `10000` | ▓░░░░ | `−1.00` | Line far left — hard left |
| `00000` | ░░░░░ | `±last` | Line lost — search using last direction |

### 3. PID Controller

```
u(t) = Kp·e(t) + Ki·∫e dt + Kd·de/dt

angular_z = u(t)          [clamped to ±max_angular = ±1.0 rad/s]
linear_x  = vbase·(1−|e|) + vmin·|e|      [speed reduces on curves]
```

| Parameter | Value | Effect |
|-----------|-------|--------|
| `Kp` | 0.5 | Primary proportional correction strength |
| `Ki` | 0.0 | Disabled — prevents wind-up on noisy IR |
| `Kd` | 0.05 | Derivative damping — reduces oscillation |
| `base_speed` | 0.15 m/s | Full speed when error = 0 |
| `min_speed` | 0.05 m/s | Minimum speed at maximum error |
| `max_angular` | 1.0 rad/s | Angular velocity ceiling |

### 4. Motor Correction

```python
if |ω| > angular_threshold:  direction = 'L' if ω > 0 else 'R'
elif v > linear_threshold:   direction = 'F'
else:                        direction = 'S'

# Combined single-frame protocol: "F7", "L4", "R6", "S"
# Prevents AVR ISR race condition that discarded the speed byte
```

---

## 🔧 Troubleshooting

### HC-05 Not Connecting

```bash
# Check Bluetooth service
sudo systemctl status bluetooth

# Re-scan and verify MAC address
sudo bluetoothctl
[bluetooth]# scan on

# Release and rebind rfcomm
sudo rfcomm release /dev/rfcomm0
sudo rfcomm bind /dev/rfcomm0 XX:XX:XX:XX:XX:XX
sudo chmod 666 /dev/rfcomm0
```

**Check:** HC-05 LED should blink slowly (≈1 Hz) when connected. Fast blinking means not paired.

---

### Permission Denied on `/dev/rfcomm0`

```bash
sudo chmod 666 /dev/rfcomm0

# Permanent fix:
sudo usermod -aG dialout $USER
# Then log out and log back in
```

---

### rfcomm Module Missing

```bash
sudo modprobe rfcomm

# Make it persistent
echo "rfcomm" | sudo tee /etc/modules-load.d/rfcomm.conf
```

---

### No ROS2 Topics Appearing

```bash
# Verify all nodes are running
ros2 node list

# Check serial_comm_node specifically
ros2 node info /serial_reader_node

# If missing, launch manually
ros2 run line_follower serial_comm

# Check for Python import errors
python3 -c "import serial; print(serial.__version__)"
```

---

### Robot Not Moving After Launch

1. Confirm handshake: look for `✅ Device connected — response: PONG` in terminal
2. Check `/motor_cmd` is being published: `ros2 topic echo /motor_cmd`
3. Check `/line_error` is non-zero: `ros2 topic echo /line_error`
4. Verify AVR firmware is flashed and running (ATmega32 LED or UART output)
5. Check battery voltage — motors stall below ~9V on a 12V system

---

### Motors Spinning in Wrong Direction

Edit `MOTOR_config.h` in the AVR firmware to swap H-bridge direction pins, or swap the motor lead wires on the L298N output terminals. Alternatively adjust the direction logic in `DcMotor_voidForward()`.

---

### Encoder Data Missing (`/encoder_raw` empty)

```bash
ros2 topic echo /encoder_raw
```

- Verify encoder ISRs are enabled: check `ENCODER_voidInit()` is called before `GI_voidEnable()` in `main.c`
- Verify encoder disc and interrupter are aligned
- Check INT0/INT1 pin connections (PD2, PD3 on ATmega32)
- Confirm `encoder_ppr` parameter matches firmware PPR (default: 20)

---

### Serial `No PONG received` Warning

```bash
# Check AVR is responding
cat /dev/rfcomm0      # should see S: and E: frames

# Manually send PING
echo -ne "PING\n" > /dev/rfcomm0
cat /dev/rfcomm0      # should see PONG
```

**Common causes:** wrong baud rate, AVR not running, HC-05 connected to a different device.

---

### ROS2 `source` Not Found

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Add to .bashrc permanently
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## 📈 Future Improvements

| Feature | Description | Priority |
|---------|-------------|----------|
| 🎥 Camera-based detection | Replace IR array with monocular camera + OpenCV lane detection | High |
| 🗺️ SLAM Integration | Add LiDAR or depth camera for simultaneous localisation and mapping | High |
| 🧭 IMU fusion | Combine wheel odometry with IMU for improved heading estimation | Medium |
| 📡 LiDAR obstacle avoidance | Add RPLiDAR A1 for real-time obstacle detection and avoidance | Medium |
| 🤖 Nav2 autonomous navigation | Integrate ROS2 Nav2 stack for point-to-point navigation beyond fixed tracks | High |
| 🔄 Higher resolution encoders | Replace 20 PPR encoders with 100+ PPR for improved odometry | Medium |
| 📊 Web dashboard | Real-time browser dashboard showing sensor data, error plots, and trajectory | Low |
| 🧠 ML-based control | Replace PID with a trained neural network controller | Experimental |
| ⚡ Wireless charging | Autonomous docking and wireless power transfer station | Low |
| 🔌 Wired UART fallback | USB-UART cable as fallback when Bluetooth is unavailable | Low |

---

## 📁 Quick Reference

```bash
# ── Build ─────────────────────────────────────────────
cd ~/ros2_ws && colcon build --packages-select line_follower
source install/setup.bash

# ── Bluetooth ─────────────────────────────────────────
sudo rfcomm bind /dev/rfcomm0 XX:XX:XX:XX:XX:XX
sudo chmod 666 /dev/rfcomm0

# ── Launch ────────────────────────────────────────────
ros2 launch line_follower line_follow.launch.py

# ── Monitor ───────────────────────────────────────────
ros2 topic echo /line_error
ros2 topic echo /cmd_vel
ros2 topic echo /odom
ros2 topic hz /line_error

# ── Record ────────────────────────────────────────────
ros2 bag record /ir_raw /line_error /cmd_vel /motor_cmd /encoder_raw /odom -o run_001
ros2 bag play run_001/ --clock

# ── Debug ─────────────────────────────────────────────
ros2 node list
ros2 node info /serial_reader_node
ros2 topic list
```

---

## 📄 License

This project is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.

```
MIT License — Copyright (c) 2026 Team VEGA
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software to use, copy, modify, merge, and distribute, subject to the
following conditions: The above copyright notice shall be included in all copies.
```

---

## 🙏 Acknowledgments

- **ROS2 Community** — for the Humble Hawksbill documentation and rclpy framework
- **AVR-LibC Project** — for the open-source AVR C library
- **PySerial** — for the Python serial communication library
- **Open-source robotics community** — for tutorials, code examples, and issue solutions that made this project possible

---

<div align="center">

**Team VEGA** 

*Built with ❤️ using ROS2, C, Python, and a lot of debugging*

</div>

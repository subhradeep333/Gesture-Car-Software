# Real-Time AI Hand Gesture Controlled IoT Car (ESP32 Wi-Fi & PySide6 Edition)

An end-to-end, ultra-low latency, real-time AI hand gesture-controlled IoT car software built with Python 3, PySide6 (Qt6), OpenCV, MediaPipe, WebSockets, and modular ESP32 C++ firmware.

---

## 📐 System Architecture & Wireless Data Flow

```text
LAPTOP (macOS)
┌─────────────────────────────────────────────────────────┐
│ Live Camera Feed (AVFoundation)                         │
│   ↓                                                     │
│ OpenCV & MediaPipe Hands (21 3D Landmarks)              │
│   ↓                                                     │
│ Gesture Classifier & Temporal Hysteresis                │
│   ↓                                                     │
│ Safety Rules Engine (Camera/Confidence/GUI Check)       │
│   ↓                                                     │
│ WebSocket Client Manager (ws://192.168.4.1:81)          │
└─────────────────────────────────────────────────────────┘
                            │
                            │ Structured JSON Packets
                            │ {"command":"F","speed":180,"sequence":125}
                            ▼
CAR (ESP32 Controller)
┌─────────────────────────────────────────────────────────┐
│ ESP32 Wi-Fi SoftAP ("GestureCar" @ 192.168.4.1)         │
│   ↓                                                     │
│ WebSocket Server (Port 81)                              │
│   ↓                                                     │
│ ArduinoJson Deserializer & Telemetry Broadcaster        │
│   ↓                                                     │
│ 500ms Hardware Watchdog Safety Controller               │
│   ↓                                                     │
│ Motor Controller (L298N via ESP32 LEDC PWM Channels)    │
│   ↓                                                     │
│ 4 DC Motors (Left & Right Pair)                         │
└─────────────────────────────────────────────────────────┘
```

---

## ✌️ Active Hand Gesture (Return-To-Home)

All manual directional steering hand gestures (pointing, fist, palm) have been cleared to give full priority to the autonomous navigation & 2D mapping engine.

| Gesture | Command | Code | Action | Visual Feedback |
| :--- | :--- | :---: | :--- | :--- |
| ✌️ **Two Fingers (V-Sign / Peace Sign)** | RETURN HOME | `H` | Autonomously calculates angle & drives back to starting coordinate $(0, 0)$ | Cyan badge & glowing card |

> **Note:** The robot navigates autonomously using its servo-mounted ultrasonic sensor. Showing ✌️ at any point summons the car back to where it started. Manual directional buttons remain available in the UI for bench testing.

---

## 🗺️ Autonomous Car Navigation & Real-Time 2D Mapping

The system includes a full-featured **Autonomous Exploration & 2D Occupancy Grid Mapping System**:

* **Occupancy Grid Mapping (`mapping_engine.py`):**  
  Uses **Bresenham's raycasting algorithm** to convert ultrasonic distances and servo angles into a probabilistic 2D spatial grid (6m x 6m @ 4cm/cell resolution). Marks free space, detected walls, and unexplored boundaries dynamically.
* **Dead-Reckoning Kinematic Odometry:**  
  Continuously tracks robot estimated position $(X, Y\text{ cm})$ and heading $(\theta^\circ)$ based on active movement commands and calibrated velocities.
* **Autonomous State Machine (`autonomous_navigator.py`):**  
  Coordinates systematic area exploration:
  - **Scanning:** Halts wheels and sweeps the SG90 servo across $30^\circ - 150^\circ$ to gather panoramic sector clearances (Left, Center, Right).
  - **Exploring:** Advances forward in safe pulses while monitoring ultrasonic path clearance.
  - **Obstacle Avoidance:** Automatically halts and pivots towards the sector with maximum open clearance ($> 25\text{cm}$), or reverses if cornered.
* **Interactive Mapping Dashboard (`map_widget.py`):**  
  Integrated directly as the primary tab in the PySide6 Command Center:
  - **Live 2D Map Canvas:** Displays grid lines, explored space, glowing obstacle points, robot trail history, and live sweeping sensor beam.
  - **Interactive Viewport:** Smooth mouse drag pan and mouse wheel zoom with Recenter (🎯) button.
  - **Controls:** Start Mapping (▶), Pause (⏸), Resume (▶), Stop (⏹), Reset Map (🔄), and Manual / Autonomous mode toggle.
  - **Safety:** Camera hand-tracking remains active — a closed fist (✊) triggers an immediate emergency stop even during autonomous mode.

---

## 🔌 ESP32 Hardware Wiring & Pin Mapping

| ESP32 Pin | Component / Module Pin | Function | Notes |
| :--- | :--- | :--- | :--- |
| **GPIO 25** | **L298N ENA** | Left Motors Speed | ESP32 LEDC PWM Channel 0 |
| **GPIO 26** | **L298N IN1** | Left Motors Direction 1 | Digital Output |
| **GPIO 27** | **L298N IN2** | Left Motors Direction 2 | Digital Output |
| **GPIO 14** | **L298N IN3** | Right Motors Direction 1 | Digital Output |
| **GPIO 18** | **L298N IN4** | Right Motors Direction 2 | Digital Output |
| **GPIO 33** | **L298N ENB** | Right Motors Speed | ESP32 LEDC PWM Channel 1 |
| **GPIO 13** | **SG90 Servo Signal** | Radar Sweeping Motor | ESP32 LEDC PWM Channel 2 (50Hz) |
| **GPIO 5** | **HC-SR04 TRIG** | Ultrasonic Trigger | 10µs Pulse Output |
| **GPIO 19** | **HC-SR04 ECHO** | Ultrasonic Echo | Pulse Width Input |
| **GND** | **GND** | **COMMON GROUND** | ⚠️ Connect to Battery GND & ESP32 GND |
| **VIN / 5V** | **5V Out / VCC** | Power Input | Servo & Sensor 5V VCC |
| — | **12V In** | Battery Pack + | Connect to +7.4V to +12V battery pack |

---

## 📦 Modular Project Structure

```text
gesture-car/
├── main.py                    # Application launcher (PySide6 QApplication event loop)
├── config.py                  # Settings, Wi-Fi credentials, thresholds, Qt styling
├── camera.py                  # Threaded OpenCV camera acquisition (1-frame buffer for macOS)
├── gesture_detector.py        # MediaPipe Hands detector (Tasks & Solutions API)
├── gesture_classifier.py      # Geometric landmark classifier with squared distances & hysteresis
├── safety.py                  # Safety rules engine (camera, confidence, GUI overrides)
├── car_controller.py          # Command state machine, deduplication, & heartbeat timer
├── websocket_manager.py       # Async WebSocket client manager & Qt signal serializer
├── ui.py                      # Modern PySide6 Qt6 desktop GUI dashboard (with Radar Scope)
├── requirements.txt           # Python dependencies
│
└── esp32/
    ├── esp32.ino              # Main ESP32 Arduino sketch
    ├── config.h               # GPIO pin assignments, AP credentials, safety thresholds
    ├── radar_controller.h     # SG90 Servo & HC-SR04 Ultrasonic Radar module header
    ├── radar_controller.cpp   # Non-blocking 30°-150° sweep & obstacle distance engine
    ├── wifi_manager.h         # Wi-Fi SoftAP controller header
    ├── wifi_manager.cpp       # SoftAP initialization implementation
    ├── websocket_server.h     # WebSocket server header
    ├── websocket_server.cpp   # WebSocket JSON request handler & telemetry broadcaster
    ├── motor_controller.h     # L298N Motor driver header
    ├── motor_controller.cpp   # Motor direction & ESP32 LEDC PWM implementation
    ├── safety_controller.h    # 500ms Hardware Watchdog timer header
    └── safety_controller.cpp  # Watchdog auto-stop implementation
```
```

---

## 🏗️ 10-Stage Development & Deployment Guide

### Stage 1: ESP32 Wi-Fi SoftAP Setup
Open Arduino IDE, upload `esp32/main.ino`. Turn on ESP32 and confirm the Wi-Fi network `GestureCar` appears on your laptop.

### Stage 2: Laptop → ESP32 WebSocket Test
Connect your laptop Wi-Fi to `GestureCar` (Password: `12345678`). Run `.venv/bin/python3 main.py` and verify `CONNECTED 🟢` badge in PySide6 GUI.

### Stage 3: ESP32 → L298N → Motors Verification
Power the L298N motor driver with a 2x 18650 Li-ion battery pack (+7.4V). Verify motor direction.

### Stage 4: Keyboard / GUI Motor Speed Control Test
Adjust the motor speed slider in the PySide6 dashboard (100 - 255 PWM).

### Stage 5: Camera Stream Verification
Verify 60 FPS live video preview on the left panel of PySide6 GUI.

### Stage 6 & 7: MediaPipe Hand Tracking & Gesture Classification
Hold your hand in front of the webcam. Test each of the 6 gestures (Index Up, Index Down, Left, Right, Open Palm, Fist).

### Stage 8: Wireless Car Control Integration
Verify hand gesture controls driving the physical car wirelessly.

### Stage 9: Safety System & Timeout Validation
Turn off laptop Wi-Fi or close app -> verify ESP32 auto-stops motors within **500ms** (Watchdog timeout).

### Stage 10: Performance & Latency Optimization
Verify target 30–60 FPS camera acquisition with sub-30ms end-to-end WebSocket transmission delay.

---

## 💻 macOS Installation & Launch Instructions

### 1. Install Dependencies in Python `.venv`
```bash
cd "/Users/apple/iot software"
source .venv/bin/activate
pip install -r requirements.txt
```

### 2. Launch PySide6 GUI Application
```bash
.venv/bin/python3 main.py
```

---

## 🛠️ Arduino IDE Setup for ESP32

1. Install **ESP32 Board Support** in Arduino IDE:
   - Go to **Settings > Additional Boards Manager URLs** and add:
     `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
   - Open **Tools > Board > Boards Manager**, search for **esp32**, and install.
2. Install Required Libraries via **Tools > Manage Libraries**:
   - **`WebSockets`** by Markus Sattler
   - **`ArduinoJson`** (v6 or v7) by Benoit Blanchon
3. Select Board: **ESP32 Dev Module** and upload `esp32/main.ino`.

# Real-Time AI Hand Gesture Controlled IoT Car (ESP32 Wi-Fi & PySide6 Edition)

An end-to-end, ultra-low latency, real-time AI hand gesture-controlled IoT car software built with Python 3, PySide6 (Qt6), OpenCV, MediaPipe, WebSockets, and modular ESP32 C++ firmware.

---

## 📐 System Architecture & Wireless Data Flow

```text
LAPTOP (macOS)
┌─────────────────────────────────────────────────────────┐
│ Live Camera Feed (AVFoundation)                         │
│   ↓                                                     │
│ OpenCV & MediaPipe Hands (21 3D Landmarks)               │
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

## ✋ Gesture Mapping Table

| Gesture | Command | Code | Motor Action | Visual Feedback |
| :--- | :--- | :---: | :--- | :--- |
| ☝️ **One Finger Up** | FORWARD | `F` | Both left & right motors drive forward | Emerald Green badge |
| 👇 **One Finger Down** | BACKWARD | `B` | Both left & right motors drive in reverse | Blue badge |
| 👈 **Pointing Left** | LEFT | `L` | Left wheels reverse, right wheels drive forward | Orange badge |
| 👉 **Pointing Right** | RIGHT | `R` | Left wheels drive forward, right wheels reverse | Purple badge |
| ✋ **Open Palm** | STOP | `S` | All motor PWM set to 0 (Smooth Stop) | Amber badge |
| ✊ **Closed Fist** | EMERGENCY STOP | `E` | Immediate emergency brake & lock | Red badge |

---

## 🔌 ESP32 to L298N Wiring & Pin Mapping

| ESP32 Pin | L298N Module Pin | Function | Notes |
| :--- | :--- | :--- | :--- |
| **GPIO 14** | **ENA** | Left Motors Speed | ESP32 LEDC PWM Channel 0 |
| **GPIO 27** | **IN1** | Left Motors Direction 1 | Digital Output |
| **GPIO 26** | **IN2** | Left Motors Direction 2 | Digital Output |
| **GPIO 32** | **ENB** | Right Motors Speed | ESP32 LEDC PWM Channel 1 |
| **GPIO 25** | **IN3** | Right Motors Direction 1 | Digital Output |
| **GPIO 33** | **IN4** | Right Motors Direction 2 | Digital Output |
| **GND** | **GND** | **COMMON GROUND** | ⚠️ Connect to Battery GND & ESP32 GND |
| **VIN / 5V** | **5V Out** | Power Input | From L298N 5V regulator if jumper attached |
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
├── ui.py                      # Modern PySide6 Qt6 desktop GUI dashboard
├── requirements.txt           # Python dependencies
│
└── esp32/
    ├── main.ino               # Main ESP32 Arduino sketch
    ├── config.h               # GPIO pin assignments, AP credentials, safety thresholds
    ├── wifi_manager.h         # Wi-Fi SoftAP controller header
    ├── wifi_manager.cpp       # SoftAP initialization implementation
    ├── websocket_server.h     # WebSocket server header
    ├── websocket_server.cpp   # WebSocket JSON request handler & telemetry broadcaster
    ├── motor_controller.h     # L298N Motor driver header
    ├── motor_controller.cpp   # Motor direction & ESP32 LEDC PWM implementation
    ├── safety_controller.h    # 500ms Hardware Watchdog timer header
    └── safety_controller.cpp  # Watchdog auto-stop implementation
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

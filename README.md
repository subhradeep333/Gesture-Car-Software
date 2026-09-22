# Real-Time AI Hand Gesture Controlled IoT Car

An end-to-end, lightweight, real-time AI hand gesture-controlled IoT car software built with Python 3, OpenCV, MediaPipe, PySerial, and Arduino C++ with nRF24L01 wireless transceivers.

---

## 📐 System Architecture & Data Flow

```
+------------------+         +------------------+         +------------------------+
|  Laptop Camera   | ------> |  MediaPipe Hands | ------> | Gesture Classifier     |
| (AVFoundation)   |  RGB    |  21 3D Landmarks |         | & Temporal Hysteresis  |
+------------------+         +------------------+         +------------------------+
                                                                      |
                                                            Command (F,B,L,R,S,E)
                                                                      v
+------------------+         +------------------+         +------------------------+
| USB Serial Port  | <------ | Fail-Safe Engine | <------ | Safety Rules Check     |
| (115200 Baud)    |  ASCII  | & Heartbeat      |         | (No Hand / Low Conf)   |
+------------------+         +------------------+         +------------------------+
         |
         | USB Cable
         v
+------------------------+
| Arduino Nano #1        |
| (Transmitter)          |
+------------------------+
         |
         | nRF24L01 2.4GHz RF Link ("CAR01")
         v
+------------------------+
| Arduino Nano #2        |
| (Car Receiver)         |
+------------------------+
         |
         | L298N Motor Driver Pins
         v
+------------------------+
| 4 DC Motors            |
| (Left & Right Pair)    |
+------------------------+
```

---

## ✋ Gesture Mapping Table

| Gesture | Command | ASCII Code | Description | Visual Action |
| :--- | :--- | :---: | :--- | :--- |
| ☝️ **One Finger Up** | FORWARD | `F` | Index finger extended UP, others folded | Car drives forward |
| 👇 **One Finger Down** | BACKWARD | `B` | Index finger extended DOWN, others folded | Car reverses backward |
| 👈 **Pointing Left** | LEFT | `L` | Index finger extended LEFT, others folded | Car turns left (skid steer) |
| 👉 **Pointing Right** | RIGHT | `R` | Index finger extended RIGHT, others folded | Car turns right (skid steer) |
| ✋ **Open Palm** | STOP | `S` | All 5 fingers fully extended | Car stops smoothly |
| ✊ **Fist** | EMERGENCY STOP | `E` | All 5 fingers folded/curled into a fist | Immediate emergency lock |

---

## 🛡️ Safety & Fail-Safe Features

1. **No Hand Detected**: Instantly sends `STOP` (`S`).
2. **Camera Disconnected**: Instantly sends `STOP` (`S`).
3. **Low Confidence (< 70%)**: Instantly sends `STOP` (`S`).
4. **Emergency Fist Gesture**: Triggers immediate `EMERGENCY STOP` (`E`).
5. **Emergency GUI Button**: One-click prominent override triggering `EMERGENCY STOP` (`E`).
6. **Application Closure**: Clean exit trap sending `STOP` over serial before releasing serial ports and threads.
7. **Temporal Hysteresis**: Requires gesture consistency across $N$ consecutive frames (default 5 frames) to prevent erratic command switching.
8. **Hardware Watchdog**: Receiver Arduino automatically shuts down motors if no RF packet is received within **500ms**.

---

## ⚡ Performance Optimization (20-30 FPS)

* **Threaded Camera Capture**: Asynchronous video acquisition via OpenCV `AVFoundation` backend prevents GUI thread blocking.
* **Resized Frame Processing**: Default $640 \times 480$ frame resolution balances tracking precision and speed.
* **Non-Blocking Serial Queue**: Background PySerial worker handles command dispatches without interrupting vision pipeline execution.
* **Command Deduplication**: Transmits commands only when state changes or on heartbeat keep-alive ticks.

---

## 🔌 Hardware Wiring & Pin Mapping

### 1. Arduino Nano #1 (Transmitter)

| nRF24L01 Pin | Arduino Nano Pin | Notes |
| :--- | :--- | :--- |
| **VCC** | **3.3V** | ⚠️ Add 10µF capacitor across VCC & GND! |
| **GND** | **GND** | |
| **CE** | **D9** | Configurable in `transmitter.ino` |
| **CSN** | **D10** | Configurable in `transmitter.ino` |
| **SCK** | **D13** | Hardware SPI |
| **MOSI** | **D11** | Hardware SPI |
| **MISO** | **D12** | Hardware SPI |

---

### 2. Arduino Nano #2 (Car Receiver & L298N)

#### nRF24L01 to Arduino Nano #2
| nRF24L01 Pin | Arduino Nano Pin | Notes |
| :--- | :--- | :--- |
| **VCC** | **3.3V** | ⚠️ Add 10µF capacitor across VCC & GND! |
| **GND** | **GND** | |
| **CE** | **D9** | |
| **CSN** | **D10** | |
| **SCK** | **D13** | |
| **MOSI** | **D11** | |
| **MISO** | **D12** | |

#### L298N Motor Driver to Arduino Nano #2
| L298N Pin | Arduino Nano Pin | Function |
| :--- | :--- | :--- |
| **ENA** | **D5 (PWM)** | Left Motors Speed Control |
| **IN1** | **D2** | Left Motors Direction 1 |
| **IN2** | **D3** | Left Motors Direction 2 |
| **ENB** | **D6 (PWM)** | Right Motors Speed Control |
| **IN3** | **D4** | Right Motors Direction 1 |
| **IN4** | **D7** | Right Motors Direction 2 |
| **GND** | **GND** | **COMMON GROUND** (Connect to Battery GND & Arduino GND) |
| **12V** | **Battery +** | External Power (+7.4V to +12V from 2x 18650 batteries) |

---

## 📦 Project Structure

```text
gesture-car/
├── config.py                  # Settings, thresholds, gesture metadata, UI colors
├── camera.py                  # Threaded OpenCV camera capture
├── gesture_detector.py        # MediaPipe Hands landmark extraction & drawing
├── gesture_classifier.py      # Geometric landmark classification & temporal hysteresis
├── serial_manager.py          # PySerial worker threads & port auto-discovery
├── car_controller.py          # Safety state machine & heartbeat keep-alive
├── ui.py                      # Modern dark GUI with live feed & settings sliders
├── main.py                    # Application launcher
├── requirements.txt           # Dependency specification
└── arduino/
    ├── transmitter/
    │   └── transmitter.ino    # USB Serial to nRF24L01 transmitter sketch
    └── receiver/
        └── receiver.ino       # nRF24L01 to L298N motor driver receiver sketch
```

---

## 💻 macOS Installation & Quick Start Guide

### 1. Clone or Open Project Directory
```bash
cd "/Users/apple/iot software"
```

### 2. Create Python Virtual Environment & Install Dependencies
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

### 3. macOS Privacy Permissions
On macOS, ensure Python/Terminal has permission to access the webcam:
> **System Settings > Privacy & Security > Camera > Grant access to Terminal / iTerm / IDE**.

### 4. Upload Arduino Sketches
1. Open `arduino/transmitter/transmitter.ino` in Arduino IDE and upload to **Arduino Nano #1**.
2. Open `arduino/receiver/receiver.ino` in Arduino IDE and upload to **Arduino Nano #2**.
   *(Requires the **RF24 library** by TMRh20 installed in Arduino IDE via Tools > Manage Libraries)*.

### 5. Launch Application
```bash
source .venv/bin/activate
python3 main.py
```

---

## 🧪 Testing Procedure

1. **Camera Feed & Gesture Verification**:
   - Launch `python3 main.py`.
   - Hold your hand in front of the laptop camera.
   - Test each gesture:
     - ☝️ **Index Up**: Verify GUI shows `FORWARD` (`F`).
     - 👇 **Index Down**: Verify GUI shows `BACKWARD` (`B`).
     - 👈 **Index Left**: Verify GUI shows `LEFT` (`L`).
     - 👉 **Index Right**: Verify GUI shows `RIGHT` (`R`).
     - ✋ **Open Palm**: Verify GUI shows `STOP` (`S`).
     - ✊ **Fist**: Verify GUI shows `EMERGENCY STOP` (`E`).
   - Remove hand from camera view -> verify GUI instantly switches to `STOP` (`S`).

2. **Serial Link Test**:
   - Plug in Arduino Nano #1 (Transmitter).
   - Select its port from the dropdown menu in GUI (e.g. `/dev/cu.usbserial-1410`).
   - Click **Connect**.
   - Watch the **Telemetry Log** in GUI for `TX` commands and `ACK:F`, `ACK:B` responses.

3. **Car Receiver & Motor Direction Test**:
   - Power up the car battery pack.
   - Perform gestures and observe motor rotation:
     - `FORWARD`: Left & Right wheels spin forward.
     - `BACKWARD`: Left & Right wheels spin in reverse.
     - `LEFT`: Left wheels reverse/stop, Right wheels spin forward.
     - `RIGHT`: Left wheels spin forward, Right wheels reverse/stop.
   - Turn off transmitter or close laptop app -> confirm car motors stop within **500ms** (Watchdog check).

---

## 🔍 Troubleshooting Guide

| Problem | Cause | Solution |
| :--- | :--- | :--- |
| **Camera Feed Blank / Black Window** | macOS privacy restriction or invalid camera index | Grant Camera permissions in **System Settings > Privacy & Security > Camera**. Change `CAMERA_INDEX` in `config.py` if using external camera. |
| **nRF24L01 Not Transmitting (`RF_FAIL`)** | Insufficient 3.3V power stability on nRF24L01 | Solder a **10µF - 100µF electrolytic capacitor** directly between VCC and GND pins of the nRF24L01 module. |
| **Serial Port Not Found** | Missing CH340 / FTDI USB driver | Install CH340 driver for macOS if using clone Arduino Nanos. Click the refresh button 🔄 in GUI. |
| **Motors Spinning Opposite Direction** | Inverted motor wiring | Swap motor wire leads on L298N terminals `OUT1/OUT2` or `OUT3/OUT4`. |
| **Erratic Command Switching** | Hand jitter or lighting issue | Increase **Stability Buffer** slider in GUI (e.g. from 5 to 7 frames) or increase **Min Confidence** slider. |
| **Car Keeps Moving After App Close** | Lost RF packet or missing fail-safe | Verify safety watchdog code in `receiver.ino`. Ensure car GND and Arduino GND are connected. |

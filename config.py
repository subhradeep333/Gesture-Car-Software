"""
Global Configuration for ESP32 Wi-Fi / WebSocket AI Hand Gesture Controlled IoT Car.
"""

# ESP32 Wi-Fi Access Point & WebSocket Credentials
WIFI_SSID = "GestureCar"
WIFI_PASS = "12345678"
ESP32_IP = "192.168.4.1"
WS_PORT = 81
WS_URL = f"ws://{ESP32_IP}:{WS_PORT}"

# Camera Acquisition Settings
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
TARGET_FPS = 30

# MediaPipe Hand Tracking Settings
MAX_NUM_HANDS = 1
MODEL_COMPLEXITY = 1  # 1 for full complexity (higher precision landmark tracking)
DEFAULT_MIN_DETECTION_CONFIDENCE = 0.60
DEFAULT_MIN_TRACKING_CONFIDENCE = 0.60

# Temporal Gesture Smoothing Hysteresis
DEFAULT_STABILITY_FRAME_THRESHOLD = 5

# Movement Commands (1-character ASCII)
CMD_FORWARD = 'F'
CMD_BACKWARD = 'B'
CMD_LEFT = 'L'
CMD_RIGHT = 'R'
CMD_STOP = 'S'
CMD_EMERGENCY_STOP = 'E'

# Motor Speed Defaults (PWM Range: 100 - 255)
DEFAULT_MOTOR_SPEED = 180
MIN_MOTOR_SPEED = 100
MAX_MOTOR_SPEED = 255

# Safety & Telemetry Timers
SAFETY_TIMEOUT_MS = 500        # Independent ESP32 Watchdog timeout
HEARTBEAT_INTERVAL_SEC = 0.35  # Keep-alive transmission frequency when holding command

# Gesture Metadata: (Display Name, Icon, Hex Color)
GESTURE_METADATA = {
    CMD_FORWARD: ("FORWARD", "☝️", "#2ecc71"),         # Emerald Green
    CMD_BACKWARD: ("BACKWARD", "👇", "#3498db"),       # Blue
    CMD_LEFT: ("LEFT", "👈", "#f39c12"),               # Orange
    CMD_RIGHT: ("RIGHT", "👉", "#9b59b6"),             # Purple
    CMD_STOP: ("STOP", "✋", "#e67e22"),               # Amber
    CMD_EMERGENCY_STOP: ("EMERGENCY STOP", "✊", "#e74c3c"), # Red
    "UNKNOWN": ("LOW CONFIDENCE", "❓", "#7f8c8d"),    # Gray
    "NONE": ("NO HAND DETECTED", "🚫", "#95a5a6")      # Muted Gray
}

# Dark Modern PySide6 Styling Palette
QSS_DARK_THEME = """
QMainWindow {
    background-color: #121214;
}
QWidget {
    color: #f1f2f6;
    font-family: 'Helvetica', 'Arial', sans-serif;
}
QFrame.Card {
    background-color: #1a1a1e;
    border-radius: 8px;
    border: 1px solid #2c3e50;
}
QLabel.Header {
    font-size: 18px;
    font-weight: bold;
    color: #00d2ff;
}
QLabel.CardHeader {
    font-size: 11px;
    font-weight: bold;
    color: #a4b0be;
}
QLabel.CardValue {
    font-size: 16px;
    font-weight: bold;
}
QPushButton {
    background-color: #242429;
    color: #ffffff;
    border: 1px solid #34495e;
    border-radius: 6px;
    padding: 8px 14px;
    font-size: 12px;
    font-weight: bold;
}
QPushButton:hover {
    background-color: #2c3e50;
}
QPushButton#btn_start {
    background-color: #2ecc71;
    border: none;
}
QPushButton#btn_start:hover {
    background-color: #27ae60;
}
QPushButton#btn_emergency {
    background-color: #e74c3c;
    border: none;
    font-size: 14px;
}
QPushButton#btn_emergency:hover {
    background-color: #c0392b;
}
QSlider::groove:horizontal {
    height: 6px;
    background: #242429;
    border-radius: 3px;
}
QSlider::sub-page:horizontal {
    background: #00d2ff;
    border-radius: 3px;
}
QSlider::handle:horizontal {
    background: #ffffff;
    width: 14px;
    margin-top: -4px;
    margin-bottom: -4px;
    border-radius: 7px;
}
QTextEdit {
    background-color: #0d0d0f;
    color: #f1f2f6;
    border: 1px solid #2c3e50;
    border-radius: 6px;
    font-family: 'Courier New', monospace;
    font-size: 11px;
}
"""

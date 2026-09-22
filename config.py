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
    background-color: #0B0E14;
}
QWidget {
    color: #F1F5F9;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
    font-size: 13px;
}

/* Card Surface Containers */
QFrame.Card {
    background-color: #141A24;
    border-radius: 10px;
    border: 1px solid #232D3F;
}
QFrame.HeaderCard {
    background-color: #18202D;
    border-radius: 10px;
    border: 1px solid #2A364D;
}
QFrame.MetricCard {
    background-color: #141A24;
    border-radius: 8px;
    border: 1px solid #232D3F;
    padding: 8px;
}

/* Headers & Labels */
QLabel.AppTitle {
    font-size: 18px;
    font-weight: 800;
    color: #00E5FF;
    letter-spacing: 0.5px;
}
QLabel.SectionHeader {
    font-size: 12px;
    font-weight: 700;
    color: #94A3B8;
    letter-spacing: 0.8px;
    text-transform: uppercase;
}
QLabel.MetricTitle {
    font-size: 11px;
    font-weight: 700;
    color: #64748B;
    text-transform: uppercase;
}
QLabel.MetricValue {
    font-size: 18px;
    font-weight: 800;
}

/* Buttons */
QPushButton {
    background-color: #1E2736;
    color: #F1F5F9;
    border: 1px solid #2E3B52;
    border-radius: 6px;
    padding: 8px 14px;
    font-size: 12px;
    font-weight: 700;
}
QPushButton:hover {
    background-color: #2A364D;
    border-color: #3B4C69;
}
QPushButton:pressed {
    background-color: #18202D;
}
QPushButton:disabled {
    background-color: #121620;
    color: #475569;
    border-color: #1E2736;
}

/* Primary Action Buttons */
QPushButton#btn_start {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #10B981, stop:1 #059669);
    color: #FFFFFF;
    border: none;
    font-size: 13px;
    font-weight: bold;
    border-radius: 6px;
}
QPushButton#btn_start:hover {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #34D399, stop:1 #10B981);
}
QPushButton#btn_pause {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #F59E0B, stop:1 #D97706);
    color: #FFFFFF;
    border: none;
    font-size: 13px;
    font-weight: bold;
    border-radius: 6px;
}
QPushButton#btn_pause:hover {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #FBBF24, stop:1 #F59E0B);
}
QPushButton#btn_emergency {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #EF4444, stop:1 #DC2626);
    color: #FFFFFF;
    border: none;
    font-size: 13px;
    font-weight: bold;
    border-radius: 6px;
}
QPushButton#btn_emergency:hover {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #F87171, stop:1 #EF4444);
}

/* Tab Widget Styling */
QTabWidget::pane {
    border: 1px solid #232D3F;
    border-radius: 10px;
    background-color: #121620;
    top: -1px;
}
QTabBar::tab {
    background-color: #141A24;
    border: 1px solid #232D3F;
    padding: 9px 18px;
    border-top-left-radius: 8px;
    border-top-right-radius: 8px;
    color: #94A3B8;
    font-weight: bold;
    font-size: 12px;
    margin-right: 4px;
}
QTabBar::tab:selected {
    background-color: #121620;
    color: #00E5FF;
    border-bottom: 2px solid #00E5FF;
}
QTabBar::tab:hover:!selected {
    color: #F1F5F9;
    background-color: #1C2433;
}

/* Sliders */
QSlider::groove:horizontal {
    height: 6px;
    background: #18202D;
    border-radius: 3px;
}
QSlider::sub-page:horizontal {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #00E5FF, stop:1 #3B82F6);
    border-radius: 3px;
}
QSlider::handle:horizontal {
    background: #FFFFFF;
    width: 16px;
    height: 16px;
    margin-top: -5px;
    margin-bottom: -5px;
    border-radius: 8px;
    border: 2px solid #00E5FF;
}
QSlider::handle:horizontal:hover {
    background: #00E5FF;
    border-color: #FFFFFF;
}

/* Progress Bars */
QProgressBar {
    background-color: #18202D;
    border: 1px solid #232D3F;
    border-radius: 6px;
    text-align: center;
    color: #F1F5F9;
    font-weight: bold;
    font-size: 11px;
    height: 14px;
}
QProgressBar::chunk {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #00E5FF, stop:1 #3B82F6);
    border-radius: 5px;
}

/* Terminal Console Log */
QTextEdit {
    background-color: #080B10;
    color: #E2E8F0;
    border: 1px solid #232D3F;
    border-radius: 8px;
    font-family: "Menlo", "Monaco", "Consolas", "Courier New", monospace;
    font-size: 12px;
    line-height: 1.4;
    padding: 8px;
    selection-background-color: #00E5FF;
    selection-color: #000000;
}

/* Scrollbars */
QScrollBar:vertical {
    background-color: #0E131F;
    width: 8px;
    border-radius: 4px;
    margin: 0;
}
QScrollBar::handle:vertical {
    background-color: #2E3B52;
    min-height: 24px;
    border-radius: 4px;
}
QScrollBar::handle:vertical:hover {
    background-color: #00E5FF;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0px;
}

/* Combo Box & Inputs */
QComboBox {
    background-color: #18202D;
    color: #F1F5F9;
    border: 1px solid #2E3B52;
    border-radius: 6px;
    padding: 5px 10px;
    font-size: 12px;
}
QComboBox:hover {
    border-color: #3B4C69;
}
QComboBox QAbstractItemView {
    background-color: #18202D;
    color: #F1F5F9;
    border: 1px solid #2E3B52;
    selection-background-color: #2A364D;
    selection-color: #00E5FF;
}
QLineEdit {
    background-color: #18202D;
    color: #F1F5F9;
    border: 1px solid #2E3B52;
    border-radius: 6px;
    padding: 5px 10px;
    font-size: 12px;
}
QLineEdit:focus {
    border-color: #00E5FF;
}
QCheckBox {
    color: #F1F5F9;
    font-size: 12px;
    spacing: 8px;
}
QCheckBox::indicator {
    width: 16px;
    height: 16px;
    border: 1px solid #2E3B52;
    border-radius: 4px;
    background-color: #18202D;
}
QCheckBox::indicator:checked {
    background-color: #00E5FF;
    border-color: #00E5FF;
}
"""


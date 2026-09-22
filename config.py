"""
Global Configuration for Real-time AI Hand Gesture Controlled IoT Car.
"""

# Camera Settings
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
TARGET_FPS = 30

# MediaPipe Hand Detection Settings
MAX_NUM_HANDS = 1
MODEL_COMPLEXITY = 0  # 0 for light/fast processing, 1 for medium
DEFAULT_MIN_DETECTION_CONFIDENCE = 0.70
DEFAULT_MIN_TRACKING_CONFIDENCE = 0.70

# Gesture Classifier Settings
DEFAULT_STABILITY_FRAME_THRESHOLD = 5  # Number of consecutive frames required for command change

# Car Commands (1-byte ASCII)
CMD_FORWARD = 'F'
CMD_BACKWARD = 'B'
CMD_LEFT = 'L'
CMD_RIGHT = 'R'
CMD_STOP = 'S'
CMD_EMERGENCY_STOP = 'E'

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

# Serial Communication Settings
DEFAULT_BAUD_RATE = 115200
SERIAL_TIMEOUT = 0.1  # seconds
HEARTBEAT_INTERVAL = 0.35  # seconds (keep-alive send interval when holding same command)

# Motor Settings
DEFAULT_MOTOR_SPEED = 220  # PWM 0-255
MIN_MOTOR_SPEED = 100
MAX_MOTOR_SPEED = 255

# UI Theme Colors (Dark Modern Palette)
BG_DARK = "#121214"
BG_CARD = "#1a1a1e"
BG_CARD_LIGHT = "#242429"
ACCENT_PRIMARY = "#00d2ff"
ACCENT_SUCCESS = "#2ecc71"
ACCENT_WARNING = "#f39c12"
ACCENT_DANGER = "#e74c3c"
TEXT_PRIMARY = "#f1f2f6"
TEXT_MUTED = "#a4b0be"

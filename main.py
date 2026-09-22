"""
Main Entry Point for ESP32 AI Hand Gesture Controlled IoT Car (PySide6 Edition).
"""

import sys
from PySide6.QtWidgets import QApplication
from camera import CameraStream
from gesture_detector import HandDetector
from gesture_classifier import GestureClassifier
from safety import SafetyManager
from websocket_manager import WebSocketManager
from car_controller import CarController
from ui import MainWindow

def main():
    print("==========================================================")
    print(" ESP32 AI Hand Gesture Controlled IoT Car System (PySide6)")
    print("==========================================================")

    # 1. Initialize PySide6 Application
    app = QApplication(sys.argv)

    # 2. Instantiate Core Subsystems
    camera_stream = CameraStream()
    hand_detector = HandDetector()
    gesture_classifier = GestureClassifier()
    safety_manager = SafetyManager()
    websocket_manager = WebSocketManager()
    car_controller = CarController(safety_manager, websocket_manager)

    # 3. Start Background Threads
    print("[+] Initializing camera stream...")
    if not camera_stream.start():
        print("[!] Warning: Unable to open camera. Check permissions/connections.")

    print("[+] Starting WebSocket Manager...")
    websocket_manager.start()

    # 4. Launch PySide6 GUI MainWindow
    window = MainWindow(
        camera_stream=camera_stream,
        hand_detector=hand_detector,
        gesture_classifier=gesture_classifier,
        safety_manager=safety_manager,
        car_controller=car_controller,
        websocket_manager=websocket_manager
    )
    window.show()

    # 5. Run Application Event Loop
    sys.exit(app.exec())

if __name__ == '__main__':
    main()

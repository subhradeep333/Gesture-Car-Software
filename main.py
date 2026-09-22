"""
Main Entry Point for Real-time AI Hand Gesture Controlled IoT Car.
"""

import sys
import tkinter as tk
from camera import CameraStream
from gesture_detector import HandDetector
from gesture_classifier import GestureClassifier
from serial_manager import SerialManager
from car_controller import CarController
from ui import CarControlUI

def main():
    print("==========================================================")
    print(" Real-Time AI Hand Gesture Controlled IoT Car System")
    print("==========================================================")
    
    # 1. Instantiate Core Subsystems
    camera_stream = CameraStream()
    hand_detector = HandDetector()
    gesture_classifier = GestureClassifier()
    serial_manager = SerialManager()
    car_controller = CarController(serial_manager)

    # 2. Start Camera Stream
    print("[+] Initializing camera stream...")
    if not camera_stream.start():
        print("[!] Error: Unable to open camera. Please check camera connection and macOS privacy permissions.")
        # Proceed so GUI opens and allows user troubleshooting/serial connection even without video
    else:
        print("[+] Camera stream initialized successfully.")

    # 3. Create Tkinter GUI Root
    root = tk.Tk()
    app = CarControlUI(
        root=root,
        camera_stream=camera_stream,
        hand_detector=hand_detector,
        gesture_classifier=gesture_classifier,
        car_controller=car_controller,
        serial_manager=serial_manager
    )

    # 4. Start Event Loop
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("\n[!] KeyboardInterrupt received. Shutting down system...")
    finally:
        print("[+] Stopping system modules...")
        car_controller.stop()
        serial_manager.disconnect()
        camera_stream.stop()
        hand_detector.close()
        print("[+] System shutdown complete.")

if __name__ == '__main__':
    main()

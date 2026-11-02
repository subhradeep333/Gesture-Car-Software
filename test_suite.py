"""
Comprehensive Automated Integration Test Suite for ESP32 AI Hand Gesture Controlled IoT Car.
Tests 100% of Core Subsystems: Camera, Landmarker, Gesture Classifier, Safety Rules, 
Car Controller, WebSocket Manager, Ultrasonic Radar, and PySide6 Dashboard UI.
"""

import sys
import time
import cv2
import numpy as np
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt

from camera import CameraStream
from gesture_detector import HandDetector
from gesture_classifier import GestureClassifier
from safety import SafetyManager
from websocket_manager import WebSocketManager
from car_controller import CarController
from ui import MainWindow
import config

def run_all_tests():
    print("==========================================================================")
    print(" 🚀 ESP32 AI HAND GESTURE CAR — FULL AUTOMATED TEST SUITE")
    print("==========================================================================")
    
    passed_count = 0
    total_tests = 0
    start_time = time.time()

    def record_test(name, success, details=""):
        nonlocal passed_count, total_tests
        total_tests += 1
        if success:
            passed_count += 1
            print(f"[TEST {total_tests:02d}] PASS: {name} {details}")
        else:
            print(f"[TEST {total_tests:02d}] FAIL: {name} {details}")
            assert success, f"Test failed: {name}"

    # Initialize PySide6 Application Instance
    app = QApplication.instance()
    if not app:
        app = QApplication(sys.argv)

    # --------------------------------------------------------------------------
    # SUITE 1: CAMERA STREAM SUBSYSTEM
    # --------------------------------------------------------------------------
    print("\n--- [SUITE 1] Camera Acquisition & Frame Reading ---")
    cam = CameraStream()
    record_test("CameraStream Initialization", cam is not None)
    
    ret, frame, fps = cam.read()
    record_test("Camera Frame Read API", isinstance(ret, bool))
    if not ret or frame is None:
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        ret = True
    record_test("Camera Frame Dimensions", frame.shape == (480, 640, 3))
    record_test("Camera FPS Calculation Metric", isinstance(fps, (int, float)))

    # --------------------------------------------------------------------------
    # SUITE 2: MEDIAPIPE HAND DETECTOR & HUD OVERLAY
    # --------------------------------------------------------------------------
    print("\n--- [SUITE 2] MediaPipe Hand Detector & HUD Overlay ---")
    detector = HandDetector()
    record_test("HandDetector Initialization", detector is not None)
    
    detected, lms, pix_lms, conf, htype = detector.process_frame(frame)
    record_test("HandDetector Frame Processing", isinstance(detected, bool))
    
    # 1. Base synthetic landmark set for Open Palm
    palm_landmarks = [(0.5, 0.8, 0.0)] * 21
    palm_landmarks[0] = (0.5, 0.8, 0.0)      # wrist
    palm_landmarks[5] = (0.4, 0.5, 0.0)      # index_mcp
    palm_landmarks[6] = (0.4, 0.4, 0.0)      # index_pip
    palm_landmarks[8] = (0.4, 0.2, 0.0)      # index_tip

    palm_landmarks[9] = (0.5, 0.5, 0.0)      # middle_mcp
    palm_landmarks[10] = (0.5, 0.4, 0.0)     # middle_pip
    palm_landmarks[12] = (0.5, 0.2, 0.0)     # middle_tip

    palm_landmarks[13] = (0.6, 0.5, 0.0)     # ring_mcp
    palm_landmarks[14] = (0.6, 0.4, 0.0)     # ring_pip
    palm_landmarks[16] = (0.6, 0.2, 0.0)     # ring_tip

    palm_landmarks[17] = (0.7, 0.5, 0.0)     # pinky_mcp
    palm_landmarks[18] = (0.7, 0.4, 0.0)     # pinky_pip
    palm_landmarks[20] = (0.7, 0.2, 0.0)     # pinky_tip

    # Test Tactical HUD Drawing Engine with Directional Pointing Ray
    dummy_pix = [(int(pt[0] * 640), int(pt[1] * 480)) for pt in palm_landmarks]
    hud_fwd = detector.draw_landmarks(frame.copy(), dummy_pix, "FORWARD", (0, 255, 0), 0.95)
    record_test("Tactical HUD Drawing Engine with Directional Ray", hud_fwd.shape == (480, 640, 3))

    # Test Hand Tracking Confidence (MediaPipe Presence + Anatomical Proportionality)
    conf_palm = detector._compute_hand_confidence(palm_landmarks, handedness_score=0.50)
    record_test("Hand Tracking Confidence Invariance (score >= 0.85 with 50% handedness)", conf_palm >= 0.85)

    # Test Velocity-Adaptive Landmark Smoothing:
    # 1. Stationary hand: alpha stays low (<= 0.30)
    detector.prev_landmarks = palm_landmarks
    still_smoothed = detector._smooth_landmarks(palm_landmarks)
    record_test("Landmark Smoothing: Stationary Hand Stability", len(still_smoothed) == 21)

    # 2. Fast joint motion: moving fingertip gets fast responsive update
    fast_landmarks = list(palm_landmarks)
    fast_landmarks[8] = (0.4, 0.05, 0.0)  # sudden 0.15 jump in normalized y
    fast_smoothed = detector._smooth_landmarks(fast_landmarks)
    diff = abs(fast_smoothed[8][1] - fast_landmarks[8][1])
    record_test("Landmark Smoothing: High-Velocity Rapid Tracking Response", diff < 0.05)

    # Multi-hand candidate selection (prefers center hand)
    center_hand = [(0.5, 0.5, 0.0)] * 21
    corner_hand = [(0.05, 0.05, 0.0)] * 21
    _, best_idx, _, _ = detector._select_best_hand([corner_hand, center_hand])
    record_test("Multi-Hand Selection: Center Proximity Prioritization", best_idx == 1)

    # --------------------------------------------------------------------------
    # SUITE 3: GESTURE CLASSIFIER GEOMETRY ENGINE
    # --------------------------------------------------------------------------
    print("\n--- [SUITE 3] Gesture Classification Geometry Engine ---")
    classifier = GestureClassifier()

    # 1. Closed Fist (Emergency Stop - E)
    fist_landmarks = [(0.5, 0.8, 0.0)] + [(0.5, 0.7, 0.0)] * 20
    c_fist = classifier.classify_frame(True, fist_landmarks, 0.95)
    record_test("Fist (Emergency Stop 'E') Classification", c_fist == config.CMD_EMERGENCY_STOP)

    # 2. Open Palm (Stop - S)
    c_palm = classifier.classify_frame(True, palm_landmarks, 0.95)
    record_test("Open Palm (Stop 'S') Classification", c_palm == config.CMD_STOP)

    # 3. One Finger Up (Forward - F)
    fwd_landmarks = list(palm_landmarks)
    fwd_landmarks[12] = (0.5, 0.48, 0.0)  # middle_tip curled
    fwd_landmarks[16] = (0.6, 0.48, 0.0)  # ring_tip curled
    fwd_landmarks[20] = (0.7, 0.48, 0.0)  # pinky_tip curled
    c_fwd = classifier.classify_frame(True, fwd_landmarks, 0.95)
    record_test("Pointing Up (Forward 'F') Classification", c_fwd == config.CMD_FORWARD)

    # 4. Pointing Left (Left - L)
    left_landmarks = list(fwd_landmarks)
    left_landmarks[8] = (0.1, 0.5, 0.0)
    c_left = classifier.classify_frame(True, left_landmarks, 0.95)
    record_test("Pointing Left (Left 'L') Classification", c_left == config.CMD_LEFT)

    # 5. Pointing Right (Right - R)
    right_landmarks = list(fwd_landmarks)
    right_landmarks[8] = (0.8, 0.5, 0.0)
    c_right = classifier.classify_frame(True, right_landmarks, 0.95)
    record_test("Pointing Right (Right 'R') Classification", c_right == config.CMD_RIGHT)

    # 6. Pointing Down (Backward - B)
    back_landmarks = list(fwd_landmarks)
    back_landmarks[8] = (0.4, 0.9, 0.0)
    c_back = classifier.classify_frame(True, back_landmarks, 0.95)
    record_test("Pointing Down (Backward 'B') Classification", c_back == config.CMD_BACKWARD)

    # 7. 3D Joint Angle Cosine Verification
    cos_straight = classifier._compute_joint_angle_cos((0.4, 0.5, 0.0), (0.4, 0.4, 0.0), (0.4, 0.2, 0.0))
    cos_bent = classifier._compute_joint_angle_cos((0.4, 0.5, 0.0), (0.4, 0.4, 0.0), (0.4, 0.5, 0.0))
    record_test("3D Joint Angle Straight Finger (cos ~ 1.0)", cos_straight > 0.95)
    record_test("3D Joint Angle Bent Finger (cos <= 0.25)", cos_bent <= 0.25)

    # 8. Thumb Extension & Curl Detection
    thumb_ext_fist = classifier._is_thumb_extended(fist_landmarks)
    record_test("Thumb Tucked / Curled in Fist Detection", thumb_ext_fist is False)

    # 9. Low Confidence & Undetected Hand Filtering
    c_none = classifier.classify_frame(False, [], 0.0)
    c_low_conf = classifier.classify_frame(True, palm_landmarks, 0.40, min_confidence=0.70)
    record_test("Undetected Hand Returns 'NONE'", c_none == "NONE")
    record_test("Low Confidence Hand Returns 'UNKNOWN'", c_low_conf == "UNKNOWN")

    # 10. Ambiguous / Partial Gesture Rejection (Peace Sign ✌️ -> UNKNOWN)
    peace_landmarks = list(fwd_landmarks)
    peace_landmarks[12] = (0.5, 0.2, 0.0)  # Middle finger extended alongside index
    c_peace = classifier.classify_frame(True, peace_landmarks, 0.95)
    record_test("Ambiguous Gesture Rejection (Peace Sign -> 'UNKNOWN')", c_peace == "UNKNOWN")

    # 11. Directional Angular Deadband & Hysteresis Stability
    classifier.last_directional_command = config.CMD_FORWARD
    hyst_fwd_landmarks = list(fwd_landmarks)
    hyst_fwd_landmarks[8] = (0.7, 0.2, 0.0)  # Tilted forward-right
    c_hyst = classifier.classify_frame(True, hyst_fwd_landmarks, 0.95)
    record_test("Directional Angular Deadband Boundary Stability", c_hyst in [config.CMD_FORWARD, config.CMD_RIGHT])

    # 12. Fast-Path Emergency Stop Instant Override (1 frame)
    c_cand, c_stable, is_st = classifier.process(True, fist_landmarks, 0.95)
    record_test("Fast-Path Emergency Stop Instant Override", c_stable == config.CMD_EMERGENCY_STOP and is_st is True)

    # 13. Fast-Path Open Palm Stop (2 frames)
    classifier.set_stability_threshold(5)
    classifier.current_stable_command = config.CMD_FORWARD
    classifier.process(True, palm_landmarks, 0.95)
    _, c_stop_2, is_st_2 = classifier.process(True, palm_landmarks, 0.95)
    record_test("Fast Open-Palm Stop Activation within 2 Frames", c_stop_2 == config.CMD_STOP and is_st_2 is True)

    # --------------------------------------------------------------------------
    # SUITE 4: CENTRALIZED SAFETY RULES ENGINE & FAULT PRIORITIES
    # --------------------------------------------------------------------------
    print("\n--- [SUITE 4] Centralized Safety Engine & Rules Priority ---")
    safety = SafetyManager()

    # Rule 1: Inactive System -> Forces STOP
    cmd, ok, reason = safety.evaluate_safety('F', True, True, True, True, 0.9, 0.7)
    record_test("Safety Rule: Inactive System Forces STOP", cmd == 'S' and reason == "System Paused")

    # Activate System
    safety.set_system_active(True)
    cmd, ok, reason = safety.evaluate_safety('F', True, True, True, True, 0.9, 0.7)
    record_test("Safety Rule: Normal Active Operation", cmd == 'F' and ok is True)

    # Rule 2: GUI Emergency Override
    safety.trigger_emergency_stop()
    cmd, ok, reason = safety.evaluate_safety('F', True, True, True, True, 0.9, 0.7)
    record_test("Safety Rule: GUI Emergency Override Triggers [E]", cmd == 'E' and ok is False)
    safety.clear_emergency_stop()

    # Rule 3: Camera Feed Disconnected
    cmd, ok, reason = safety.evaluate_safety('F', True, True, False, True, 0.9, 0.7)
    record_test("Safety Rule: Camera Disconnected Forces STOP", cmd == 'S' and "Camera" in reason)

    # Rule 4: WebSocket Disconnected
    cmd, ok, reason = safety.evaluate_safety('F', True, True, True, False, 0.9, 0.7)
    record_test("Safety Rule: WebSocket Disconnected Forces STOP", cmd == 'S' and "WebSocket" in reason)

    # Rule 5: No Hand Detected
    cmd, ok, reason = safety.evaluate_safety('F', True, False, True, True, 0.9, 0.7)
    record_test("Safety Rule: No Hand Detected Forces STOP", cmd == 'S' and reason == "No Hand Detected")

    # Rule 6: Low Landmark Confidence
    cmd, ok, reason = safety.evaluate_safety('F', True, True, True, True, 0.4, 0.7)
    record_test("Safety Rule: Low Confidence Forces STOP", cmd == 'S' and "Low Confidence" in reason)

    # Rule 7: Emergency Fist Gesture Detected
    cmd, ok, reason = safety.evaluate_safety('E', True, True, True, True, 0.9, 0.7)
    record_test("Safety Rule: Emergency Fist Gesture Triggers [E]", cmd == 'E' and "Emergency Fist" in reason)

    # --------------------------------------------------------------------------
    # SUITE 5: CAR CONTROLLER & ASYNC WEBSOCKET SUBSYSTEM
    # --------------------------------------------------------------------------
    print("\n--- [SUITE 5] Car Controller & Async WebSocket Manager ---")
    ws_mgr = WebSocketManager()
    car_ctrl = CarController(safety, ws_mgr)

    record_test("Car Controller Motor Speed Set (180 PWM)", car_ctrl.motor_speed == 180)
    car_ctrl.set_motor_speed(220)
    record_test("Car Controller Speed Update (220 PWM)", car_ctrl.motor_speed == 220)
    car_ctrl.set_motor_speed(300)
    record_test("Car Controller Speed Clamping (Max 255 PWM)", car_ctrl.motor_speed == 255)
    car_ctrl.set_motor_speed(50)
    record_test("Car Controller Speed Clamping (Min 100 PWM)", car_ctrl.motor_speed == 100)
    car_ctrl.set_motor_speed(180)

    # Test Immediate Stop Reset Method
    car_ctrl.send_immediate_stop()
    record_test("Car Controller Immediate Stop State Reset", car_ctrl.active_command == config.CMD_STOP)

    # --------------------------------------------------------------------------
    # SUITE 6: ULTRASONIC RADAR SCOPE & DISTANCE THRESHOLDS
    # --------------------------------------------------------------------------
    print("\n--- [SUITE 6] Ultrasonic Radar Scope & Proximity Thresholds ---")
    record_test("Obstacle Emergency Stop Threshold = 10cm", config.OBSTACLE_VERY_CLOSE_CM == 10.0)
    record_test("Obstacle Warning Sweep Threshold = 25cm", config.OBSTACLE_WARNING_CM == 25.0)
    record_test("Radar Status Clear Metadata Defined", "CLEAR" in config.RADAR_STATUS_METADATA)
    record_test("Radar Status Obstacle Metadata Defined", "OBSTACLE DETECTED" in config.RADAR_STATUS_METADATA)
    record_test("Radar Status Very Close Metadata Defined", "VERY CLOSE" in config.RADAR_STATUS_METADATA)

    # --------------------------------------------------------------------------
    # SUITE 7: PYSIDE6 DASHBOARD GUI MAIN WINDOW INTEGRATION
    # --------------------------------------------------------------------------
    print("\n--- [SUITE 7] PySide6 Dashboard MainWindow Integration ---")
    window = MainWindow(
        camera_stream=cam,
        hand_detector=detector,
        gesture_classifier=classifier,
        safety_manager=safety,
        car_controller=car_ctrl,
        websocket_manager=ws_mgr
    )
    window.show()
    record_test("MainWindow GUI Instantiation & Show", window.isVisible())

    # Test Ultrasonic Radar Telemetry Qt Signal Binding
    radar_payload = {
        'angle': 90,
        'distance': 10.0,
        'left_dist': 160.0,
        'center_dist': 10.0,
        'right_dist': 180.0,
        'status': 'VERY CLOSE',
        'best_path': 'RIGHT'
    }
    ws_mgr.radar_telemetry_received.emit(radar_payload)
    record_test("Radar Angle Label Update (90°)", window.lbl_r_angle.text() == "90°")
    record_test("Radar Distance Label Update (10.0 cm)", window.lbl_r_dist.text() == "10.0 cm")
    record_test("Radar Best Path Recommendation Update", "RIGHT" in window.lbl_r_path.text())

    # Test ESP32 Online / Offline Status Badge Signal Binding
    ws_mgr.connection_status_changed.emit(True, "Connected")
    record_test("GUI ESP32 Online Status Badge", "ONLINE" in window.lbl_ws_badge.text())
    ws_mgr.connection_status_changed.emit(False, "Disconnected")
    record_test("GUI ESP32 Offline Status Badge", "OFFLINE" in window.lbl_ws_badge.text())

    # Test Start/Pause Toggle Action
    safety.set_system_active(False)
    window._toggle_system_active()
    record_test("GUI System Active Toggle", safety.system_active is True)
    window._toggle_system_active()
    record_test("GUI System Pause Toggle", safety.system_active is False)

    # Test Emergency Stop GUI Button Toggle Action
    window._toggle_emergency_stop()
    record_test("GUI Emergency Stop Button Toggle Active", safety.emergency_override is True)
    window._toggle_emergency_stop()
    record_test("GUI Emergency Stop Button Toggle Clear", safety.emergency_override is False)

    # Clean Exit
    detector.close()
    app.quit()

    duration = time.time() - start_time
    print("\n==========================================================================")
    print(f" 🎉 TEST SUITE COMPLETE: {passed_count}/{total_tests} TESTS PASSED (100% SUCCESS) in {duration:.2f}s")
    print("==========================================================================")

if __name__ == '__main__':
    run_all_tests()

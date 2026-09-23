"""
Centralized Safety Rules Engine for ESP32 Gesture Control.
"""

import config

class SafetyManager:
    def __init__(self):
        self.emergency_override = False
        self.system_active = False
        self.last_fault_reason = "System Initialized (Stopped)"

    def set_system_active(self, active):
        """Activates or pauses system gesture processing."""
        self.system_active = active
        if active:
            self.emergency_override = False  # Clear stale GUI emergency overrides on start
            self.last_fault_reason = "System Active"
        else:
            self.last_fault_reason = "System Paused by User"

    def trigger_emergency_stop(self):
        """Activates immediate Emergency Stop override."""
        self.emergency_override = True
        self.last_fault_reason = "Emergency Stop GUI Button Activated"

    def clear_emergency_stop(self):
        """Clears Emergency Stop override."""
        self.emergency_override = False
        self.last_fault_reason = "Emergency Stop Cleared"

    def evaluate_safety(self, gesture_cmd, is_stable, hand_detected, camera_ok, ws_connected, confidence, min_conf):
        """
        Evaluates system safety rules in order of priority.
        Returns (target_command, safety_ok, fault_reason).
        """
        # Rule 1: System Inactive / Paused (Forces smooth STOP)
        if not self.system_active:
            return config.CMD_STOP, False, "System Paused"

        # Rule 2: Emergency GUI Override
        if self.emergency_override:
            return config.CMD_EMERGENCY_STOP, False, "GUI Emergency Stop Override"

        # Rule 3: Camera Feed Disconnected or Frame Failure
        if not camera_ok:
            return config.CMD_STOP, False, "Camera Feed Disconnected"

        # Rule 4: WebSocket Disconnected
        if not ws_connected:
            return config.CMD_STOP, False, "ESP32 WebSocket Disconnected"

        # Rule 5: No Hand Detected in Camera Field of View
        if not hand_detected:
            return config.CMD_STOP, False, "No Hand Detected"

        # Rule 6: Low Landmark Detection Confidence
        if confidence < min_conf:
            return config.CMD_STOP, False, f"Low Confidence ({confidence*100:.1f}% < {min_conf*100:.1f}%)"

        # Rule 7: Emergency Fist Gesture Detected
        if gesture_cmd == config.CMD_EMERGENCY_STOP:
            return config.CMD_EMERGENCY_STOP, False, "Emergency Fist Gesture Detected"

        # Rule 8: Valid & Stable Gesture Candidate
        if is_stable and gesture_cmd in [config.CMD_FORWARD, config.CMD_BACKWARD, config.CMD_LEFT, config.CMD_RIGHT, config.CMD_STOP]:
            return gesture_cmd, True, "Normal Operation"

        # Default Safety Fallback
        return config.CMD_STOP, True, "Unstable Gesture / Default Stop"

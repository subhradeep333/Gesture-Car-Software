"""
Car Controller orchestrating Safety Evaluation, Command Deduplication, and WebSocket Transmission.
"""

import time
import config

class CarController:
    def __init__(self, safety_manager, websocket_manager):
        self.safety_manager = safety_manager
        self.websocket_manager = websocket_manager
        
        self.active_command = config.CMD_STOP
        self.last_sent_command = None
        self.last_sent_speed = None
        self.last_send_time = 0.0
        
        self.heartbeat_interval = config.HEARTBEAT_INTERVAL_SEC
        self.motor_speed = config.DEFAULT_MOTOR_SPEED

    def set_motor_speed(self, speed):
        """Sets target motor PWM speed (100 to 255)."""
        self.motor_speed = max(config.MIN_MOTOR_SPEED, min(config.MAX_MOTOR_SPEED, int(speed)))

    def update(self, gesture_cmd, is_stable, hand_detected, camera_ok, confidence, min_conf):
        """
        Evaluates safety rules, checks deduplication, and sends command to ESP32 over WebSocket.
        Returns (active_command, safety_ok, fault_reason).
        """
        now = time.time()
        ws_connected = self.websocket_manager.is_connected if self.websocket_manager else False

        # Evaluate safety rules
        target_cmd, safety_ok, fault_reason = self.safety_manager.evaluate_safety(
            gesture_cmd, is_stable, hand_detected, camera_ok, ws_connected, confidence, min_conf
        )

        self.active_command = target_cmd

        # Deduplication & Heartbeat transmission logic:
        # Send if command or speed changed OR if heartbeat interval elapsed
        cmd_changed = (target_cmd != self.last_sent_command)
        speed_changed = (self.motor_speed != self.last_sent_speed)
        heartbeat_due = ((now - self.last_send_time) >= self.heartbeat_interval)

        if (cmd_changed or speed_changed or heartbeat_due):
            if self.websocket_manager and ws_connected:
                self.websocket_manager.send_command(target_cmd, self.motor_speed)
                self.last_sent_command = target_cmd
                self.last_sent_speed = self.motor_speed
                self.last_send_time = now

        return target_cmd, safety_ok, fault_reason

    def send_immediate_stop(self):
        """Sends an immediate STOP packet over WebSocket."""
        self.active_command = config.CMD_STOP
        self.last_sent_command = config.CMD_STOP
        self.last_send_time = time.time()
        if self.websocket_manager and self.websocket_manager.is_connected:
            self.websocket_manager.send_command(config.CMD_STOP, self.motor_speed)

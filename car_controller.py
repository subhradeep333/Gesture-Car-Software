"""
Car Controller managing fail-safe behavior, command deduplication, and heartbeat keep-alive.
"""

import time
import config

class CarController:
    def __init__(self, serial_manager):
        self.serial_manager = serial_manager
        
        self.is_active = False           # True when user clicks Start System
        self.emergency_override = False  # True when GUI Emergency STOP is pressed
        
        self.active_command = config.CMD_STOP
        self.last_sent_command = None
        self.last_send_time = 0.0
        self.heartbeat_interval = config.HEARTBEAT_INTERVAL
        self.motor_speed = config.DEFAULT_MOTOR_SPEED

    def start(self):
        """Starts car controller execution."""
        self.is_active = True
        self.emergency_override = False
        self.active_command = config.CMD_STOP
        self.send_immediate_command(config.CMD_STOP)

    def stop(self):
        """Stops car controller execution and sends STOP to car."""
        self.is_active = False
        self.active_command = config.CMD_STOP
        self.send_immediate_command(config.CMD_STOP)

    def trigger_emergency_stop(self):
        """Triggers emergency stop state."""
        self.emergency_override = True
        self.active_command = config.CMD_EMERGENCY_STOP
        self.send_immediate_command(config.CMD_EMERGENCY_STOP)

    def clear_emergency_stop(self):
        """Clears emergency stop override state."""
        self.emergency_override = False
        self.active_command = config.CMD_STOP
        self.send_immediate_command(config.CMD_STOP)

    def set_motor_speed(self, speed):
        """Sets target motor PWM speed (100 to 255)."""
        self.motor_speed = max(config.MIN_MOTOR_SPEED, min(config.MAX_MOTOR_SPEED, int(speed)))

    def update_gesture_command(self, gesture_cmd, is_stable, detected, camera_ok):
        """
        Evaluates current gesture command against safety fail-safes and dispatches to serial.
        Returns target_cmd (str).
        """
        now = time.time()

        # Fail-Safe Rules Evaluation Order:
        # 1. Emergency GUI Override -> EMERGENCY_STOP
        if self.emergency_override:
            target_cmd = config.CMD_EMERGENCY_STOP
        # 2. System inactive -> STOP
        elif not self.is_active:
            target_cmd = config.CMD_STOP
        # 3. Camera stream disconnected or failed -> STOP
        elif not camera_ok:
            target_cmd = config.CMD_STOP
        # 4. No hand detected -> STOP
        elif not detected:
            target_cmd = config.CMD_STOP
        # 5. Gesture is Emergency Fist -> EMERGENCY_STOP
        elif gesture_cmd == config.CMD_EMERGENCY_STOP:
            target_cmd = config.CMD_EMERGENCY_STOP
        # 6. Gesture valid and stable -> gesture_cmd
        elif is_stable and gesture_cmd in [config.CMD_FORWARD, config.CMD_BACKWARD, config.CMD_LEFT, config.CMD_RIGHT, config.CMD_STOP]:
            target_cmd = gesture_cmd
        else:
            target_cmd = config.CMD_STOP

        self.active_command = target_cmd

        # Deduplication & Heartbeat transmission logic:
        # Send if command changed OR if heartbeat interval elapsed
        if (target_cmd != self.last_sent_command) or ((now - self.last_send_time) >= self.heartbeat_interval):
            if self.serial_manager and self.serial_manager.is_connected:
                self.serial_manager.send_command(target_cmd)
                self.last_sent_command = target_cmd
                self.last_send_time = now

        return target_cmd

    def send_immediate_command(self, cmd_char):
        """Sends immediate unbuffered command over serial."""
        self.active_command = cmd_char
        self.last_sent_command = cmd_char
        self.last_send_time = time.time()
        if self.serial_manager and self.serial_manager.is_connected:
            self.serial_manager.send_command(cmd_char)

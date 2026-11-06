"""
Autonomous Exploration & Navigation Controller for ESP32 AI Car.
Coordinates systematic area exploration, servo scanning, obstacle avoidance,
and dead-reckoning odometry synchronization with the 2D mapping engine.
"""

import math
import time
from PySide6.QtCore import QObject, Signal, QTimer
import config

class AutonomousNavigator(QObject):
    # Signals for UI notifications
    mode_changed = Signal(str)         # "MANUAL" or "AUTONOMOUS"
    state_changed = Signal(str)        # "IDLE", "EXPLORING", "SCANNING", "TURNING", "AVOIDING", "PAUSED"
    command_dispatched = Signal(str)   # 'F', 'B', 'L', 'R', 'S'
    status_message = Signal(str)       # Descriptive status for dashboard

    # Modes
    MODE_MANUAL = "MANUAL"
    MODE_AUTONOMOUS = "AUTONOMOUS"

    # Autonomous States
    STATE_IDLE = "IDLE"
    STATE_EXPLORING = "EXPLORING"
    STATE_SCANNING = "SCANNING"
    STATE_TURNING = "TURNING"
    STATE_AVOIDING = "AVOIDING"
    STATE_BACKING = "BACKING"
    STATE_PAUSED = "PAUSED"
    STATE_HOMING = "HOMING"

    def __init__(self, mapping_engine, websocket_manager, car_controller=None):
        super().__init__()
        self.mapping_engine = mapping_engine
        self.websocket_manager = websocket_manager
        self.car_controller = car_controller

        self.current_mode = self.MODE_MANUAL
        self.current_state = self.STATE_IDLE
        self.is_active = False

        # Navigation decision parameters (cm)
        self.safe_distance_cm = 32.0        # Min clearance to move forward
        self.warning_distance_cm = 28.0     # Obstacle caution threshold (triggers scan & divert)
        self.critical_distance_cm = 12.0    # Emergency stop & reverse threshold

        # Latest radar telemetry cache from ESP32
        self.current_dist = 200.0
        self.current_angle = 90
        self.left_dist = 200.0
        self.center_dist = 200.0
        self.right_dist = 200.0
        self.radar_status = "CLEAR"
        self.best_path = "CENTER"

        # Movement step & timing management
        self.action_start_time = time.time()
        self.action_duration = 0.0
        self.active_command = 'S'
        self.current_speed = 200
        self.turn_direction = 'L'
        self.sweep_cycles_done = 0

        # Heartbeat timer for ESP32 500ms hardware watchdog keep-alive
        self.last_heartbeat_time = 0.0
        self.heartbeat_interval = 0.20  # Send keep-alive every 200ms

        # High-frequency decision loop timer (50ms ~ 20 Hz)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._navigation_loop_tick)
        self.timer.start(50)

    def set_mode(self, mode):
        """Switches between MANUAL (Gesture/UI) and AUTONOMOUS mapping."""
        if mode not in [self.MODE_MANUAL, self.MODE_AUTONOMOUS]:
            return
        self.current_mode = mode
        if mode == self.MODE_MANUAL:
            self._set_state(self.STATE_IDLE)
            self._dispatch_command(config.CMD_STOP)
        self.mode_changed.emit(mode)

    def start_mapping(self):
        """Initiates autonomous scanning and exploration."""
        self.set_mode(self.MODE_AUTONOMOUS)
        self.is_active = True
        self._set_state(self.STATE_SCANNING)
        self.action_start_time = time.time()
        self.action_duration = 0.8  # Initial survey scan duration
        self._dispatch_command(config.CMD_STOP)
        self.status_message.emit("Autonomous mapping started. Surveying surroundings...")

    def pause_mapping(self):
        """Pauses autonomous exploration while keeping the current map intact."""
        if self.current_mode == self.MODE_AUTONOMOUS:
            self.is_active = False
            self._set_state(self.STATE_PAUSED)
            self._dispatch_command(config.CMD_STOP)
            self.status_message.emit("Autonomous mapping paused.")

    def resume_mapping(self):
        """Resumes autonomous exploration."""
        if self.current_mode == self.MODE_AUTONOMOUS:
            self.is_active = True
            self._set_state(self.STATE_SCANNING)
            self.action_start_time = time.time()
            self.action_duration = 0.6
            self._dispatch_command(config.CMD_STOP)
            self.status_message.emit("Autonomous mapping resumed.")

    def stop_robot(self):
        """Immediately halts robot and returns to IDLE."""
        self.is_active = False
        self._set_state(self.STATE_IDLE)
        self._dispatch_command(config.CMD_STOP)
        self.status_message.emit("Robot stopped.")

    def return_to_home(self):
        """Initiates autonomous Return-To-Home navigation back to starting origin (0, 0)."""
        self.set_mode(self.MODE_AUTONOMOUS)
        self.is_active = True
        self._set_state(self.STATE_HOMING)
        self.action_start_time = time.time()
        self.action_duration = 0.0
        self.status_message.emit("Returning to starting position (0, 0)...")

    def reset_map(self):
        """Resets the occupancy grid map and stops robot."""
        self.stop_robot()
        self.mapping_engine.reset_map()
        self.status_message.emit("2D Occupancy Grid map reset to initial state.")

    def on_radar_telemetry(self, radar_data):
        """
        Receives real-time ultrasonic and servo telemetry from ESP32.
        Feeds into mapping engine and updates navigation state.
        """
        if not isinstance(radar_data, dict):
            return

        self.current_dist = float(radar_data.get("distance", 200.0))
        self.current_angle = int(radar_data.get("angle", 90))
        self.left_dist = float(radar_data.get("left_dist", 200.0))
        self.center_dist = float(radar_data.get("center_dist", 200.0))
        self.right_dist = float(radar_data.get("right_dist", 200.0))
        self.radar_status = str(radar_data.get("status", "CLEAR"))
        self.best_path = str(radar_data.get("best_path", "CENTER"))

        # Feed sensor ray directly into the 2D occupancy grid
        self.mapping_engine.add_sensor_reading(self.current_angle, self.current_dist)

    def _set_state(self, new_state):
        if self.current_state != new_state:
            self.current_state = new_state
            self.state_changed.emit(new_state)

    def _dispatch_command(self, cmd_char, speed=200):
        """Dispatches movement command to ESP32 and notifies odometry tracker."""
        self.active_command = cmd_char
        self.current_speed = speed
        self.last_heartbeat_time = time.time()
        if self.websocket_manager and self.websocket_manager.is_connected:
            self.websocket_manager.send_command(cmd_char, speed=speed)
        self.mapping_engine.update_odometry(cmd_char)
        self.command_dispatched.emit(cmd_char)

    def _navigation_loop_tick(self):
        """
        Autonomous state machine tick executed every 50ms.
        """
        now = time.time()

        # Update odometry dead-reckoning on every tick
        self.mapping_engine.update_odometry(self.active_command)

        if self.current_mode != self.MODE_AUTONOMOUS or not self.is_active:
            return

        # Keep-alive heartbeat: prevent ESP32 500ms safety watchdog timeout while driving
        if self.active_command != config.CMD_STOP and (now - self.last_heartbeat_time) >= self.heartbeat_interval:
            self.last_heartbeat_time = now
            if self.websocket_manager and self.websocket_manager.is_connected:
                self.websocket_manager.send_command(self.active_command, speed=self.current_speed)

        elapsed = now - self.action_start_time

        # ----------------------------------------------------
        # STATE: SCANNING (Halt wheels, gather panoramic distance)
        # ----------------------------------------------------
        if self.current_state == self.STATE_SCANNING:
            if self.active_command != config.CMD_STOP:
                self._dispatch_command(config.CMD_STOP)

            # Wait for servo scan to sample environment
            if elapsed >= self.action_duration:
                # Decide next move based on panoramic clearance
                effective_center = min(self.center_dist, self.current_dist if abs(self.current_angle - 90) <= 20 else self.center_dist)

                if effective_center > self.safe_distance_cm:
                    # Clear ahead: Explore forward!
                    self._set_state(self.STATE_EXPLORING)
                    self.action_start_time = now
                    self.action_duration = 0.9  # Move forward in steady 900ms pulses
                    self._dispatch_command(config.CMD_FORWARD, speed=200)
                    self.status_message.emit("Path clear. Exploring forward...")
                else:
                    # Obstacle ahead: Determine best turning direction
                    self._trigger_avoidance()

        # ----------------------------------------------------
        # STATE: EXPLORING (Moving forward while monitoring front clearance)
        # ----------------------------------------------------
        elif self.current_state == self.STATE_EXPLORING:
            # Immediate Reactive Obstacle Guard
            effective_center = min(self.center_dist, self.current_dist if abs(self.current_angle - 90) <= 25 else self.center_dist)

            if effective_center < self.critical_distance_cm:
                # Obstacle is critically close! Emergency halt & back up
                self._set_state(self.STATE_BACKING)
                self.action_start_time = now
                self.action_duration = 0.5  # Reverse for 500ms
                self._dispatch_command(config.CMD_BACKWARD, speed=210)
                self.status_message.emit("Critical proximity detected! Reversing...")
                return

            if effective_center < self.warning_distance_cm:
                # Obstacle detected ahead within warning threshold
                self._trigger_avoidance()
                return

            # Completed exploration forward pulse -> Pause & perform quick radar sweep
            if elapsed >= self.action_duration:
                self._set_state(self.STATE_SCANNING)
                self.action_start_time = now
                self.action_duration = 0.5
                self._dispatch_command(config.CMD_STOP)

        # ----------------------------------------------------
        # STATE: AVOIDING (Car halted, servo & ultrasonic scanning surroundings for empty space)
        # ----------------------------------------------------
        elif self.current_state == self.STATE_AVOIDING:
            if self.active_command != config.CMD_STOP:
                self._dispatch_command(config.CMD_STOP)

            # Wait for servo look-around sweep to refresh left and right clearances
            if elapsed >= self.action_duration:
                # Compare clearances: Left sector vs Right sector
                if self.left_dist > self.right_dist + 5.0 and self.left_dist > self.warning_distance_cm:
                    self.turn_direction = config.CMD_LEFT
                    turn_name = "LEFT"
                    open_space = self.left_dist
                elif self.right_dist > self.left_dist + 5.0 and self.right_dist > self.warning_distance_cm:
                    self.turn_direction = config.CMD_RIGHT
                    turn_name = "RIGHT"
                    open_space = self.right_dist
                elif self.left_dist >= self.right_dist and self.left_dist > self.critical_distance_cm:
                    self.turn_direction = config.CMD_LEFT
                    turn_name = "LEFT"
                    open_space = self.left_dist
                elif self.right_dist > self.critical_distance_cm:
                    self.turn_direction = config.CMD_RIGHT
                    turn_name = "RIGHT"
                    open_space = self.right_dist
                else:
                    # Both sides blocked: Reverse to create room, then re-scan
                    self._set_state(self.STATE_BACKING)
                    self.action_start_time = now
                    self.action_duration = 0.6
                    self._dispatch_command(config.CMD_BACKWARD, speed=210)
                    self.status_message.emit("Both sides blocked! Reversing to create clearance...")
                    return

                # Divert towards the open space!
                self._set_state(self.STATE_TURNING)
                self.action_start_time = now
                self.action_duration = 0.55  # ~75° pivot turn
                self._dispatch_command(self.turn_direction, speed=210)
                self.status_message.emit(f"Open space found on {turn_name} ({open_space:.0f}cm). Diverting route...")

        # ----------------------------------------------------
        # STATE: BACKING (Reversing slightly to disengage from obstacle)
        # ----------------------------------------------------
        elif self.current_state == self.STATE_BACKING:
            if elapsed >= self.action_duration:
                # After backing up, halt and scan surroundings for open space
                self._trigger_avoidance()

        # ----------------------------------------------------
        # STATE: TURNING (Pivoting towards maximum open space)
        # ----------------------------------------------------
        elif self.current_state == self.STATE_TURNING:
            if elapsed >= self.action_duration:
                # Finished turn -> Pause, rescan surroundings
                self._set_state(self.STATE_SCANNING)
                self.action_start_time = now
                self.action_duration = 0.6
                self._dispatch_command(config.CMD_STOP)
                self.status_message.emit("Turn completed. Rescanning sector...")

        # ----------------------------------------------------
        # STATE: HOMING (Return-to-Home: driving back to origin 0, 0)
        # ----------------------------------------------------
        elif self.current_state == self.STATE_HOMING:
            rx, ry, heading = self.mapping_engine.get_robot_pose()
            dx = 0.0 - rx
            dy = 0.0 - ry
            dist = math.sqrt(dx * dx + dy * dy)

            if dist <= 12.0:
                # Arrived at starting position!
                self._dispatch_command(config.CMD_STOP)
                self._set_state(self.STATE_IDLE)
                self.is_active = False
                self.status_message.emit("🎉 Arrived at starting position (0, 0)! Robot parked.")
                return

            # Target angle in world coordinates:
            target_deg = math.degrees(math.atan2(dy, dx)) % 360.0
            ang_diff = (target_deg - heading) % 360.0
            if ang_diff > 180.0:
                ang_diff -= 360.0

            # 1. Align heading towards start position if angle error > 25°
            if abs(ang_diff) > 25.0:
                turn_cmd = config.CMD_LEFT if ang_diff > 0 else config.CMD_RIGHT
                self._dispatch_command(turn_cmd, speed=200)
                self.status_message.emit(f"Homing: Aligning to start ({ang_diff:+.0f}°) | {dist:.0f}cm away")
                return

            # 2. Check front obstacle clearance before moving forward
            effective_center = min(self.center_dist, self.current_dist if abs(self.current_angle - 90) <= 25 else self.center_dist)
            if effective_center < self.critical_distance_cm:
                self._set_state(self.STATE_BACKING)
                self.action_start_time = now
                self.action_duration = 0.5
                self._dispatch_command(config.CMD_BACKWARD, speed=210)
                self.status_message.emit("Homing path blocked! Reversing...")
                return

            if effective_center < self.warning_distance_cm:
                self._trigger_avoidance()
                return

            # 3. Drive forward towards home
            self._dispatch_command(config.CMD_FORWARD, speed=200)
            self.status_message.emit(f"Homing: Driving to start ({dist:.1f} cm remaining)")

    def _trigger_avoidance(self):
        """Halts the robot and activates look-around scanning to find open space to divert."""
        self._dispatch_command(config.CMD_STOP)
        self._set_state(self.STATE_AVOIDING)
        self.action_start_time = time.time()
        self.action_duration = 1.0  # Allow 1.0s for the servo to sweep across left & right
        self.status_message.emit("Obstacle detected! Scanning left & right for open space...")


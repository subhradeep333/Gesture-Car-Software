"""
Autonomous Exploration & Navigation Controller for ESP32 AI Car.
Coordinates systematic area exploration, servo scanning, obstacle avoidance,
and dead-reckoning odometry synchronization with the 2D mapping engine.
"""

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

    def __init__(self, mapping_engine, websocket_manager, car_controller=None):
        super().__init__()
        self.mapping_engine = mapping_engine
        self.websocket_manager = websocket_manager
        self.car_controller = car_controller

        self.current_mode = self.MODE_MANUAL
        self.current_state = self.STATE_IDLE
        self.is_active = False

        # Navigation decision parameters (cm)
        self.safe_distance_cm = 30.0        # Min clearance to move forward
        self.warning_distance_cm = 22.0     # Obstacle caution threshold
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
        # STATE: BACKING (Reversing slightly to disengage from obstacle)
        # ----------------------------------------------------
        elif self.current_state == self.STATE_BACKING:
            if elapsed >= self.action_duration:
                # After backing up, halt and survey again
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

    def _trigger_avoidance(self):
        """Evaluates sector clearances and initiates pivot turn towards most open space."""
        self._dispatch_command(config.CMD_STOP)

        # Compare clearance: Left sector vs Right sector
        if self.left_dist > self.right_dist + 5.0 and self.left_dist > 25.0:
            self.turn_direction = config.CMD_LEFT
            turn_name = "LEFT"
        elif self.right_dist > self.left_dist + 5.0 and self.right_dist > 25.0:
            self.turn_direction = config.CMD_RIGHT
            turn_name = "RIGHT"
        elif self.left_dist >= self.right_dist:
            self.turn_direction = config.CMD_LEFT
            turn_name = "LEFT"
        else:
            self.turn_direction = config.CMD_RIGHT
            turn_name = "RIGHT"

        self._set_state(self.STATE_TURNING)
        self.action_start_time = time.time()
        # Rotate for ~550ms (~65° - 80° pivot)
        self.action_duration = 0.55
        self._dispatch_command(self.turn_direction, speed=210)
        self.status_message.emit(f"Obstacle ahead. Turning {turn_name} toward open space...")


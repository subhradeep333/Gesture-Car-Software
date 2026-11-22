"""
2D Occupancy Grid Mapping & Robot Odometry Engine for ESP32 AI Car.
Implements probabilistic / log-odds occupancy grid mapping with HC-SR04 ultrasonic
acoustic cone raycasting and dead-reckoning kinematic odometry tracking.
"""

import math
import time
import numpy as np
from PySide6.QtCore import QObject, Signal

class MappingEngine(QObject):
    # Signals for UI notifications
    map_updated = Signal()
    pose_updated = Signal(float, float, float)  # (x_cm, y_cm, heading_deg)
    obstacle_detected = Signal(float, float, float)  # (x_cm, y_cm, dist_cm)

    def __init__(self, width_m=6.0, height_m=6.0, resolution_cm=4.0):
        super().__init__()
        self.width_m = width_m
        self.height_m = height_m
        self.resolution_cm = resolution_cm

        # Grid dimensions in cells
        self.grid_width = int((width_m * 100.0) / resolution_cm)
        self.grid_height = int((height_m * 100.0) / resolution_cm)

        # Center origin in grid cell coordinates (0, 0 in cm corresponds to grid center)
        self.origin_x = self.grid_width // 2
        self.origin_y = self.grid_height // 2

        # Occupancy probabilities: 0.5 = Unknown, 0.0 = Completely Free, 1.0 = Fully Occupied
        # Stored as float32 matrix
        self.grid = np.full((self.grid_height, self.grid_width), 0.5, dtype=np.float32)

        # Log-odds probability constants for Bayes update
        # P_free = 0.35 -> l_free ~ -0.62
        # P_occ = 0.85  -> l_occ ~ +1.73
        self.L_FREE = -0.45
        self.L_OCC = 1.6   # Elevated confidence so detected objects appear promptly
        self.L_MIN = -4.0  # Cap free space confidence
        self.L_MAX = 4.0   # Cap obstacle confidence
        self.log_odds_grid = np.zeros((self.grid_height, self.grid_width), dtype=np.float32)

        # Active detected obstacle landmarks list: dicts with {x, y, dist, angle, time, hits}
        self.detected_obstacles = []

        # Robot Pose: x (cm), y (cm), heading (degrees: 90 = facing up along +Y)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_heading = 90.0  # Start facing North (+Y)

        # Kinematic Motion Parameters (Calibrated for ESP32 DC Motors @ PWM 180)
        self.linear_speed_cms = 30.0    # cm/sec forward/backward
        self.angular_speed_degs = 120.0 # deg/sec rotational turn speed
        self.last_odometry_time = time.time()
        self.current_movement_cmd = 'S'

        # Odometry trajectory history: List of (x, y) tuples
        self.path_history = [(0.0, 0.0)]
        self.max_path_points = 2000

        # Sensor physical offset from car center (cm along car forward axis)
        self.sensor_offset_front_cm = 8.0
        self.max_ultrasonic_range_cm = 200.0

        # Ultrasonic beam divergence model (HC-SR04 ~15° cone)
        self.acoustic_cone_half_deg = 5.0

        # Explored cell counts cache
        self.explored_cells_count = 0

    def reset_map(self):
        """Clears the 2D occupancy grid, robot path, and resets pose to origin."""
        self.grid.fill(0.5)
        self.log_odds_grid.fill(0.0)
        self.detected_obstacles.clear()
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_heading = 90.0
        self.path_history = [(0.0, 0.0)]
        self.last_odometry_time = time.time()
        self.current_movement_cmd = 'S'
        self.explored_cells_count = 0
        self.pose_updated.emit(self.robot_x, self.robot_y, self.robot_heading)
        self.map_updated.emit()

    def update_odometry(self, cmd_char, dt=None):
        """
        Updates estimated robot pose via dead-reckoning kinematics based on active command.
        """
        now = time.time()
        if dt is None:
            dt = now - self.last_odometry_time
        self.last_odometry_time = now

        # Clamp dt to avoid huge jumps on pauses
        dt = max(0.001, min(0.5, dt))
        self.current_movement_cmd = cmd_char

        moved = False
        rad = math.radians(self.robot_heading)

        if cmd_char == 'F':
            dist = self.linear_speed_cms * dt
            self.robot_x += dist * math.cos(rad)
            self.robot_y += dist * math.sin(rad)
            moved = True
        elif cmd_char == 'B':
            dist = self.linear_speed_cms * dt
            self.robot_x -= dist * math.cos(rad)
            self.robot_y -= dist * math.sin(rad)
            moved = True
        elif cmd_char == 'L':
            # Skid steer turn Left: increases heading angle (counter-clockwise)
            self.robot_heading = (self.robot_heading + self.angular_speed_degs * dt) % 360.0
            moved = True
        elif cmd_char == 'R':
            # Skid steer turn Right: decreases heading angle (clockwise)
            self.robot_heading = (self.robot_heading - self.angular_speed_degs * dt) % 360.0
            moved = True

        # Clamp robot to world boundary limits
        max_dist_cm = (self.width_m * 100.0 / 2.0) - 20.0
        self.robot_x = max(-max_dist_cm, min(max_dist_cm, self.robot_x))
        self.robot_y = max(-max_dist_cm, min(max_dist_cm, self.robot_y))

        if moved:
            # Append to trajectory history if moved significantly (> 2cm or > 5 deg)
            last_pt = self.path_history[-1]
            if (self.robot_x - last_pt[0])**2 + (self.robot_y - last_pt[1])**2 > 4.0:
                self.path_history.append((self.robot_x, self.robot_y))
                if len(self.path_history) > self.max_path_points:
                    self.path_history.pop(0)

            self.pose_updated.emit(self.robot_x, self.robot_y, self.robot_heading)

    def world_to_grid(self, x_cm, y_cm):
        """Converts world coordinates (cm from start) to grid matrix indices (col, row)."""
        col = int(self.origin_x + (x_cm / self.resolution_cm))
        row = int(self.origin_y - (y_cm / self.resolution_cm))  # Invert Y so +Y is up
        return col, row

    def grid_to_world(self, col, row):
        """Converts grid matrix indices (col, row) to world coordinates (cm)."""
        x_cm = (col - self.origin_x) * self.resolution_cm
        y_cm = (self.origin_y - row) * self.resolution_cm
        return x_cm, y_cm

    def _bresenham_line(self, x0, y0, x1, y1):
        """Fast Bresenham's line algorithm returning list of grid cell coordinates."""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        curr_x, curr_y = x0, y0
        while True:
            points.append((curr_x, curr_y))
            if curr_x == x1 and curr_y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                curr_x += sx
            if e2 < dx:
                err += dx
                curr_y += sy
        return points

    def add_sensor_reading(self, servo_angle_deg, distance_cm):
        """
        Integrates an ultrasonic distance measurement at a given servo angle into the occupancy grid.
        Models the HC-SR04 acoustic cone dispersion across ~15° (center beam + flanks).
        servo_angle_deg: 90 = facing car forward, 40 = right, 140 = left.
        distance_cm: measured distance in cm.
        """
        # Validate reading
        if distance_cm <= 1.0:
            return

        # Calculate sensor origin in world frame (mounted slightly forward on car chassis)
        car_rad = math.radians(self.robot_heading)
        sensor_world_x = self.robot_x + self.sensor_offset_front_cm * math.cos(car_rad)
        sensor_world_y = self.robot_y + self.sensor_offset_front_cm * math.sin(car_rad)

        # Beam direction in world frame:
        # Servo 90° is collinear with car_heading.
        # Servo > 90° is scanning left (+ angle), < 90° is scanning right (- angle)
        beam_angle_deg = self.robot_heading + (servo_angle_deg - 90.0)

        has_obstacle = (distance_cm < self.max_ultrasonic_range_cm - 2.0)
        effective_dist = min(distance_cm, self.max_ultrasonic_range_cm)

        # Center obstacle endpoint in world space
        center_rad = math.radians(beam_angle_deg)
        obs_world_x = sensor_world_x + effective_dist * math.cos(center_rad)
        obs_world_y = sensor_world_y + effective_dist * math.sin(center_rad)

        # Start grid coords
        start_col, start_row = self.world_to_grid(sensor_world_x, sensor_world_y)
        if not (0 <= start_col < self.grid_width and 0 <= start_row < self.grid_height):
            return

        # Ultrasonic Acoustic Cone: sample center beam and two lateral flank rays
        cone_offsets = [0.0]
        if has_obstacle and effective_dist > 25.0:
            # Model acoustic wavefront arc dispersion
            cone_offsets = [-self.acoustic_cone_half_deg, 0.0, self.acoustic_cone_half_deg]

        min_col, max_col = start_col, start_col
        min_row, max_row = start_row, start_row

        for delta_ang in cone_offsets:
            ang_rad = math.radians(beam_angle_deg + delta_ang)
            end_wx = sensor_world_x + effective_dist * math.cos(ang_rad)
            end_wy = sensor_world_y + effective_dist * math.sin(ang_rad)

            end_col, end_row = self.world_to_grid(end_wx, end_wy)
            min_col = min(min_col, end_col)
            max_col = max(max_col, end_col)
            min_row = min(min_row, end_row)
            max_row = max(max_row, end_row)

            line_cells = self._bresenham_line(start_col, start_row, end_col, end_row)
            if not line_cells:
                continue

            # Free space update along beam path
            free_cells = line_cells[:-1] if has_obstacle else line_cells
            l_free_weighted = self.L_FREE if delta_ang == 0.0 else self.L_FREE * 0.70
            for c, r in free_cells:
                if 0 <= c < self.grid_width and 0 <= r < self.grid_height:
                    self.log_odds_grid[r, c] = max(self.L_MIN, self.log_odds_grid[r, c] + l_free_weighted)

            # Occupied endpoint update across acoustic wavefront arc
            if has_obstacle:
                weight = 1.0 if delta_ang == 0.0 else 0.65
                l_occ_weighted = self.L_OCC * weight

                oc, or_ = line_cells[-1]
                obs_cells = [(oc, or_)]
                # Thicken endpoint slightly for robust surface continuity
                for dc in [-1, 0, 1]:
                    for dr in [-1, 0, 1]:
                        if abs(dc) + abs(dr) <= 1:
                            obs_cells.append((oc + dc, or_ + dr))

                for c, r in obs_cells:
                    if 0 <= c < self.grid_width and 0 <= r < self.grid_height:
                        self.log_odds_grid[r, c] = min(self.L_MAX, self.log_odds_grid[r, c] + l_occ_weighted)

        # Active obstacle landmark tracking in world space
        if has_obstacle and distance_cm <= 195.0:
            now = time.time()
            found = False
            for obs in self.detected_obstacles:
                if math.hypot(obs["x"] - obs_world_x, obs["y"] - obs_world_y) < 12.0:
                    obs["x"] = 0.6 * obs["x"] + 0.4 * obs_world_x
                    obs["y"] = 0.6 * obs["y"] + 0.4 * obs_world_y
                    obs["dist"] = distance_cm
                    obs["angle"] = servo_angle_deg
                    obs["time"] = now
                    obs["hits"] = obs.get("hits", 1) + 1
                    found = True
                    break
            if not found:
                self.detected_obstacles.append({
                    "x": obs_world_x,
                    "y": obs_world_y,
                    "dist": distance_cm,
                    "angle": servo_angle_deg,
                    "time": now,
                    "hits": 1
                })
                if len(self.detected_obstacles) > 250:
                    self.detected_obstacles.pop(0)

            self.obstacle_detected.emit(obs_world_x, obs_world_y, distance_cm)

        # Bounding box probability conversion: P = 1 / (1 + exp(-L))
        c_min = max(0, min_col - 2)
        c_max = min(self.grid_width, max_col + 3)
        r_min = max(0, min_row - 2)
        r_max = min(self.grid_height, max_row + 3)

        sub_l = self.log_odds_grid[r_min:r_max, c_min:c_max]
        self.grid[r_min:r_max, c_min:c_max] = 1.0 / (1.0 + np.exp(-sub_l))

        self.map_updated.emit()

    def get_explored_stats(self):
        """
        Returns mapping exploration statistics:
        (explored_cells, total_cells, coverage_percent, explored_area_m2)
        """
        # Cells that differ significantly from prior 0.5 (free < 0.45 or occupied > 0.55)
        explored_mask = (self.grid < 0.45) | (self.grid > 0.55)
        explored_count = int(np.count_nonzero(explored_mask))
        total_cells = self.grid_width * self.grid_height

        pct = (explored_count / total_cells) * 100.0 if total_cells > 0 else 0.0
        cell_area_m2 = (self.resolution_cm / 100.0) ** 2
        area_m2 = explored_count * cell_area_m2

        return explored_count, total_cells, pct, area_m2

    def get_robot_pose(self):
        """Returns (x_cm, y_cm, heading_deg)."""
        return self.robot_x, self.robot_y, self.robot_heading

    def get_detected_obstacles(self):
        """Returns list of active detected obstacle objects with world coordinates and distances."""
        return list(self.detected_obstacles)

    def refine_map_with_ml(self):
        """
        Runs the ultrasonic ML spatial perception model on the active 2D occupancy grid
        to extract continuous wall contours, obstacle clusters, and safe corridors.
        """
        from map_ml_processor import MapMLProcessor
        proc = MapMLProcessor(self.resolution_cm)
        return proc.refine_map(self.grid, self.resolution_cm, self.origin_x, self.origin_y)

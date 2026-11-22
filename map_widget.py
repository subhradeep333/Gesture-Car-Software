"""
High-Performance Interactive 2D Occupancy Grid Map Visualization Widget.
Features:
- Real-time grid rendering with laser / ultrasonic occupancy probabilities.
- ML Perception Layer: Vectorized wall segments, obstacle clusters, and safe corridors.
- Interactive Route Drawing Tool: Click & drag to set navigation waypoints.
- Live Autonomous Waypoint Tracking with dynamic beacon highlights.
- Smooth mouse drag pan & mouse wheel zoom with Recenter (🎯) control.
"""

import math
import time
import numpy as np
from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QColor, QPen, QBrush, QFont, QPolygonF
from PySide6.QtCore import Qt, QPointF, QRectF, Signal


class Map2DWidget(QWidget):
    # Signal emitted when user modifies the drawn route
    route_updated = Signal(list)  # List of (x_cm, y_cm)

    def __init__(self, mapping_engine=None):
        super().__init__()
        self.mapping_engine = mapping_engine
        self.setMinimumSize(420, 420)
        self.setMouseTracking(True)

        # Pan & Zoom parameters
        self.zoom = 1.0           # 1.0 = Default scale (pixels per cm)
        self.min_zoom = 0.4
        self.max_zoom = 3.5
        self.pan_offset_x = 0.0   # Screen pixel pan offset
        self.pan_offset_y = 0.0
        self.is_dragging = False
        self.last_mouse_pos = QPointF(0, 0)

        # Live sensor ray cache for drawing the sweeping beam
        self.current_servo_angle = 90
        self.current_sensor_dist = 200.0

        # Interactive Route Drawing Mode
        self.draw_route_mode = False
        self.drawn_route = []            # List of (x_cm, y_cm) world waypoints
        self.active_waypoint_idx = 0     # Index of currently pursued waypoint
        self.min_draw_spacing_cm = 16.0  # Minimum spacing between drag-drawn waypoints

        # Machine Learning Spatial Perception Layer
        self.ml_map_data = None
        self.show_ml_overlay = True

        if self.mapping_engine:
            self.mapping_engine.map_updated.connect(self.update)
            self.mapping_engine.pose_updated.connect(self._on_pose_updated)

    def _on_pose_updated(self, x, y, heading):
        self.update()

    def set_sensor_reading(self, servo_angle, distance_cm):
        """Updates active beam overlay."""
        self.current_servo_angle = servo_angle
        self.current_sensor_dist = distance_cm
        self.update()

    def zoom_in(self):
        self.zoom = min(self.max_zoom, self.zoom * 1.25)
        self.update()

    def zoom_out(self):
        self.zoom = max(self.min_zoom, self.zoom / 1.25)
        self.update()

    def recenter_robot(self):
        """Resets pan offset so robot position is centered in viewport."""
        self.pan_offset_x = 0.0
        self.pan_offset_y = 0.0
        self.zoom = 1.0
        self.update()

    def screen_to_world(self, sx, sy, w, h):
        """Converts screen pixel coordinates to world coordinates (cm from start)."""
        view_cx = (w / 2.0) + self.pan_offset_x
        view_cy = (h / 2.0) + self.pan_offset_y
        px_per_cm = 0.85 * self.zoom

        world_x = (sx - view_cx) / px_per_cm
        world_y = (view_cy - sy) / px_per_cm
        return world_x, world_y

    def world_to_screen(self, world_x, world_y, w, h):
        """Converts world coordinates (cm) to screen pixel coordinates."""
        view_cx = (w / 2.0) + self.pan_offset_x
        view_cy = (h / 2.0) + self.pan_offset_y
        px_per_cm = 0.85 * self.zoom

        sx = view_cx + (world_x * px_per_cm)
        sy = view_cy - (world_y * px_per_cm)
        return sx, sy

    # ----------------------------------------------------
    # ROUTE DRAWING & EDITING API
    # ----------------------------------------------------
    def set_draw_route_mode(self, enabled: bool):
        """Enables or disables interactive mouse route drawing mode."""
        self.draw_route_mode = enabled
        self.setCursor(Qt.CrossCursor if enabled else Qt.ArrowCursor)
        self.update()

    def clear_drawn_route(self):
        """Clears all drawn waypoints."""
        self.drawn_route.clear()
        self.active_waypoint_idx = 0
        self.route_updated.emit(self.drawn_route)
        self.update()

    def get_drawn_route(self):
        """Returns list of (x_cm, y_cm) waypoints."""
        return list(self.drawn_route)

    def set_active_waypoint_idx(self, idx: int):
        """Sets the current waypoint target index."""
        self.active_waypoint_idx = idx
        self.update()

    def set_ml_map_overlay(self, ml_data: dict):
        """Sets the machine learning processed map data for overlay."""
        self.ml_map_data = ml_data
        self.update()

    def toggle_ml_overlay(self):
        """Toggles ML map refinement visibility."""
        self.show_ml_overlay = not self.show_ml_overlay
        self.update()
        return self.show_ml_overlay

    def _add_waypoint(self, wx, wy):
        if self.mapping_engine:
            max_d = (self.mapping_engine.width_m * 100.0 / 2.0) - 20.0
            wx = max(-max_d, min(max_d, wx))
            wy = max(-max_d, min(max_d, wy))
        self.drawn_route.append((round(wx, 1), round(wy, 1)))
        self.route_updated.emit(self.drawn_route)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            if self.draw_route_mode:
                pos = event.position()
                wx, wy = self.screen_to_world(pos.x(), pos.y(), self.width(), self.height())
                self._add_waypoint(wx, wy)
                self.update()
            else:
                self.is_dragging = True
                self.last_mouse_pos = event.position()

    def mouseMoveEvent(self, event):
        if self.draw_route_mode:
            if event.buttons() & Qt.LeftButton:
                pos = event.position()
                wx, wy = self.screen_to_world(pos.x(), pos.y(), self.width(), self.height())
                if self.drawn_route:
                    last_wx, last_wy = self.drawn_route[-1]
                    dist_sq = (wx - last_wx)**2 + (wy - last_wy)**2
                    if dist_sq >= (self.min_draw_spacing_cm ** 2):
                        self._add_waypoint(wx, wy)
                        self.update()
                else:
                    self._add_waypoint(wx, wy)
                    self.update()
        else:
            if self.is_dragging:
                delta = event.position() - self.last_mouse_pos
                self.pan_offset_x += delta.x()
                self.pan_offset_y += delta.y()
                self.last_mouse_pos = event.position()
                self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.is_dragging = False

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        factor = 1.15 if delta > 0 else 0.87
        self.zoom = max(self.min_zoom, min(self.max_zoom, self.zoom * factor))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()

        # Fill background (Deep Cosmic Slate)
        painter.fillRect(self.rect(), QColor("#080C14"))

        if not self.mapping_engine:
            return

        # Viewport center in widget coordinates
        view_cx = (w / 2.0) + self.pan_offset_x
        view_cy = (h / 2.0) + self.pan_offset_y

        # Scale factor: pixels per centimeter in world
        px_per_cm = 0.85 * self.zoom

        # ----------------------------------------------------
        # 1. DRAW METRIC GRID LINES (Every 50cm & 100cm)
        # ----------------------------------------------------
        grid_step_cm = 50.0
        max_extent_cm = (self.mapping_engine.width_m * 100.0) / 2.0

        pen_subgrid = QPen(QColor("#131C2B"), 1, Qt.DashLine)
        pen_axis = QPen(QColor("#1F2C42"), 1, Qt.SolidLine)
        font_grid = QFont("Segoe UI", 7, QFont.Bold)
        painter.setFont(font_grid)

        steps = int(max_extent_cm / grid_step_cm)
        for s in range(-steps, steps + 1):
            val_cm = s * grid_step_cm
            screen_x = view_cx + (val_cm * px_per_cm)
            screen_y = view_cy - (val_cm * px_per_cm)

            if 0 <= screen_x <= w:
                painter.setPen(pen_axis if s == 0 else pen_subgrid)
                painter.drawLine(QPointF(screen_x, 0), QPointF(screen_x, h))
                if s != 0 and s % 2 == 0:
                    painter.setPen(QColor("#475569"))
                    painter.drawText(int(screen_x + 3), h - 8, f"{val_cm/100:.0f}m")

            if 0 <= screen_y <= h:
                painter.setPen(pen_axis if s == 0 else pen_subgrid)
                painter.drawLine(QPointF(0, screen_y), QPointF(w, screen_y))
                if s != 0 and s % 2 == 0:
                    painter.setPen(QColor("#475569"))
                    painter.drawText(8, int(screen_y - 3), f"{val_cm/100:.0f}m")

        # ----------------------------------------------------
        # 2. DRAW 2D OCCUPANCY GRID CELLS
        # ----------------------------------------------------
        grid = self.mapping_engine.grid
        res = self.mapping_engine.resolution_cm
        gh, gw = grid.shape
        orig_x, orig_y = self.mapping_engine.origin_x, self.mapping_engine.origin_y

        cell_px = res * px_per_cm

        occupied_mask = (grid > 0.65)
        free_mask = (grid < 0.42)

        # Draw Free Space (Soft dark cyan-blue tint)
        free_rows, free_cols = np.where(free_mask)
        if len(free_rows) > 0:
            free_brush = QBrush(QColor(16, 42, 66, 150))
            painter.setBrush(free_brush)
            painter.setPen(Qt.NoPen)
            for r, c in zip(free_rows, free_cols):
                world_x = (c - orig_x) * res
                world_y = (orig_y - r) * res
                sx = view_cx + (world_x * px_per_cm)
                sy = view_cy - (world_y * px_per_cm)
                if -20 <= sx <= w + 20 and -20 <= sy <= h + 20:
                    painter.drawRect(QRectF(sx, sy, cell_px, cell_px))

        # Draw Raw Occupied Obstacles (High-Visibility Hazard Red/Orange Blocks)
        occ_rows, occ_cols = np.where(occupied_mask)
        if len(occ_rows) > 0:
            occ_brush = QBrush(QColor(239, 68, 68, 220))
            painter.setBrush(occ_brush)
            painter.setPen(QColor(255, 107, 107, 120))
            for r, c in zip(occ_rows, occ_cols):
                world_x = (c - orig_x) * res
                world_y = (orig_y - r) * res
                sx = view_cx + (world_x * px_per_cm)
                sy = view_cy - (world_y * px_per_cm)
                if -20 <= sx <= w + 20 and -20 <= sy <= h + 20:
                    painter.drawRoundedRect(QRectF(sx - 1, sy - 1, cell_px + 2, cell_px + 2), 2, 2)

        # ----------------------------------------------------
        # 2B. DRAW DETECTED OBSTACLES & LIVE HAZARD PINS
        # ----------------------------------------------------
        now = time.time()
        obstacles = getattr(self.mapping_engine, "detected_obstacles", [])
        if len(obstacles) > 0:
            font_obs = QFont("Segoe UI", 8, QFont.Bold)
            painter.setFont(font_obs)

            for obs in obstacles:
                wx, wy = obs["x"], obs["y"]
                d_cm = obs["dist"]
                age = now - obs.get("time", now)

                sx = view_cx + (wx * px_per_cm)
                sy = view_cy - (wy * px_per_cm)

                if -30 <= sx <= w + 30 and -30 <= sy <= h + 30:
                    # Pulsing hazard halo for fresh detections (< 3.0s)
                    if age < 3.0:
                        pulse = 0.5 + 0.5 * math.sin(now * 8.0)
                        pulse_r = 10.0 + (pulse * 8.0)
                        alpha = int(max(20, (1.0 - (age / 3.0)) * 140))
                        painter.setBrush(Qt.NoBrush)
                        painter.setPen(QPen(QColor(239, 68, 68, alpha), 2, Qt.DashLine))
                        painter.drawEllipse(QPointF(sx, sy), pulse_r, pulse_r)

                    # Solid Obstacle Beacon (Crimson Red for close, Amber for medium)
                    pin_col = QColor("#EF4444") if d_cm <= 25.0 else QColor("#F97316")
                    painter.setBrush(QBrush(pin_col))
                    painter.setPen(QPen(QColor("#FFFFFF"), 1.5))
                    painter.drawEllipse(QPointF(sx, sy), 6.0, 6.0)

                    # Dynamic Distance Callout Tag for Active Objects (< 5.0s)
                    if age < 5.0:
                        tag_txt = f"⚠️ {d_cm:.0f}cm"
                        painter.setPen(QColor("#000000"))
                        painter.drawText(int(sx - 15), int(sy - 9), tag_txt)  # Shadow
                        painter.setPen(QColor("#FF4D4D") if d_cm <= 25.0 else QColor("#FBBF24"))
                        painter.drawText(int(sx - 16), int(sy - 10), tag_txt)

        # ----------------------------------------------------
        # 3. DRAW ML SPATIAL MODEL LAYER (WALLS & CORRIDORS)
        # ----------------------------------------------------
        if self.show_ml_overlay and self.ml_map_data:
            # Draw ML Safe Corridors (soft emerald green overlay)
            corridor_pts = self.ml_map_data.get("corridor_points", [])
            if len(corridor_pts) > 0:
                painter.setPen(Qt.NoPen)
                painter.setBrush(QBrush(QColor(16, 185, 129, 45)))
                for wx, wy in corridor_pts:
                    sx = view_cx + (wx * px_per_cm)
                    sy = view_cy - (wy * px_per_cm)
                    if 0 <= sx <= w and 0 <= sy <= h:
                        painter.drawEllipse(QPointF(sx, sy), max(4.0, cell_px * 1.5), max(4.0, cell_px * 1.5))

            # Draw ML Vectorized Wall Segments (crisp glowing blueprint lines)
            wall_segments = self.ml_map_data.get("wall_segments", [])
            if len(wall_segments) > 0:
                pen_wall = QPen(QColor("#00E5FF"), 3, Qt.SolidLine)
                painter.setPen(pen_wall)
                for p1, p2 in wall_segments:
                    s1_x = view_cx + (p1[0] * px_per_cm)
                    s1_y = view_cy - (p1[1] * px_per_cm)
                    s2_x = view_cx + (p2[0] * px_per_cm)
                    s2_y = view_cy - (p2[1] * px_per_cm)
                    painter.drawLine(QPointF(s1_x, s1_y), QPointF(s2_x, s2_y))

        # ----------------------------------------------------
        # 4. DRAW USER-DRAWN NAVIGATION ROUTE & WAYPOINTS
        # ----------------------------------------------------
        if len(self.drawn_route) > 0:
            # 1. Draw glowing connecting route lines
            pen_route = QPen(QColor("#F59E0B"), 3, Qt.DashLine)
            painter.setPen(pen_route)
            for i in range(len(self.drawn_route) - 1):
                w1 = self.drawn_route[i]
                w2 = self.drawn_route[i + 1]
                p1_x = view_cx + (w1[0] * px_per_cm)
                p1_y = view_cy - (w1[1] * px_per_cm)
                p2_x = view_cx + (w2[0] * px_per_cm)
                p2_y = view_cy - (w2[1] * px_per_cm)
                painter.drawLine(QPointF(p1_x, p1_y), QPointF(p2_x, p2_y))

            # 2. Draw circular waypoint pins
            font_wp = QFont("Segoe UI", 8, QFont.Bold)
            painter.setFont(font_wp)

            for idx, (wx, wy) in enumerate(self.drawn_route):
                sx = view_cx + (wx * px_per_cm)
                sy = view_cy - (wy * px_per_cm)

                is_completed = idx < self.active_waypoint_idx
                is_current = idx == self.active_waypoint_idx
                is_future = idx > self.active_waypoint_idx

                if is_completed:
                    # Completed: Bright Green with checkmark
                    painter.setBrush(QBrush(QColor("#10B981")))
                    painter.setPen(QPen(QColor("#FFFFFF"), 1))
                    painter.drawEllipse(QPointF(sx, sy), 8, 8)
                    painter.setPen(QColor("#FFFFFF"))
                    painter.drawText(int(sx - 4), int(sy + 4), "✓")
                elif is_current:
                    # Active target: Electric Cyan with pulsing outer ring
                    painter.setBrush(QBrush(QColor(0, 229, 255, 60)))
                    painter.setPen(QPen(QColor("#00E5FF"), 2))
                    painter.drawEllipse(QPointF(sx, sy), 14, 14)

                    painter.setBrush(QBrush(QColor("#00E5FF")))
                    painter.setPen(Qt.NoPen)
                    painter.drawEllipse(QPointF(sx, sy), 8, 8)

                    painter.setPen(QColor("#000000"))
                    painter.drawText(int(sx - 4), int(sy + 4), str(idx + 1))
                else:
                    # Future waypoint: Slate with Amber border
                    painter.setBrush(QBrush(QColor("#1E293B")))
                    painter.setPen(QPen(QColor("#F59E0B"), 2))
                    painter.drawEllipse(QPointF(sx, sy), 8, 8)
                    painter.setPen(QColor("#F8FAFC"))
                    painter.drawText(int(sx - 4), int(sy + 4), str(idx + 1))

        # ----------------------------------------------------
        # 5. DRAW ROBOT TRAJECTORY (PATH HISTORY)
        # ----------------------------------------------------
        path = self.mapping_engine.path_history
        if len(path) > 1:
            pen_path = QPen(QColor("#10B981"), 2, Qt.SolidLine)
            painter.setPen(pen_path)
            for i in range(len(path) - 1):
                pt1 = path[i]
                pt2 = path[i + 1]
                p1_x = view_cx + (pt1[0] * px_per_cm)
                p1_y = view_cy - (pt1[1] * px_per_cm)
                p2_x = view_cx + (pt2[0] * px_per_cm)
                p2_y = view_cy - (pt2[1] * px_per_cm)
                painter.drawLine(QPointF(p1_x, p1_y), QPointF(p2_x, p2_y))

        # ----------------------------------------------------
        # 6. DRAW ROBOT CAR & HEADING ORIENTATION
        # ----------------------------------------------------
        rx, ry, r_heading = self.mapping_engine.get_robot_pose()
        car_screen_x = view_cx + (rx * px_per_cm)
        car_screen_y = view_cy - (ry * px_per_cm)

        painter.save()
        painter.translate(car_screen_x, car_screen_y)
        painter.rotate(-r_heading + 90.0)

        car_w_px = max(16.0, 15.0 * px_per_cm)
        car_l_px = max(24.0, 22.0 * px_per_cm)

        painter.setBrush(QBrush(QColor("#1E293B")))
        painter.setPen(QPen(QColor("#38BDF8"), 2))
        painter.drawRoundedRect(QRectF(-car_w_px / 2.0, -car_l_px / 2.0, car_w_px, car_l_px), 4, 4)

        # Headlights
        painter.setBrush(QBrush(QColor("#F59E0B")))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(QPointF(-car_w_px / 3.0, -car_l_px / 2.0 + 2), 3, 3)
        painter.drawEllipse(QPointF(car_w_px / 3.0, -car_l_px / 2.0 + 2), 3, 3)

        # Direction forward arrow
        painter.setBrush(QBrush(QColor("#00E5FF")))
        arrow = QPolygonF([
            QPointF(0, -car_l_px / 2.0 - 6),
            QPointF(-5, -car_l_px / 4.0),
            QPointF(5, -car_l_px / 4.0)
        ])
        painter.drawPolygon(arrow)

        # Active sweeping sensor beam emerging from front bumper
        sensor_y = -car_l_px / 2.0
        beam_angle_rel = (self.current_servo_angle - 90.0)
        beam_rad = math.radians(-beam_angle_rel - 90.0)

        beam_len_px = min(self.current_sensor_dist, 200.0) * px_per_cm
        beam_end_x = beam_len_px * math.cos(beam_rad)
        beam_end_y = beam_len_px * math.sin(beam_rad)

        is_hit = (self.current_sensor_dist < 195.0)
        col_beam = QColor(239, 68, 68, 220) if is_hit else QColor(0, 229, 255, 180)
        painter.setPen(QPen(col_beam, 2, Qt.SolidLine))
        painter.drawLine(QPointF(0, sensor_y), QPointF(beam_end_x, beam_end_y))

        if is_hit:
            col_hit = QColor("#EF4444") if self.current_sensor_dist <= 25.0 else QColor("#F59E0B")
            painter.setBrush(QBrush(col_hit))
            painter.setPen(QPen(QColor("#FFFFFF"), 1))
            painter.drawEllipse(QPointF(beam_end_x, beam_end_y), 6, 6)

            # Impact target reticle
            painter.setBrush(Qt.NoBrush)
            painter.setPen(QPen(col_hit, 1.5, Qt.DashLine))
            painter.drawEllipse(QPointF(beam_end_x, beam_end_y), 12, 12)

        painter.restore()

        # ----------------------------------------------------
        # 7. HUD OVERLAY: COMPASS, SCALE & ROUTE INFO
        # ----------------------------------------------------
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(15, 23, 42, 220)))
        painter.drawRoundedRect(12, 12, 260, 84, 8, 8)

        painter.setFont(QFont("Segoe UI", 8, QFont.Bold))
        painter.setPen(QColor("#00E5FF"))
        painter.drawText(22, 28, f"ROBOT POSE: X={rx:+.1f}cm  Y={ry:+.1f}cm")

        painter.setPen(QColor("#94A3B8"))
        painter.drawText(22, 44, f"HEADING: {r_heading:.0f}° | ULTRASONIC: {self.current_sensor_dist:.0f}cm")

        explored, total, pct, area_m2 = self.mapping_engine.get_explored_stats()
        n_obs = len(getattr(self.mapping_engine, "detected_obstacles", []))
        if n_obs > 0:
            painter.setPen(QColor("#EF4444"))
            painter.drawText(22, 60, f"MAPPED: {area_m2:.2f}m² | ⚠️ OBSTACLES: {n_obs} PLOTTED")
        else:
            painter.setPen(QColor("#10B981"))
            painter.drawText(22, 60, f"MAPPED AREA: {area_m2:.2f} m² ({pct:.1f}% space)")

        if self.draw_route_mode:
            painter.setPen(QColor("#F59E0B"))
            painter.drawText(22, 78, f"✏️ DRAW MODE ACTIVE: {len(self.drawn_route)} waypoints")
        elif len(self.drawn_route) > 0:
            painter.setPen(QColor("#38BDF8"))
            painter.drawText(22, 78, f"📍 ROUTE READY: {len(self.drawn_route)} waypoints (Target: #{self.active_waypoint_idx + 1})")
        else:
            painter.setPen(QColor("#64748B"))
            painter.drawText(22, 78, "Click '✏️ Draw Route' to plan path")

        # Metric Scale Bar (1 Meter)
        scale_len_px = 100.0 * px_per_cm
        scale_x = w - scale_len_px - 20
        scale_y = h - 16

        painter.setPen(QPen(QColor("#E2E8F0"), 2))
        painter.drawLine(QPointF(scale_x, scale_y), QPointF(scale_x + scale_len_px, scale_y))
        painter.drawLine(QPointF(scale_x, scale_y - 4), QPointF(scale_x, scale_y + 4))
        painter.drawLine(QPointF(scale_x + scale_len_px, scale_y - 4), QPointF(scale_x + scale_len_px, scale_y + 4))

        painter.setPen(QColor("#CBD5E1"))
        painter.setFont(QFont("Segoe UI", 8, QFont.Bold))
        painter.drawText(int(scale_x + scale_len_px / 2.0 - 20), int(scale_y - 6), "1.0 METER")

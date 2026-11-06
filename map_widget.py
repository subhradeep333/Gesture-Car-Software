"""
High-Performance Interactive 2D Occupancy Grid Map Visualization Widget.
Renders real-time grid cells, detected obstacles, robot trajectory, orientation,
active ultrasonic scanning beam, scale legend, and supports smooth pan & zoom.
"""

import math
import numpy as np
from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QColor, QPen, QBrush, QFont, QPolygonF
from PySide6.QtCore import Qt, QPointF, QRectF

class Map2DWidget(QWidget):
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

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.is_dragging = True
            self.last_mouse_pos = event.position()

    def mouseMoveEvent(self, event):
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

        # Fill background (Deep Cosmic Navy)
        painter.fillRect(self.rect(), QColor("#080C14"))

        if not self.mapping_engine:
            return

        # Viewport center in widget coordinates
        view_cx = (w / 2.0) + self.pan_offset_x
        view_cy = (h / 2.0) + self.pan_offset_y

        # Scale factor: pixels per centimeter in world
        # Base scale: 6m world fits comfortably in ~400px -> base ~ 0.7 pixels/cm
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

        # Vertical & horizontal grid lines
        steps = int(max_extent_cm / grid_step_cm)
        for s in range(-steps, steps + 1):
            val_cm = s * grid_step_cm
            screen_x = view_cx + (val_cm * px_per_cm)
            screen_y = view_cy - (val_cm * px_per_cm)

            # Vertical Line
            if 0 <= screen_x <= w:
                painter.setPen(pen_axis if s == 0 else pen_subgrid)
                painter.drawLine(QPointF(screen_x, 0), QPointF(screen_x, h))
                if s != 0 and s % 2 == 0:
                    painter.setPen(QColor("#475569"))
                    painter.drawText(int(screen_x + 3), h - 8, f"{val_cm/100:.0f}m")

            # Horizontal Line
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

        # Find cells that have been measured (Free < 0.45 or Occupied > 0.55)
        # Optimized rendering: sample only non-prior cells
        occupied_mask = (grid > 0.65)
        free_mask = (grid < 0.42)

        # Draw Free Space (Soft dark cyan-blue tint)
        free_rows, free_cols = np.where(free_mask)
        if len(free_rows) > 0:
            free_brush = QBrush(QColor(16, 42, 66, 160))
            painter.setBrush(free_brush)
            painter.setPen(Qt.NoPen)
            for r, c in zip(free_rows, free_cols):
                world_x = (c - orig_x) * res
                world_y = (orig_y - r) * res
                sx = view_cx + (world_x * px_per_cm)
                sy = view_cy - (world_y * px_per_cm)
                if -20 <= sx <= w + 20 and -20 <= sy <= h + 20:
                    painter.drawRect(QRectF(sx, sy, cell_px, cell_px))

        # Draw Occupied Obstacles / Walls (Bright glowing cyan/amber blocks)
        occ_rows, occ_cols = np.where(occupied_mask)
        if len(occ_rows) > 0:
            occ_brush = QBrush(QColor(0, 229, 255, 235))
            painter.setBrush(occ_brush)
            painter.setPen(QColor(0, 229, 255, 100))
            for r, c in zip(occ_rows, occ_cols):
                world_x = (c - orig_x) * res
                world_y = (orig_y - r) * res
                sx = view_cx + (world_x * px_per_cm)
                sy = view_cy - (world_y * px_per_cm)
                if -20 <= sx <= w + 20 and -20 <= sy <= h + 20:
                    painter.drawRoundedRect(QRectF(sx - 1, sy - 1, cell_px + 2, cell_px + 2), 2, 2)

        # ----------------------------------------------------
        # 3. DRAW ROBOT TRAJECTORY (PATH HISTORY)
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
        # 4. DRAW ROBOT CAR & HEADING ORIENTATION
        # ----------------------------------------------------
        rx, ry, r_heading = self.mapping_engine.get_robot_pose()
        car_screen_x = view_cx + (rx * px_per_cm)
        car_screen_y = view_cy - (ry * px_per_cm)

        painter.save()
        painter.translate(car_screen_x, car_screen_y)
        painter.rotate(-r_heading + 90.0)  # Convert heading (0° East, 90° North) to Qt rotation

        # Car Chassis: 22cm length x 15cm width scaled to pixels
        car_w_px = max(16.0, 15.0 * px_per_cm)
        car_l_px = max(24.0, 22.0 * px_per_cm)

        # Car body
        painter.setBrush(QBrush(QColor("#1E293B")))
        painter.setPen(QPen(QColor("#38BDF8"), 2))
        painter.drawRoundedRect(QRectF(-car_w_px / 2.0, -car_l_px / 2.0, car_w_px, car_l_px), 4, 4)

        # Headlights / Front bumper indicator
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
        beam_angle_rel = (self.current_servo_angle - 90.0)  # Relative to car heading
        beam_rad = math.radians(-beam_angle_rel - 90.0)

        beam_len_px = min(self.current_sensor_dist, 200.0) * px_per_cm
        beam_end_x = beam_len_px * math.cos(beam_rad)
        beam_end_y = beam_len_px * math.sin(beam_rad)

        # Beam ray
        painter.setPen(QPen(QColor(0, 229, 255, 200), 2, Qt.SolidLine))
        painter.drawLine(QPointF(0, sensor_y), QPointF(beam_end_x, beam_end_y))

        # Reflection hit point
        if self.current_sensor_dist < 195.0:
            col_hit = QColor("#EF4444") if self.current_sensor_dist <= 15.0 else QColor("#F59E0B")
            painter.setBrush(QBrush(col_hit))
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(QPointF(beam_end_x, beam_end_y), 5, 5)

        painter.restore()

        # ----------------------------------------------------
        # 5. HUD OVERLAY: COMPASS, SCALE & COVERAGE INFO
        # ----------------------------------------------------
        # Top-Left: Mode & Coordinates Badge
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(15, 23, 42, 220)))
        painter.drawRoundedRect(12, 12, 230, 68, 8, 8)

        painter.setFont(QFont("Segoe UI", 8, QFont.Bold))
        painter.setPen(QColor("#00E5FF"))
        painter.drawText(22, 30, f"ROBOT POSE: X={rx:+.1f}cm  Y={ry:+.1f}cm")

        painter.setPen(QColor("#94A3B8"))
        painter.drawText(22, 48, f"HEADING: {r_heading:.0f}° | SERVO: {self.current_servo_angle}°")

        explored, total, pct, area_m2 = self.mapping_engine.get_explored_stats()
        painter.setPen(QColor("#10B981"))
        painter.drawText(22, 66, f"MAPPED AREA: {area_m2:.2f} m² ({pct:.1f}% space)")

        # Bottom-Right: Metric Scale Bar (1 Meter)
        scale_len_px = 100.0 * px_per_cm  # 1 meter
        scale_x = w - scale_len_px - 20
        scale_y = h - 16

        painter.setPen(QPen(QColor("#E2E8F0"), 2))
        painter.drawLine(QPointF(scale_x, scale_y), QPointF(scale_x + scale_len_px, scale_y))
        painter.drawLine(QPointF(scale_x, scale_y - 4), QPointF(scale_x, scale_y + 4))
        painter.drawLine(QPointF(scale_x + scale_len_px, scale_y - 4), QPointF(scale_x + scale_len_px, scale_y + 4))

        painter.setPen(QColor("#CBD5E1"))
        painter.setFont(QFont("Segoe UI", 8, QFont.Bold))
        painter.drawText(int(scale_x + scale_len_px / 2.0 - 20), int(scale_y - 6), "1.0 METER")

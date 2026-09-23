"""
Modern PySide6 (Qt6) Desktop Command Dashboard for ESP32 AI Hand Gesture Controlled IoT Car.
"""

import math
import time
import cv2
from PySide6.QtCore import Qt, QTimer, Slot, QPointF, QRectF
from PySide6.QtGui import QImage, QPixmap, QFont, QPainter, QPen, QColor, QBrush
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QLabel, QPushButton, QSlider, QTextEdit,
    QVBoxLayout, QHBoxLayout, QGridLayout, QFrame, QTabWidget,
    QProgressBar, QComboBox, QLineEdit, QCheckBox
)
import config

class RadarWidget(QWidget):
    """
    Custom 60 FPS Sci-Fi Polar Radar Scope (30° to 150°) for SG90 Servo & HC-SR04 Scanner.
    Draws distance rings, sweep ray, and color-coded obstacle points.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(320, 240)
        self.current_angle = 90
        self.current_distance = 200.0
        self.max_range = 200.0
        self.obstacle_points = []  # List of tuples: (angle, distance, timestamp)

    def update_radar(self, angle, distance):
        self.current_angle = angle
        self.current_distance = distance
        now = time.time()
        self.obstacle_points.append((angle, distance, now))
        # Keep recent points (fade after 4 seconds)
        self.obstacle_points = [p for p in self.obstacle_points if (now - p[2]) < 4.0]
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        center_x = w / 2.0
        center_y = h - 25.0
        radius = min(w / 2.2, h - 45.0)

        # Background Fill
        painter.fillRect(self.rect(), QColor("#080B10"))

        # Concentric distance rings: 50cm, 100cm, 150cm, 200cm
        pen_grid = QPen(QColor("#1E2736"), 1, Qt.DashLine)
        painter.setPen(pen_grid)

        for r_ratio in [0.25, 0.5, 0.75, 1.0]:
            r_curr = radius * r_ratio
            rect = QRectF(center_x - r_curr, center_y - r_curr, r_curr * 2, r_curr * 2)
            painter.drawArc(rect, 30 * 16, 120 * 16)
            
            # Distance text labels
            dist_val = int(r_ratio * self.max_range)
            painter.setPen(QColor("#64748B"))
            painter.setFont(QFont("Segoe UI", 8, QFont.Bold))
            painter.drawText(int(center_x + 5), int(center_y - r_curr + 12), f"{dist_val}cm")
            painter.setPen(pen_grid)

        # Radiating angle lines: 30°, 60°, 90°, 120°, 150°
        angles = [30, 60, 90, 120, 150]
        pen_line = QPen(QColor("#2A364D"), 1, Qt.SolidLine)
        painter.setPen(pen_line)
        for a in angles:
            rad = math.radians(180 - a)
            x_end = center_x + radius * math.cos(rad)
            y_end = center_y - radius * math.sin(rad)
            painter.drawLine(QPointF(center_x, center_y), QPointF(x_end, y_end))

            # Angle text labels
            x_lbl = center_x + (radius + 15) * math.cos(rad) - 10
            y_lbl = center_y - (radius + 15) * math.sin(rad) + 4
            painter.setPen(QColor("#00E5FF"))
            painter.drawText(int(x_lbl), int(y_lbl), f"{a}°")
            painter.setPen(pen_line)

        # Draw historical obstacle detection points
        now = time.time()
        for a, d, t in self.obstacle_points:
            if d >= self.max_range:
                continue
            age = now - t
            alpha = max(0.1, 1.0 - (age / 4.0))

            rad = math.radians(180 - a)
            r_pt = (d / self.max_range) * radius
            pt_x = center_x + r_pt * math.cos(rad)
            pt_y = center_y - r_pt * math.sin(rad)

            # Color coding based on obstacle distance Proximity
            if d < 20.0:
                col = QColor(239, 68, 68, int(255 * alpha))   # Red
            elif d < 50.0:
                col = QColor(245, 158, 11, int(255 * alpha))  # Amber
            else:
                col = QColor(16, 185, 129, int(255 * alpha))  # Green

            painter.setBrush(QBrush(col))
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(QPointF(pt_x, pt_y), 5, 5)

        # Sweeping Radial Beam Line tracking current Servo Angle
        beam_rad = math.radians(180 - self.current_angle)
        beam_x = center_x + radius * math.cos(beam_rad)
        beam_y = center_y - radius * math.sin(beam_rad)

        pen_beam = QPen(QColor("#00E5FF"), 2, Qt.SolidLine)
        painter.setPen(pen_beam)
        painter.drawLine(QPointF(center_x, center_y), QPointF(beam_x, beam_y))

        # Center Origin Hub Dot
        painter.setBrush(QBrush(QColor("#00E5FF")))
        painter.drawEllipse(QPointF(center_x, center_y), 6, 6)

class MainWindow(QMainWindow):
    def __init__(self, camera_stream, hand_detector, gesture_classifier, safety_manager, car_controller, websocket_manager):
        super().__init__()
        self.camera_stream = camera_stream
        self.hand_detector = hand_detector
        self.gesture_classifier = gesture_classifier
        self.safety_manager = safety_manager
        self.car_controller = car_controller
        self.websocket_manager = websocket_manager

        self.setWindowTitle("⚡ ESP32 AI Gesture Car Controller - Command Center")
        self.resize(1240, 820)
        self.setMinimumSize(1024, 720)
        self.setStyleSheet(config.QSS_DARK_THEME)

        self.start_time = time.time()
        self.log_history = []  # Store raw tuples (timestamp, tag, message)

        # Wire PySide6 Signals from WebSocket Manager
        self.websocket_manager.connection_status_changed.connect(self._on_ws_status_changed)
        self.websocket_manager.telemetry_received.connect(self._on_ws_telemetry)
        self.websocket_manager.radar_telemetry_received.connect(self._on_ws_radar_telemetry)
        self.websocket_manager.log_emitted.connect(self._on_ws_log)

        self._build_ui()

        # Timer for 60 FPS video and tracking pipeline loop (~16ms)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._process_frame_loop)
        self.timer.start(16)

    def _build_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        root_layout = QVBoxLayout(main_widget)
        root_layout.setContentsMargins(14, 14, 14, 14)
        root_layout.setSpacing(12)

        # ====================================================
        # TOP HEADER BAR: App Title & Live Telemetry Pills
        # ====================================================
        header_card = QFrame()
        header_card.setProperty("class", "HeaderCard")
        header_layout = QHBoxLayout(header_card)
        header_layout.setContentsMargins(16, 10, 16, 10)

        # App Brand Title
        title_box = QVBoxLayout()
        title_lbl = QLabel("⚡ ESP32 AI GESTURE CAR CONTROLLER")
        title_lbl.setProperty("class", "AppTitle")
        sub_lbl = QLabel("REAL-TIME VISION TELEMETRY & ROBOTICS COMMAND DASHBOARD")
        sub_lbl.setStyleSheet("font-size: 11px; font-weight: 600; color: #64748B; letter-spacing: 0.5px;")
        title_box.addWidget(title_lbl)
        title_box.addWidget(sub_lbl)
        header_layout.addLayout(title_box)

        header_layout.addStretch()

        # Live Status Badges (Pills)
        self.lbl_system_badge = QLabel("● SYSTEM INACTIVE")
        self.lbl_system_badge.setStyleSheet(
            "background-color: #261F18; color: #F59E0B; border: 1px solid #D97706; "
            "border-radius: 14px; padding: 5px 14px; font-size: 12px; font-weight: 800;"
        )
        header_layout.addWidget(self.lbl_system_badge)

        self.lbl_ws_badge = QLabel("● ESP32 OFFLINE")
        self.lbl_ws_badge.setStyleSheet(
            "background-color: #27171A; color: #EF4444; border: 1px solid #DC2626; "
            "border-radius: 14px; padding: 5px 14px; font-size: 12px; font-weight: 800;"
        )
        header_layout.addWidget(self.lbl_ws_badge)

        self.lbl_uptime = QLabel("⏱ 00:00:00")
        self.lbl_uptime.setStyleSheet(
            "background-color: #141A24; color: #00E5FF; border: 1px solid #232D3F; "
            "border-radius: 14px; padding: 5px 14px; font-size: 12px; font-weight: 700;"
        )
        header_layout.addWidget(self.lbl_uptime)

        root_layout.addWidget(header_card)

        # ====================================================
        # MAIN CONTENT AREA: Split Columns
        # ====================================================
        content_layout = QHBoxLayout()
        content_layout.setSpacing(14)

        # ----------------------------------------------------
        # LEFT COLUMN: Camera Stream Viewport & Primary Actions
        # ----------------------------------------------------
        left_layout = QVBoxLayout()
        left_layout.setSpacing(12)

        # Video Frame Container Card
        video_card = QFrame()
        video_card.setProperty("class", "Card")
        video_card_layout = QVBoxLayout(video_card)
        video_card_layout.setContentsMargins(12, 12, 12, 12)
        video_card_layout.setSpacing(8)

        # Viewport Header
        vp_header = QHBoxLayout()
        vp_title = QLabel("📹 LIVE CAMERA STREAM")
        vp_title.setProperty("class", "SectionHeader")
        vp_header.addWidget(vp_title)
        
        self.lbl_res_badge = QLabel(f"{config.FRAME_WIDTH}x{config.FRAME_HEIGHT} • 30 FPS")
        self.lbl_res_badge.setStyleSheet("font-size: 11px; font-weight: bold; color: #64748B;")
        vp_header.addWidget(self.lbl_res_badge, alignment=Qt.AlignRight)
        video_card_layout.addLayout(vp_header)

        # Video Canvas Label
        self.video_label = QLabel()
        self.video_label.setFixedSize(config.FRAME_WIDTH, config.FRAME_HEIGHT)
        self.video_label.setStyleSheet("background-color: #05070A; border: 1px solid #1E2736; border-radius: 8px;")
        self.video_label.setAlignment(Qt.AlignCenter)
        video_card_layout.addWidget(self.video_label, alignment=Qt.AlignCenter)

        # Viewport Options (HUD Overlay & Flip Toggles)
        vp_options = QHBoxLayout()
        self.chk_hud = QCheckBox("Tactical HUD Overlay")
        self.chk_hud.setChecked(True)
        vp_options.addWidget(self.chk_hud)

        self.chk_flip = QCheckBox("Flip Video Horizontal")
        self.chk_flip.setChecked(False)
        vp_options.addWidget(self.chk_flip)
        video_card_layout.addLayout(vp_options)

        left_layout.addWidget(video_card)

        # Primary Action Control Box
        actions_card = QFrame()
        actions_card.setProperty("class", "Card")
        actions_layout = QHBoxLayout(actions_card)
        actions_layout.setContentsMargins(12, 12, 12, 12)
        actions_layout.setSpacing(12)

        self.btn_start = QPushButton("▶ START SYSTEM")
        self.btn_start.setObjectName("btn_start")
        self.btn_start.setMinimumHeight(44)
        self.btn_start.setCursor(Qt.PointingHandCursor)
        self.btn_start.clicked.connect(self._toggle_system_active)
        actions_layout.addWidget(self.btn_start, stretch=1)

        self.btn_emergency = QPushButton("🚨 EMERGENCY STOP 🚨")
        self.btn_emergency.setObjectName("btn_emergency")
        self.btn_emergency.setMinimumHeight(44)
        self.btn_emergency.setCursor(Qt.PointingHandCursor)
        self.btn_emergency.clicked.connect(self._toggle_emergency_stop)
        actions_layout.addWidget(self.btn_emergency, stretch=1)

        left_layout.addWidget(actions_card)
        left_layout.addStretch()

        content_layout.addLayout(left_layout, stretch=4)

        # ----------------------------------------------------
        # RIGHT COLUMN: Tabbed Command & Telemetry Dashboard
        # ----------------------------------------------------
        self.tab_widget = QTabWidget()

        # TAB 1: Real-Time Telemetry, Directional Matrix & Radar Scope
        self.tab_telemetry = QWidget()
        self._build_tab_telemetry()
        self.tab_widget.addTab(self.tab_telemetry, "📊 Telemetry & Status")

        # TAB 2: Calibration & Settings
        self.tab_settings = QWidget()
        self._build_tab_settings()
        self.tab_widget.addTab(self.tab_settings, "⚙️ Settings & Calibration")

        # TAB 3: System Terminal & Logs
        self.tab_logs = QWidget()
        self._build_tab_logs()
        self.tab_widget.addTab(self.tab_logs, "💻 Terminal Logs")

        content_layout.addWidget(self.tab_widget, stretch=5)

        root_layout.addLayout(content_layout, stretch=1)

    def _build_tab_telemetry(self):
        layout = QVBoxLayout(self.tab_telemetry)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        # 2x2 Grid of Status Metric Cards
        grid = QGridLayout()
        grid.setSpacing(8)

        # Metric 1: Detected Gesture & Confidence Bar
        card_gesture = QFrame()
        card_gesture.setProperty("class", "MetricCard")
        g_box = QVBoxLayout(card_gesture)
        g_box.setContentsMargins(6, 6, 6, 6)
        g_box.addWidget(self._create_metric_header("DETECTED GESTURE"))
        self.lbl_gesture_val = QLabel("🚫 NONE")
        self.lbl_gesture_val.setProperty("class", "MetricValue")
        self.lbl_gesture_val.setStyleSheet("color: #94A3B8;")
        g_box.addWidget(self.lbl_gesture_val)
        
        self.bar_conf = QProgressBar()
        self.bar_conf.setRange(0, 100)
        self.bar_conf.setValue(0)
        g_box.addWidget(self.bar_conf)
        grid.addWidget(card_gesture, 0, 0)

        # Metric 2: Active Car Command & Safety Fault
        card_command = QFrame()
        card_command.setProperty("class", "MetricCard")
        c_box = QVBoxLayout(card_command)
        c_box.setContentsMargins(6, 6, 6, 6)
        c_box.addWidget(self._create_metric_header("CAR COMMAND"))
        self.lbl_command_val = QLabel("✋ STOP [S]")
        self.lbl_command_val.setProperty("class", "MetricValue")
        self.lbl_command_val.setStyleSheet("color: #F59E0B;")
        c_box.addWidget(self.lbl_command_val)

        self.lbl_safety_status = QLabel("System Initialized")
        self.lbl_safety_status.setStyleSheet("font-size: 11px; font-weight: bold; color: #10B981;")
        c_box.addWidget(self.lbl_safety_status)
        grid.addWidget(card_command, 0, 1)

        # Metric 3: System Performance & FPS
        card_perf = QFrame()
        card_perf.setProperty("class", "MetricCard")
        p_box = QVBoxLayout(card_perf)
        p_box.setContentsMargins(6, 6, 6, 6)
        p_box.addWidget(self._create_metric_header("CAMERA PERFORMANCE"))
        self.lbl_fps_val = QLabel("0.0 FPS")
        self.lbl_fps_val.setProperty("class", "MetricValue")
        self.lbl_fps_val.setStyleSheet("color: #00E5FF;")
        p_box.addWidget(self.lbl_fps_val)

        self.bar_fps = QProgressBar()
        self.bar_fps.setRange(0, 60)
        self.bar_fps.setValue(0)
        p_box.addWidget(self.bar_fps)
        grid.addWidget(card_perf, 1, 0)

        # Metric 4: ESP32 Link & Battery Level
        card_link = QFrame()
        card_link.setProperty("class", "MetricCard")
        l_box = QVBoxLayout(card_link)
        l_box.setContentsMargins(6, 6, 6, 6)
        l_box.addWidget(self._create_metric_header("ESP32 BATTERY & LINK"))
        self.lbl_battery_val = QLabel("100% 🔋")
        self.lbl_battery_val.setProperty("class", "MetricValue")
        self.lbl_battery_val.setStyleSheet("color: #10B981;")
        l_box.addWidget(self.lbl_battery_val)

        self.bar_battery = QProgressBar()
        self.bar_battery.setRange(0, 100)
        self.bar_battery.setValue(100)
        l_box.addWidget(self.bar_battery)
        grid.addWidget(card_link, 1, 1)

        layout.addLayout(grid)

        # ====================================================
        # EMBEDDED ULTRASONIC RADAR SCOPE CARD (SINGLE WINDOW)
        # ====================================================
        card_radar = QFrame()
        card_radar.setProperty("class", "Card")
        r_box = QVBoxLayout(card_radar)
        r_box.setContentsMargins(8, 8, 8, 8)
        r_box.setSpacing(4)

        r_hdr = QHBoxLayout()
        r_hdr.addWidget(self._create_metric_header("LIVE ULTRASONIC RADAR SCOPE (ON-DEMAND OBSTACLE SWEEP)"))
        self.lbl_radar_status_badge = QLabel("🟢 CLEAR (PARKED @ 90°)")
        self.lbl_radar_status_badge.setStyleSheet("font-size: 11px; font-weight: bold; color: #10B981;")
        r_hdr.addWidget(self.lbl_radar_status_badge, alignment=Qt.AlignRight)
        r_box.addLayout(r_hdr)

        self.radar_widget = RadarWidget()
        r_box.addWidget(self.radar_widget, stretch=1)

        # Compact Radar Telemetry Metrics Row
        rm_grid = QGridLayout()
        rm_grid.setSpacing(6)

        self.lbl_r_angle = QLabel("90°")
        self.lbl_r_angle.setStyleSheet("font-weight: bold; color: #00E5FF;")

        self.lbl_r_dist = QLabel("200 cm")
        self.lbl_r_dist.setStyleSheet("font-weight: bold; color: #10B981;")

        self.lbl_r_left = QLabel("200 cm")
        self.lbl_r_left.setStyleSheet("color: #F1F5F9;")

        self.lbl_r_center = QLabel("200 cm")
        self.lbl_r_center.setStyleSheet("color: #F1F5F9;")

        self.lbl_r_right = QLabel("200 cm")
        self.lbl_r_right.setStyleSheet("color: #F1F5F9;")

        self.lbl_r_path = QLabel("CENTER CLEAR")
        self.lbl_r_path.setStyleSheet("font-weight: bold; color: #00E5FF;")

        rm_grid.addWidget(self._create_metric_header("ANGLE"), 0, 0)
        rm_grid.addWidget(self.lbl_r_angle, 1, 0)

        rm_grid.addWidget(self._create_metric_header("FRONT DIST"), 0, 1)
        rm_grid.addWidget(self.lbl_r_dist, 1, 1)

        rm_grid.addWidget(self._create_metric_header("BEST PATH"), 0, 2)
        rm_grid.addWidget(self.lbl_r_path, 1, 2)

        rm_grid.addWidget(self._create_metric_header("LEFT (110°-150°)"), 0, 3)
        rm_grid.addWidget(self.lbl_r_left, 1, 3)

        rm_grid.addWidget(self._create_metric_header("CENTER (70°-110°)"), 0, 4)
        rm_grid.addWidget(self.lbl_r_center, 1, 4)

        rm_grid.addWidget(self._create_metric_header("RIGHT (30°-70°)"), 0, 5)
        rm_grid.addWidget(self.lbl_r_right, 1, 5)

        r_box.addLayout(rm_grid)
        layout.addWidget(card_radar)

        # Directional Visual Indicator Grid Matrix (Tactical Compass)
        dir_card = QFrame()
        dir_card.setProperty("class", "Card")
        dir_layout = QVBoxLayout(dir_card)
        dir_layout.setContentsMargins(8, 8, 8, 8)
        dir_layout.addWidget(self._create_metric_header("DIRECTIONAL CONTROL MATRIX"))

        matrix_grid = QGridLayout()
        matrix_grid.setSpacing(6)

        self.btn_dir_fwd = QPushButton("▲ FORWARD")
        self.btn_dir_left = QPushButton("◄ LEFT")
        self.btn_dir_stop = QPushButton("● STOP")
        self.btn_dir_right = QPushButton("RIGHT ►")
        self.btn_dir_back = QPushButton("▼ BACKWARD")

        self.dir_buttons = {
            config.CMD_FORWARD: self.btn_dir_fwd,
            config.CMD_LEFT: self.btn_dir_left,
            config.CMD_STOP: self.btn_dir_stop,
            config.CMD_RIGHT: self.btn_dir_right,
            config.CMD_BACKWARD: self.btn_dir_back
        }

        for b in self.dir_buttons.values():
            b.setEnabled(False)
            b.setStyleSheet("background-color: #18202D; color: #475569; border: 1px solid #232D3F;")

        matrix_grid.addWidget(self.btn_dir_fwd, 0, 1)
        matrix_grid.addWidget(self.btn_dir_left, 1, 0)
        matrix_grid.addWidget(self.btn_dir_stop, 1, 1)
        matrix_grid.addWidget(self.btn_dir_right, 1, 2)
        matrix_grid.addWidget(self.btn_dir_back, 2, 1)

        dir_layout.addLayout(matrix_grid)
        layout.addWidget(dir_card)

    @Slot(dict)
    def _on_ws_radar_telemetry(self, data):
        angle = data.get("angle", 90)
        dist = data.get("distance", 200)
        left = data.get("left_dist", 200)
        center = data.get("center_dist", 200)
        right = data.get("right_dist", 200)
        status = data.get("status", "CLEAR")
        path = data.get("best_path", "CENTER")

        self.radar_widget.update_radar(angle, dist)

        self.lbl_r_angle.setText(f"{angle}°")
        self.lbl_r_dist.setText(f"{dist} cm")
        self.lbl_r_left.setText(f"{left} cm")
        self.lbl_r_center.setText(f"{center} cm")
        self.lbl_r_right.setText(f"{right} cm")

        meta = config.RADAR_STATUS_METADATA.get(status, ("CLEAR", "🟢", "#10B981"))
        self.lbl_radar_status_badge.setText(f"{meta[1]} {meta[0]}")
        self.lbl_r_path.setText(f"RECOMMENDED: {path}")

    def closeEvent(self, event):
        """Clean application exit handler."""
        if self.car_controller:
            self.car_controller.send_immediate_stop()
        if self.websocket_manager:
            self.websocket_manager.stop()
        if self.camera_stream:
            self.camera_stream.stop()
        if self.hand_detector:
            self.hand_detector.close()
        event.accept()

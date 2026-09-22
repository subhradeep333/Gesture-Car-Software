"""
Modern PySide6 (Qt6) Desktop Command Dashboard for ESP32 AI Hand Gesture Controlled IoT Car.
"""

import time
import cv2
from PySide6.QtCore import Qt, QTimer, Slot
from PySide6.QtGui import QImage, QPixmap, QFont
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QLabel, QPushButton, QSlider, QTextEdit,
    QVBoxLayout, QHBoxLayout, QGridLayout, QFrame, QTabWidget,
    QProgressBar, QComboBox, QLineEdit, QCheckBox
)
import config

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

        # TAB 1: Real-Time Telemetry & Gauges
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
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(12)

        # 2x2 Grid of Status Metric Cards
        grid = QGridLayout()
        grid.setSpacing(10)

        # Metric 1: Detected Gesture & Confidence Bar
        card_gesture = QFrame()
        card_gesture.setProperty("class", "MetricCard")
        g_box = QVBoxLayout(card_gesture)
        g_box.addWidget(self._create_metric_header("DETECTED GESTURE"))
        self.lbl_gesture_val = QLabel("🚫 NONE")
        self.lbl_gesture_val.setProperty("class", "MetricValue")
        self.lbl_gesture_val.setStyleSheet("color: #94A3B8;")
        g_box.addWidget(self.lbl_gesture_val)
        
        g_box.addWidget(QLabel("Confidence Score:"))
        self.bar_conf = QProgressBar()
        self.bar_conf.setRange(0, 100)
        self.bar_conf.setValue(0)
        g_box.addWidget(self.bar_conf)
        grid.addWidget(card_gesture, 0, 0)

        # Metric 2: Active Car Command & Safety Fault
        card_command = QFrame()
        card_command.setProperty("class", "MetricCard")
        c_box = QVBoxLayout(card_command)
        c_box.addWidget(self._create_metric_header("CAR COMMAND"))
        self.lbl_command_val = QLabel("✋ STOP [S]")
        self.lbl_command_val.setProperty("class", "MetricValue")
        self.lbl_command_val.setStyleSheet("color: #F59E0B;")
        c_box.addWidget(self.lbl_command_val)

        c_box.addWidget(QLabel("Safety Status:"))
        self.lbl_safety_status = QLabel("System Initialized")
        self.lbl_safety_status.setStyleSheet("font-size: 11px; font-weight: bold; color: #10B981;")
        c_box.addWidget(self.lbl_safety_status)
        grid.addWidget(card_command, 0, 1)

        # Metric 3: System Performance & FPS
        card_perf = QFrame()
        card_perf.setProperty("class", "MetricCard")
        p_box = QVBoxLayout(card_perf)
        p_box.addWidget(self._create_metric_header("CAMERA PERFORMANCE"))
        self.lbl_fps_val = QLabel("0.0 FPS")
        self.lbl_fps_val.setProperty("class", "MetricValue")
        self.lbl_fps_val.setStyleSheet("color: #00E5FF;")
        p_box.addWidget(self.lbl_fps_val)

        p_box.addWidget(QLabel("Target Frame Rate:"))
        self.bar_fps = QProgressBar()
        self.bar_fps.setRange(0, 60)
        self.bar_fps.setValue(0)
        p_box.addWidget(self.bar_fps)
        grid.addWidget(card_perf, 1, 0)

        # Metric 4: ESP32 Link & Battery Level
        card_link = QFrame()
        card_link.setProperty("class", "MetricCard")
        l_box = QVBoxLayout(card_link)
        l_box.addWidget(self._create_metric_header("ESP32 BATTERY & LINK"))
        self.lbl_battery_val = QLabel("100% 🔋")
        self.lbl_battery_val.setProperty("class", "MetricValue")
        self.lbl_battery_val.setStyleSheet("color: #10B981;")
        l_box.addWidget(self.lbl_battery_val)

        l_box.addWidget(QLabel("Battery Level:"))
        self.bar_battery = QProgressBar()
        self.bar_battery.setRange(0, 100)
        self.bar_battery.setValue(100)
        l_box.addWidget(self.bar_battery)
        grid.addWidget(card_link, 1, 1)

        layout.addLayout(grid)

        # Directional Visual Indicator Grid Matrix (Tactical Compass)
        dir_card = QFrame()
        dir_card.setProperty("class", "Card")
        dir_layout = QVBoxLayout(dir_card)
        dir_layout.addWidget(self._create_metric_header("DIRECTIONAL CONTROL MATRIX"))

        matrix_grid = QGridLayout()
        matrix_grid.setSpacing(8)

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
        layout.addStretch()

    def _build_tab_settings(self):
        layout = QVBoxLayout(self.tab_settings)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(12)

        # Tuning Card 1: Motor Speed PWM
        card_speed = QFrame()
        card_speed.setProperty("class", "Card")
        s_layout = QVBoxLayout(card_speed)

        s_hdr = QHBoxLayout()
        s_hdr.addWidget(self._create_metric_header("MOTOR SPEED PWM"))
        self.lbl_speed_val = QLabel("180 (71%)")
        self.lbl_speed_val.setStyleSheet("font-weight: bold; color: #00E5FF;")
        s_hdr.addWidget(self.lbl_speed_val, alignment=Qt.AlignRight)
        s_layout.addLayout(s_hdr)

        self.slider_speed = QSlider(Qt.Horizontal)
        self.slider_speed.setRange(100, 255)
        self.slider_speed.setValue(config.DEFAULT_MOTOR_SPEED)
        self.slider_speed.valueChanged.connect(self._on_speed_slider_changed)
        s_layout.addWidget(self.slider_speed)
        layout.addWidget(card_speed)

        # Tuning Card 2: Hysteresis Frame Buffer
        card_stab = QFrame()
        card_stab.setProperty("class", "Card")
        st_layout = QVBoxLayout(card_stab)

        st_hdr = QHBoxLayout()
        st_hdr.addWidget(self._create_metric_header("STABILITY HYSTERESIS BUFFER"))
        self.lbl_stab_val = QLabel("5 Frames")
        self.lbl_stab_val.setStyleSheet("font-weight: bold; color: #00E5FF;")
        st_hdr.addWidget(self.lbl_stab_val, alignment=Qt.AlignRight)
        st_layout.addLayout(st_hdr)

        self.slider_stab = QSlider(Qt.Horizontal)
        self.slider_stab.setRange(1, 10)
        self.slider_stab.setValue(config.DEFAULT_STABILITY_FRAME_THRESHOLD)
        self.slider_stab.valueChanged.connect(self._on_stab_slider_changed)
        st_layout.addWidget(self.slider_stab)
        layout.addWidget(card_stab)

        # Tuning Card 3: Min Landmark Confidence
        card_conf = QFrame()
        card_conf.setProperty("class", "Card")
        c_layout = QVBoxLayout(card_conf)

        c_hdr = QHBoxLayout()
        c_hdr.addWidget(self._create_metric_header("MIN LANDMARK CONFIDENCE"))
        self.lbl_conf_setting = QLabel("0.70")
        self.lbl_conf_setting.setStyleSheet("font-weight: bold; color: #00E5FF;")
        c_hdr.addWidget(self.lbl_conf_setting, alignment=Qt.AlignRight)
        c_layout.addLayout(c_hdr)

        self.slider_conf = QSlider(Qt.Horizontal)
        self.slider_conf.setRange(50, 95)
        self.slider_conf.setValue(70)
        self.slider_conf.valueChanged.connect(self._on_conf_slider_changed)
        c_layout.addWidget(self.slider_conf)
        layout.addWidget(card_conf)

        layout.addStretch()

    def _build_tab_logs(self):
        layout = QVBoxLayout(self.tab_logs)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        # Filter & Action Toolbar
        toolbar = QHBoxLayout()
        
        toolbar.addWidget(QLabel("Filter:"))
        self.cmb_filter = QComboBox()
        self.cmb_filter.addItems(["ALL LOGS", "WS", "TX", "RX", "SAFETY", "SYSTEM", "ERROR"])
        self.cmb_filter.currentTextChanged.connect(self._refresh_log_display)
        toolbar.addWidget(self.cmb_filter)

        self.txt_search = QLineEdit()
        self.txt_search.setPlaceholderText("Search log messages...")
        self.txt_search.textChanged.connect(self._refresh_log_display)
        toolbar.addWidget(self.txt_search, stretch=1)

        btn_clear = QPushButton("Clear")
        btn_clear.clicked.connect(self._clear_log)
        toolbar.addWidget(btn_clear)

        self.chk_autoscroll = QCheckBox("Auto-Scroll")
        self.chk_autoscroll.setChecked(True)
        toolbar.addWidget(self.chk_autoscroll)

        layout.addLayout(toolbar)

        # Terminal Console Box
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        layout.addWidget(self.log_text, stretch=1)

    def _create_metric_header(self, text):
        lbl = QLabel(text)
        lbl.setProperty("class", "MetricTitle")
        return lbl

    def _on_speed_slider_changed(self, val):
        pct = int((val / 255.0) * 100)
        self.lbl_speed_val.setText(f"{val} ({pct}%)")
        self.car_controller.set_motor_speed(val)

    def _on_stab_slider_changed(self, val):
        self.lbl_stab_val.setText(f"{val} Frames")
        self.gesture_classifier.set_stability_threshold(val)

    def _on_conf_slider_changed(self, val):
        conf = val / 100.0
        self.lbl_conf_setting.setText(f"{conf:.2f}")
        self.hand_detector.set_confidence_thresholds(conf, conf)

    def _toggle_system_active(self):
        if self.safety_manager.system_active:
            self.safety_manager.set_system_active(False)
            self.car_controller.send_immediate_stop()
            self.btn_start.setText("▶ START SYSTEM")
            self.btn_start.setObjectName("btn_start")
            self.btn_start.setStyleSheet("")  # Apply QSS rule
            self.lbl_system_badge.setText("● SYSTEM PAUSED")
            self.lbl_system_badge.setStyleSheet(
                "background-color: #261F18; color: #F59E0B; border: 1px solid #D97706; "
                "border-radius: 14px; padding: 5px 14px; font-size: 12px; font-weight: 800;"
            )
            self._on_ws_log("SYSTEM", "System Paused by User")
        else:
            self.safety_manager.set_system_active(True)
            self.btn_start.setText("⏸ PAUSE SYSTEM")
            self.btn_start.setObjectName("btn_pause")
            self.btn_start.setStyleSheet("")  # Apply QSS rule
            self.lbl_system_badge.setText("● SYSTEM ACTIVE")
            self.lbl_system_badge.setStyleSheet(
                "background-color: #162B22; color: #10B981; border: 1px solid #059669; "
                "border-radius: 14px; padding: 5px 14px; font-size: 12px; font-weight: 800;"
            )
            self._on_ws_log("SYSTEM", "System Started")

    def _toggle_emergency_stop(self):
        if self.safety_manager.emergency_override:
            self.safety_manager.clear_emergency_stop()
            self.btn_emergency.setText("🚨 EMERGENCY STOP 🚨")
            self.lbl_system_badge.setText("● SYSTEM PAUSED")
            self.lbl_system_badge.setStyleSheet(
                "background-color: #261F18; color: #F59E0B; border: 1px solid #D97706; "
                "border-radius: 14px; padding: 5px 14px; font-size: 12px; font-weight: 800;"
            )
            self._on_ws_log("SYSTEM", "Emergency Stop Cleared")
        else:
            self.safety_manager.trigger_emergency_stop()
            self.car_controller.send_immediate_stop()
            self.btn_emergency.setText("CLEAR EMERGENCY ⚠️")
            self.lbl_system_badge.setText("🚨 EMERGENCY STOP")
            self.lbl_system_badge.setStyleSheet(
                "background-color: #3B1619; color: #EF4444; border: 1px solid #DC2626; "
                "border-radius: 14px; padding: 5px 14px; font-size: 12px; font-weight: 800;"
            )
            self._on_ws_log("EMERGENCY", "EMERGENCY STOP ACTIVATED VIA GUI BUTTON")

    @Slot(bool, str)
    def _on_ws_status_changed(self, is_connected, status_text):
        if is_connected:
            self.lbl_ws_badge.setText("● ESP32 ONLINE")
            self.lbl_ws_badge.setStyleSheet(
                "background-color: #162B22; color: #10B981; border: 1px solid #059669; "
                "border-radius: 14px; padding: 5px 14px; font-size: 12px; font-weight: 800;"
            )
        else:
            self.lbl_ws_badge.setText("● ESP32 OFFLINE")
            self.lbl_ws_badge.setStyleSheet(
                "background-color: #27171A; color: #EF4444; border: 1px solid #DC2626; "
                "border-radius: 14px; padding: 5px 14px; font-size: 12px; font-weight: 800;"
            )

    @Slot(dict)
    def _on_ws_telemetry(self, data):
        if "battery" in data:
            bat = data["battery"]
            self.lbl_battery_val.setText(f"{bat}% 🔋")
            self.bar_battery.setValue(int(bat))
            if bat > 50:
                self.lbl_battery_val.setStyleSheet("color: #10B981;")
            elif bat > 20:
                self.lbl_battery_val.setStyleSheet("color: #F59E0B;")
            else:
                self.lbl_battery_val.setStyleSheet("color: #EF4444;")

    @Slot(str, str)
    def _on_ws_log(self, tag, message):
        timestamp = time.strftime("%H:%M:%S")
        self.log_history.append((timestamp, tag, message))
        if len(self.log_history) > 500:
            self.log_history.pop(0)

        self._append_log_entry(timestamp, tag, message)

    def _append_log_entry(self, timestamp, tag, message):
        # Apply filters
        selected_filter = self.cmb_filter.currentText()
        if selected_filter != "ALL LOGS" and tag != selected_filter:
            if selected_filter == "ERROR" and "ERROR" not in tag:
                return
            elif selected_filter != "ERROR" and selected_filter not in tag:
                return

        search_txt = self.txt_search.text().lower()
        if search_txt and search_txt not in message.lower() and search_txt not in tag.lower():
            return

        # Color palette for HTML tags
        tag_colors = {
            "WS": "#00E5FF",
            "TX": "#10B981",
            "RX": "#8B5CF6",
            "SYSTEM": "#F59E0B",
            "EMERGENCY": "#EF4444",
            "WS_ERROR": "#EF4444",
            "TX_ERROR": "#EF4444",
            "RX_ERROR": "#EF4444"
        }
        color = tag_colors.get(tag, "#94A3B8")

        html = f"""
        <div style="margin-bottom: 2px;">
            <span style="color: #64748B;">[{timestamp}]</span>
            <span style="color: {color}; font-weight: bold;">[{tag}]</span>
            <span style="color: #E2E8F0;">{message}</span>
        </div>
        """
        self.log_text.append(html)

        if self.chk_autoscroll.isChecked():
            self.log_text.verticalScrollBar().setValue(
                self.log_text.verticalScrollBar().maximum()
            )

    def _refresh_log_display(self):
        self.log_text.clear()
        for ts, tag, msg in self.log_history:
            self._append_log_entry(ts, tag, msg)

    def _clear_log(self):
        self.log_history.clear()
        self.log_text.clear()

    def _process_frame_loop(self):
        # 1. Update Uptime Display
        elapsed_sec = int(time.time() - self.start_time)
        hrs, rem = divmod(elapsed_sec, 3600)
        mins, secs = divmod(rem, 60)
        self.lbl_uptime.setText(f"⏱ {hrs:02d}:{mins:02d}:{secs:02d}")

        # 2. Grab camera frame
        ret, frame, fps = self.camera_stream.read()

        candidate_gesture = "NONE"
        stable_cmd = config.CMD_STOP
        is_stable = False
        detected = False
        confidence = 0.0

        min_conf = self.slider_conf.value() / 100.0

        if ret and frame is not None:
            if self.chk_flip.isChecked():
                frame = cv2.flip(frame, 1)

            # 3. Process Hand Detection
            detected, landmarks_list, pixel_landmarks, confidence, hand_type = self.hand_detector.process_frame(frame)

            # 4. Classify Gesture & Apply Hysteresis
            candidate_gesture, stable_cmd, is_stable = self.gesture_classifier.process(
                detected, landmarks_list, confidence, min_confidence=min_conf
            )

            # 5. Evaluate Safety & Dispatch Command via WebSocket
            active_car_cmd, safety_ok, fault_reason = self.car_controller.update(
                stable_cmd, is_stable, detected, ret, confidence, min_conf
            )

            # 6. Draw HUD Overlay on Frame
            if self.chk_hud.isChecked():
                g_meta = config.GESTURE_METADATA.get(candidate_gesture, config.GESTURE_METADATA["NONE"])
                hex_col = g_meta[2].lstrip('#')
                bgr_col = tuple(int(hex_col[i:i+2], 16) for i in (4, 2, 0))
                frame = self.hand_detector.draw_landmarks(frame, pixel_landmarks, g_meta[0], bgr_col, confidence)

            # 7. Render Frame to QLabel via QImage
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_frame.shape
            bytes_per_line = ch * w
            q_img = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_img)
            self.video_label.setPixmap(pixmap)
        else:
            active_car_cmd, safety_ok, fault_reason = self.car_controller.update(
                config.CMD_STOP, True, False, False, 0.0, min_conf
            )

        # 8. Update Telemetry UI Displays & Controls
        g_meta = config.GESTURE_METADATA.get(candidate_gesture, config.GESTURE_METADATA["NONE"])
        self.lbl_gesture_val.setText(f"{g_meta[1]} {g_meta[0]}")
        self.lbl_gesture_val.setStyleSheet(f"color: {g_meta[2]};")

        conf_pct = int(confidence * 100.0) if detected else 0
        self.bar_conf.setValue(conf_pct)

        c_meta = config.GESTURE_METADATA.get(active_car_cmd, config.GESTURE_METADATA[config.CMD_STOP])
        self.lbl_command_val.setText(f"{c_meta[1]} {c_meta[0]} [{active_car_cmd}]")
        self.lbl_command_val.setStyleSheet(f"color: {c_meta[2]};")

        self.lbl_safety_status.setText(fault_reason)
        if safety_ok and active_car_cmd != config.CMD_STOP:
            self.lbl_safety_status.setStyleSheet("font-size: 11px; font-weight: bold; color: #10B981;")
        else:
            self.lbl_safety_status.setStyleSheet("font-size: 11px; font-weight: bold; color: #F59E0B;")

        self.lbl_fps_val.setText(f"{fps:.1f} FPS")
        self.bar_fps.setValue(min(60, int(fps)))

        # Highlight Active Direction Button in Grid Matrix
        for cmd_key, btn in self.dir_buttons.items():
            if cmd_key == active_car_cmd:
                meta = config.GESTURE_METADATA.get(cmd_key, ("STOP", "●", "#00E5FF"))
                btn.setStyleSheet(f"background-color: {meta[2]}; color: #FFFFFF; font-weight: bold; border: none;")
            else:
                btn.setStyleSheet("background-color: #18202D; color: #475569; border: 1px solid #232D3F;")

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

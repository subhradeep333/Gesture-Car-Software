"""
Modern PySide6 (Qt6) Desktop GUI Dashboard for ESP32 AI Hand Gesture Controlled IoT Car.
"""

import time
import cv2
from PIL import Image
from PySide6.QtCore import Qt, QTimer, Slot
from PySide6.QtGui import QImage, QPixmap, QFont, QColor
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QLabel, QPushButton, QSlider, QTextEdit,
    QVBoxLayout, QHBoxLayout, QGridLayout, QFrame, QGroupBox, QSplitter
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

        self.setWindowTitle("🤖 AI Gesture Car Controller - ESP32 Wi-Fi Edition")
        self.resize(1180, 760)
        self.setStyleSheet(config.QSS_DARK_THEME)

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
        main_layout = QHBoxLayout(main_widget)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(12)

        # ====================================================
        # LEFT COLUMN: Live Camera Feed & Primary Controls
        # ====================================================
        left_layout = QVBoxLayout()
        left_layout.setSpacing(10)

        # Header Title
        title_lbl = QLabel("🤖 GESTURE CAR CONTROLLER")
        title_lbl.setProperty("class", "Header")
        title_lbl.setStyleSheet("font-size: 20px; font-weight: bold; color: #00d2ff;")
        left_layout.addWidget(title_lbl)

        # Video Preview Label (Canvas)
        self.video_label = QLabel()
        self.video_label.setFixedSize(config.FRAME_WIDTH, config.FRAME_HEIGHT)
        self.video_label.setStyleSheet("background-color: #000000; border: 1px solid #2c3e50; border-radius: 8px;")
        self.video_label.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(self.video_label)

        # Main Action Buttons (Start, Pause, Emergency Stop)
        btn_box = QHBoxLayout()
        btn_box.setSpacing(10)

        self.btn_start = QPushButton("▶ START SYSTEM")
        self.btn_start.setObjectName("btn_start")
        self.btn_start.setMinimumHeight(42)
        self.btn_start.clicked.connect(self._toggle_system_active)
        btn_box.addWidget(self.btn_start)

        self.btn_emergency = QPushButton("🚨 EMERGENCY STOP 🚨")
        self.btn_emergency.setObjectName("btn_emergency")
        self.btn_emergency.setMinimumHeight(42)
        self.btn_emergency.clicked.connect(self._toggle_emergency_stop)
        btn_box.addWidget(self.btn_emergency)

        left_layout.addLayout(btn_box)
        main_layout.addLayout(left_layout, stretch=3)

        # ====================================================
        # RIGHT COLUMN: Real-Time Telemetry & Tuning Controls
        # ====================================================
        right_layout = QVBoxLayout()
        right_layout.setSpacing(10)

        # --- 1. Status Cards Grid (2x3 Grid) ---
        status_box = QFrame()
        status_box.setProperty("class", "Card")
        status_box.setStyleSheet("background-color: #1a1a1e; border-radius: 8px; border: 1px solid #2c3e50; padding: 8px;")
        grid = QGridLayout(status_box)
        grid.setSpacing(10)

        # Card: Detected Gesture
        grid.addWidget(self._create_card_header("DETECTED GESTURE"), 0, 0)
        self.lbl_gesture_val = QLabel("🚫 NONE")
        self.lbl_gesture_val.setStyleSheet("font-size: 16px; font-weight: bold; color: #95a5a6;")
        grid.addWidget(self.lbl_gesture_val, 1, 0)

        # Card: Car Movement Command
        grid.addWidget(self._create_card_header("CAR COMMAND"), 0, 1)
        self.lbl_command_val = QLabel("✋ STOP [S]")
        self.lbl_command_val.setStyleSheet("font-size: 16px; font-weight: bold; color: #e67e22;")
        grid.addWidget(self.lbl_command_val, 1, 1)

        # Card: Confidence %
        grid.addWidget(self._create_card_header("CONFIDENCE"), 2, 0)
        self.lbl_conf_val = QLabel("0.0%")
        self.lbl_conf_val.setStyleSheet("font-size: 16px; font-weight: bold; color: #00d2ff;")
        grid.addWidget(self.lbl_conf_val, 3, 0)

        # Card: System FPS
        grid.addWidget(self._create_card_header("CAMERA FPS"), 2, 1)
        self.lbl_fps_val = QLabel("0.0 FPS")
        self.lbl_fps_val.setStyleSheet("font-size: 16px; font-weight: bold; color: #00d2ff;")
        grid.addWidget(self.lbl_fps_val, 3, 1)

        # Card: ESP32 Wi-Fi / WebSocket Link
        grid.addWidget(self._create_card_header("ESP32 LINK"), 4, 0)
        self.lbl_ws_status = QLabel("DISCONNECTED 🔴")
        self.lbl_ws_status.setStyleSheet("font-size: 13px; font-weight: bold; color: #e74c3c;")
        grid.addWidget(self.lbl_ws_status, 5, 0)

        # Card: Car Battery %
        grid.addWidget(self._create_card_header("CAR BATTERY"), 4, 1)
        self.lbl_battery_val = QLabel("N/A 🔋")
        self.lbl_battery_val.setStyleSheet("font-size: 13px; font-weight: bold; color: #a4b0be;")
        grid.addWidget(self.lbl_battery_val, 5, 1)

        right_layout.addWidget(status_box)

        # --- 2. Tuning Sliders Card ---
        sliders_box = QFrame()
        sliders_box.setStyleSheet("background-color: #1a1a1e; border-radius: 8px; border: 1px solid #2c3e50; padding: 10px;")
        slider_layout = QVBoxLayout(sliders_box)

        hdr_settings = QLabel("SETTINGS & TUNING")
        hdr_settings.setStyleSheet("font-size: 12px; font-weight: bold; color: #a4b0be;")
        slider_layout.addWidget(hdr_settings)

        # Motor Speed PWM Slider
        speed_hdr = QHBoxLayout()
        speed_hdr.addWidget(QLabel("Motor Speed (PWM):"))
        self.lbl_speed_val = QLabel("180 (71%)")
        self.lbl_speed_val.setStyleSheet("font-weight: bold; color: #00d2ff;")
        speed_hdr.addWidget(self.lbl_speed_val, alignment=Qt.AlignRight)
        slider_layout.addLayout(speed_hdr)

        self.slider_speed = QSlider(Qt.Horizontal)
        self.slider_speed.setRange(100, 255)
        self.slider_speed.setValue(180)
        self.slider_speed.valueChanged.connect(self._on_speed_slider_changed)
        slider_layout.addWidget(self.slider_speed)

        # Stability Frame Buffer Slider
        stab_hdr = QHBoxLayout()
        stab_hdr.addWidget(QLabel("Stability Buffer (Frames):"))
        self.lbl_stab_val = QLabel("5")
        self.lbl_stab_val.setStyleSheet("font-weight: bold; color: #00d2ff;")
        stab_hdr.addWidget(self.lbl_stab_val, alignment=Qt.AlignRight)
        slider_layout.addLayout(stab_hdr)

        self.slider_stab = QSlider(Qt.Horizontal)
        self.slider_stab.setRange(1, 10)
        self.slider_stab.setValue(5)
        self.slider_stab.valueChanged.connect(self._on_stab_slider_changed)
        slider_layout.addWidget(self.slider_stab)

        # Min Confidence Threshold Slider
        conf_hdr = QHBoxLayout()
        conf_hdr.addWidget(QLabel("Min Landmark Confidence:"))
        self.lbl_conf_setting = QLabel("0.70")
        self.lbl_conf_setting.setStyleSheet("font-weight: bold; color: #00d2ff;")
        conf_hdr.addWidget(self.lbl_conf_setting, alignment=Qt.AlignRight)
        slider_layout.addLayout(conf_hdr)

        self.slider_conf = QSlider(Qt.Horizontal)
        self.slider_conf.setRange(50, 95)
        self.slider_conf.setValue(70)
        self.slider_conf.valueChanged.connect(self._on_conf_slider_changed)
        slider_layout.addWidget(self.slider_conf)

        right_layout.addWidget(sliders_box)

        # --- 3. Telemetry Log Window ---
        log_box = QFrame()
        log_box.setStyleSheet("background-color: #1a1a1e; border-radius: 8px; border: 1px solid #2c3e50; padding: 10px;")
        log_layout = QVBoxLayout(log_box)

        log_hdr_box = QHBoxLayout()
        log_hdr_box.addWidget(QLabel("WEBSOCKET TELEMETRY LOG"))
        btn_clear_log = QPushButton("Clear")
        btn_clear_log.setMaximumWidth(60)
        btn_clear_log.clicked.connect(self._clear_log)
        log_hdr_box.addWidget(btn_clear_log, alignment=Qt.AlignRight)
        log_layout.addLayout(log_hdr_box)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)

        right_layout.addWidget(log_box, stretch=1)

        main_layout.addLayout(right_layout, stretch=2)

    def _create_card_header(self, text):
        lbl = QLabel(text)
        lbl.setStyleSheet("font-size: 10px; font-weight: bold; color: #a4b0be;")
        return lbl

    def _on_speed_slider_changed(self, val):
        pct = int((val / 255.0) * 100)
        self.lbl_speed_val.setText(f"{val} ({pct}%)")
        self.car_controller.set_motor_speed(val)

    def _on_stab_slider_changed(self, val):
        self.lbl_stab_val.setText(str(val))
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
            self.btn_start.setStyleSheet("background-color: #2ecc71;")
            self._on_ws_log("SYSTEM", "System Paused by User")
        else:
            self.safety_manager.set_system_active(True)
            self.btn_start.setText("⏸ PAUSE SYSTEM")
            self.btn_start.setStyleSheet("background-color: #f39c12;")
            self._on_ws_log("SYSTEM", "System Started")

    def _toggle_emergency_stop(self):
        if self.safety_manager.emergency_override:
            self.safety_manager.clear_emergency_stop()
            self.btn_emergency.setText("🚨 EMERGENCY STOP 🚨")
            self.btn_emergency.setStyleSheet("background-color: #e74c3c;")
            self._on_ws_log("SYSTEM", "Emergency Stop Cleared")
        else:
            self.safety_manager.trigger_emergency_stop()
            self.car_controller.send_immediate_stop()
            self.btn_emergency.setText("CLEAR EMERGENCY ⚠️")
            self.btn_emergency.setStyleSheet("background-color: #f39c12;")
            self._on_ws_log("EMERGENCY", "EMERGENCY STOP ACTIVATED VIA GUI BUTTON")

    @Slot(bool, str)
    def _on_ws_status_changed(self, is_connected, status_text):
        if is_connected:
            self.lbl_ws_status.setText("CONNECTED 🟢")
            self.lbl_ws_status.setStyleSheet("font-size: 13px; font-weight: bold; color: #2ecc71;")
        else:
            self.lbl_ws_status.setText("DISCONNECTED 🔴")
            self.lbl_ws_status.setStyleSheet("font-size: 13px; font-weight: bold; color: #e74c3c;")

    @Slot(dict)
    def _on_ws_telemetry(self, data):
        if "battery" in data:
            self.lbl_battery_val.setText(f"{data['battery']}% 🔋")
            if data["battery"] > 50:
                self.lbl_battery_val.setStyleSheet("font-size: 13px; font-weight: bold; color: #2ecc71;")
            else:
                self.lbl_battery_val.setStyleSheet("font-size: 13px; font-weight: bold; color: #f39c12;")

    @Slot(str, str)
    def _on_ws_log(self, tag, message):
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] [{tag}] {message}")

    def _clear_log(self):
        self.log_text.clear()

    def _process_frame_loop(self):
        # 1. Grab camera frame
        ret, frame, fps = self.camera_stream.read()

        candidate_gesture = "NONE"
        stable_cmd = config.CMD_STOP
        is_stable = False
        detected = False
        confidence = 0.0

        min_conf = self.slider_conf.value() / 100.0

        if ret and frame is not None:
            # 2. Process Hand Detection
            detected, landmarks_list, pixel_landmarks, confidence, hand_type = self.hand_detector.process_frame(frame)

            # 3. Classify Gesture & Apply Hysteresis
            candidate_gesture, stable_cmd, is_stable = self.gesture_classifier.process(
                detected, landmarks_list, confidence, min_confidence=min_conf
            )

            # 4. Evaluate Safety & Dispatch Command via WebSocket
            active_car_cmd, safety_ok, fault_reason = self.car_controller.update(
                stable_cmd, is_stable, detected, ret, confidence, min_conf
            )

            # 5. Draw Sleek Landmarks on Frame
            g_meta = config.GESTURE_METADATA.get(candidate_gesture, config.GESTURE_METADATA["NONE"])
            hex_col = g_meta[2].lstrip('#')
            bgr_col = tuple(int(hex_col[i:i+2], 16) for i in (4, 2, 0))
            frame = self.hand_detector.draw_landmarks(frame, pixel_landmarks, g_meta[0], bgr_col)

            # 6. Render Frame to QLabel via QImage
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

        # 7. Update Status Badges
        g_meta = config.GESTURE_METADATA.get(candidate_gesture, config.GESTURE_METADATA["NONE"])
        self.lbl_gesture_val.setText(f"{g_meta[1]} {g_meta[0]}")
        self.lbl_gesture_val.setStyleSheet(f"font-size: 16px; font-weight: bold; color: {g_meta[2]};")

        c_meta = config.GESTURE_METADATA.get(active_car_cmd, config.GESTURE_METADATA[config.CMD_STOP])
        self.lbl_command_val.setText(f"{c_meta[1]} {c_meta[0]} [{active_car_cmd}]")
        self.lbl_command_val.setStyleSheet(f"font-size: 16px; font-weight: bold; color: {c_meta[2]};")

        conf_pct = confidence * 100.0 if detected else 0.0
        self.lbl_conf_val.setText(f"{conf_pct:.1f}%")
        self.lbl_fps_val.setText(f"{fps:.1f} FPS")

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

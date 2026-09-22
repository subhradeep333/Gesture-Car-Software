"""
Modern Dark Desktop GUI for Real-time AI Hand Gesture Controlled IoT Car.
"""

import time
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
import cv2
import config

class CarControlUI:
    def __init__(self, root, camera_stream, hand_detector, gesture_classifier, car_controller, serial_manager):
        self.root = root
        self.camera_stream = camera_stream
        self.hand_detector = hand_detector
        self.gesture_classifier = gesture_classifier
        self.car_controller = car_controller
        self.serial_manager = serial_manager

        self.root.title("Real-Time AI Hand Gesture Controlled IoT Car")
        self.root.geometry("1180x740")
        self.root.configure(bg=config.BG_DARK)
        self.root.resizable(True, True)

        # Register serial callbacks
        self.serial_manager.register_callbacks(
            telemetry_cb=self._on_serial_telemetry,
            status_cb=self._on_serial_status_change
        )

        self._setup_styles()
        self._build_layout()
        self._refresh_serial_ports()

        # GUI Update Loop
        self.is_running = True
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._update_loop()

    def _setup_styles(self):
        style = ttk.Style()
        style.theme_use('clam')

        # Configure dark theme colors for ttk components
        style.configure('.', background=config.BG_DARK, foreground=config.TEXT_PRIMARY)
        style.configure('TFrame', background=config.BG_DARK)
        style.configure('Card.TFrame', background=config.BG_CARD, relief='flat', borderwidth=1)
        style.configure('TLabel', background=config.BG_DARK, foreground=config.TEXT_PRIMARY, font=('Helvetica', 10))
        style.configure('Header.TLabel', font=('Helvetica', 14, 'bold'), foreground=config.ACCENT_PRIMARY)
        style.configure('CardHeader.TLabel', background=config.BG_CARD, font=('Helvetica', 11, 'bold'), foreground=config.TEXT_MUTED)
        style.configure('CardValue.TLabel', background=config.BG_CARD, font=('Helvetica', 16, 'bold'), foreground=config.ACCENT_PRIMARY)
        style.configure('Status.TLabel', font=('Helvetica', 10, 'bold'))

        # Button styles
        style.configure('TButton', font=('Helvetica', 10, 'bold'), padding=6)
        style.configure('Accent.TButton', background=config.ACCENT_PRIMARY, foreground='#ffffff')
        style.configure('Danger.TButton', background=config.ACCENT_DANGER, foreground='#ffffff')

        # Combobox style
        style.configure('TCombobox', fieldbackground=config.BG_CARD_LIGHT, background=config.BG_CARD_LIGHT, foreground=config.TEXT_PRIMARY)

    def _build_layout(self):
        # Main Grid Layout (2 columns: Video on Left, Controls on Right)
        self.main_container = ttk.Frame(self.root, padding=12)
        self.main_container.pack(fill=tk.BOTH, expand=True)

        self.main_container.columnconfigure(0, weight=3)
        self.main_container.columnconfigure(1, weight=2)
        self.main_container.rowconfigure(0, weight=1)

        # ----------------------------------------------------
        # LEFT COLUMN: Video Stream & Overlay
        # ----------------------------------------------------
        left_frame = ttk.Frame(self.main_container)
        left_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

        # Title Header
        title_label = ttk.Label(left_frame, text="AI GESTURE CONTROL CENTER", style='Header.TLabel')
        title_label.pack(anchor="w", pady=(0, 8))

        # Canvas for video frame
        self.video_canvas = tk.Canvas(
            left_frame, 
            width=config.FRAME_WIDTH, 
            height=config.FRAME_HEIGHT, 
            bg="#000000", 
            highlightthickness=1,
            highlightbackground="#2c3e50"
        )
        self.video_canvas.pack(fill=tk.BOTH, expand=True)

        # Bottom Bar under Video for System Action Buttons
        video_btn_bar = ttk.Frame(left_frame, padding=(0, 8, 0, 0))
        video_btn_bar.pack(fill=tk.X)

        self.btn_start_stop = tk.Button(
            video_btn_bar, 
            text="▶ START SYSTEM", 
            font=('Helvetica', 11, 'bold'),
            bg=config.ACCENT_SUCCESS, 
            fg="#ffffff", 
            activebackground="#27ae60",
            relief=tk.FLAT,
            command=self._toggle_system_state
        )
        self.btn_start_stop.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))

        self.btn_emergency = tk.Button(
            video_btn_bar, 
            text="🚨 EMERGENCY STOP 🚨", 
            font=('Helvetica', 12, 'bold'),
            bg=config.ACCENT_DANGER, 
            fg="#ffffff", 
            activebackground="#c0392b",
            relief=tk.FLAT,
            command=self._on_emergency_click
        )
        self.btn_emergency.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(5, 0))

        # ----------------------------------------------------
        # RIGHT COLUMN: Status, Telemetry & Settings
        # ----------------------------------------------------
        right_frame = ttk.Frame(self.main_container)
        right_frame.grid(row=0, column=1, sticky="nsew")

        # --- 1. Serial Port Selector Card ---
        serial_card = ttk.Frame(right_frame, style='Card.TFrame', padding=10)
        serial_card.pack(fill=tk.X, pady=(0, 8))

        ttk.Label(serial_card, text="SERIAL CONNECTION", style='CardHeader.TLabel').pack(anchor="w")

        serial_row = ttk.Frame(serial_card, style='Card.TFrame')
        serial_row.pack(fill=tk.X, pady=(5, 0))

        self.port_combo = ttk.Combobox(serial_row, state="readonly", width=18)
        self.port_combo.pack(side=tk.LEFT, padx=(0, 5))

        self.btn_refresh_ports = ttk.Button(serial_row, text="🔄", width=3, command=self._refresh_serial_ports)
        self.btn_refresh_ports.pack(side=tk.LEFT, padx=(0, 5))

        self.btn_connect_serial = ttk.Button(serial_row, text="Connect", command=self._toggle_serial_connection)
        self.btn_connect_serial.pack(side=tk.LEFT, fill=tk.X, expand=True)

        self.lbl_serial_status = ttk.Label(serial_card, text="Disconnected 🔴", style='Status.TLabel', background=config.BG_CARD, foreground=config.ACCENT_DANGER)
        self.lbl_serial_status.pack(anchor="w", pady=(5, 0))

        # --- 2. Live Status Cards (Grid 2x2) ---
        stats_grid = ttk.Frame(right_frame)
        stats_grid.pack(fill=tk.X, pady=(0, 8))
        stats_grid.columnconfigure(0, weight=1)
        stats_grid.columnconfigure(1, weight=1)

        # Card: Detected Gesture
        card_gesture = ttk.Frame(stats_grid, style='Card.TFrame', padding=10)
        card_gesture.grid(row=0, column=0, sticky="nsew", padx=(0, 4), pady=4)
        ttk.Label(card_gesture, text="DETECTED GESTURE", style='CardHeader.TLabel').pack(anchor="w")
        self.lbl_gesture_val = ttk.Label(card_gesture, text="🚫 NONE", style='CardValue.TLabel')
        self.lbl_gesture_val.pack(anchor="w", pady=(4, 0))

        # Card: Active Car Command
        card_cmd = ttk.Frame(stats_grid, style='Card.TFrame', padding=10)
        card_cmd.grid(row=0, column=1, sticky="nsew", padx=(4, 0), pady=4)
        ttk.Label(card_cmd, text="CAR COMMAND", style='CardHeader.TLabel').pack(anchor="w")
        self.lbl_command_val = ttk.Label(card_cmd, text="✋ STOP [S]", style='CardValue.TLabel')
        self.lbl_command_val.pack(anchor="w", pady=(4, 0))

        # Card: Confidence %
        card_conf = ttk.Frame(stats_grid, style='Card.TFrame', padding=10)
        card_conf.grid(row=1, column=0, sticky="nsew", padx=(0, 4), pady=4)
        ttk.Label(card_conf, text="CONFIDENCE", style='CardHeader.TLabel').pack(anchor="w")
        self.lbl_conf_val = ttk.Label(card_conf, text="0.0%", style='CardValue.TLabel')
        self.lbl_conf_val.pack(anchor="w", pady=(4, 0))

        # Card: System FPS
        card_fps = ttk.Frame(stats_grid, style='Card.TFrame', padding=10)
        card_fps.grid(row=1, column=1, sticky="nsew", padx=(4, 0), pady=4)
        ttk.Label(card_fps, text="CAMERA FPS", style='CardHeader.TLabel').pack(anchor="w")
        self.lbl_fps_val = ttk.Label(card_fps, text="0.0 FPS", style='CardValue.TLabel')
        self.lbl_fps_val.pack(anchor="w", pady=(4, 0))

        # --- 3. Gesture & Car Settings Card ---
        settings_card = ttk.Frame(right_frame, style='Card.TFrame', padding=10)
        settings_card.pack(fill=tk.X, pady=(0, 8))

        ttk.Label(settings_card, text="SETTINGS & TUNING", style='CardHeader.TLabel').pack(anchor="w", pady=(0, 5))

        # Confidence Threshold Slider
        conf_row = ttk.Frame(settings_card, style='Card.TFrame')
        conf_row.pack(fill=tk.X, pady=2)
        ttk.Label(conf_row, text="Min Confidence:", background=config.BG_CARD).pack(side=tk.LEFT)
        self.lbl_conf_setting = ttk.Label(conf_row, text="0.70", background=config.BG_CARD, font=('Helvetica', 9, 'bold'))
        self.lbl_conf_setting.pack(side=tk.RIGHT)
        self.slider_conf = ttk.Scale(settings_card, from_=0.50, to=0.95, value=0.70, command=self._on_conf_slider_change)
        self.slider_conf.pack(fill=tk.X, pady=(0, 5))

        # Stability Frame Threshold Slider
        stab_row = ttk.Frame(settings_card, style='Card.TFrame')
        stab_row.pack(fill=tk.X, pady=2)
        ttk.Label(stab_row, text="Stability Buffer (Frames):", background=config.BG_CARD).pack(side=tk.LEFT)
        self.lbl_stab_setting = ttk.Label(stab_row, text="5", background=config.BG_CARD, font=('Helvetica', 9, 'bold'))
        self.lbl_stab_setting.pack(side=tk.RIGHT)
        self.slider_stab = ttk.Scale(settings_card, from_=1, to=10, value=5, command=self._on_stab_slider_change)
        self.slider_stab.pack(fill=tk.X, pady=(0, 5))

        # Motor Speed PWM Slider
        speed_row = ttk.Frame(settings_card, style='Card.TFrame')
        speed_row.pack(fill=tk.X, pady=2)
        ttk.Label(speed_row, text="Motor PWM Speed:", background=config.BG_CARD).pack(side=tk.LEFT)
        self.lbl_speed_setting = ttk.Label(speed_row, text="220", background=config.BG_CARD, font=('Helvetica', 9, 'bold'))
        self.lbl_speed_setting.pack(side=tk.RIGHT)
        self.slider_speed = ttk.Scale(settings_card, from_=100, to=255, value=220, command=self._on_speed_slider_change)
        self.slider_speed.pack(fill=tk.X)

        # --- 4. Serial Command Telemetry Log ---
        log_card = ttk.Frame(right_frame, style='Card.TFrame', padding=10)
        log_card.pack(fill=tk.BOTH, expand=True)

        log_header = ttk.Frame(log_card, style='Card.TFrame')
        log_header.pack(fill=tk.X, pady=(0, 5))
        ttk.Label(log_header, text="TELEMETRY & COMMAND LOG", style='CardHeader.TLabel').pack(side=tk.LEFT)
        btn_clear = ttk.Button(log_header, text="Clear", width=6, command=self._clear_log)
        btn_clear.pack(side=tk.RIGHT)

        self.log_text = tk.Text(
            log_card, 
            height=8, 
            bg="#0d0d0f", 
            fg=config.TEXT_PRIMARY, 
            font=('Courier', 9),
            wrap=tk.WORD,
            relief=tk.FLAT,
            highlightthickness=1,
            highlightbackground="#2c3e50"
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def _refresh_serial_ports(self):
        ports = self.serial_manager.list_ports()
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.current(0)
        else:
            self.port_combo.set("No Ports Found")

    def _toggle_serial_connection(self):
        if self.serial_manager.is_connected:
            self.serial_manager.disconnect()
            self.btn_connect_serial.config(text="Connect")
        else:
            selected_port = self.port_combo.get()
            if not selected_port or selected_port == "No Ports Found":
                messagebox.showwarning("Serial Warning", "Please select a valid serial port.")
                return

            success, msg = self.serial_manager.connect(selected_port)
            if success:
                self.btn_connect_serial.config(text="Disconnect")
                self._log_entry("SYSTEM", f"Connected to {selected_port}")
            else:
                messagebox.showerror("Serial Error", msg)

    def _toggle_system_state(self):
        if self.car_controller.is_active:
            self.car_controller.stop()
            self.btn_start_stop.config(text="▶ START SYSTEM", bg=config.ACCENT_SUCCESS)
            self._log_entry("SYSTEM", "System Stopped by User")
        else:
            self.car_controller.start()
            self.btn_start_stop.config(text="⏸ PAUSE SYSTEM", bg=config.ACCENT_WARNING)
            self._log_entry("SYSTEM", "System Started")

    def _on_emergency_click(self):
        if self.car_controller.emergency_override:
            self.car_controller.clear_emergency_stop()
            self.btn_emergency.config(text="🚨 EMERGENCY STOP 🚨", bg=config.ACCENT_DANGER)
            self._log_entry("SYSTEM", "Emergency Stop Cleared")
        else:
            self.car_controller.trigger_emergency_stop()
            self.btn_emergency.config(text="CLEAR EMERGENCY ⚠️", bg=config.ACCENT_WARNING)
            self._log_entry("EMERGENCY", "EMERGENCY STOP ACTIVATED VIA GUI BUTTON")

    def _on_conf_slider_change(self, val):
        conf = round(float(val), 2)
        self.lbl_conf_setting.config(text=f"{conf:.2f}")
        self.hand_detector.set_confidence_thresholds(conf, conf)

    def _on_stab_slider_change(self, val):
        stab = int(float(val))
        self.lbl_stab_setting.config(text=str(stab))
        self.gesture_classifier.set_stability_threshold(stab)

    def _on_speed_slider_change(self, val):
        speed = int(float(val))
        self.lbl_speed_setting.config(text=str(speed))
        self.car_controller.set_motor_speed(speed)

    def _on_serial_telemetry(self, direction, message):
        self.root.after(0, self._log_entry, direction, message)

    def _on_serial_status_change(self, is_connected, status_text):
        def update():
            if is_connected:
                self.lbl_serial_status.config(text=f"Connected 🟢 ({self.serial_manager.current_port})", foreground=config.ACCENT_SUCCESS)
                self.btn_connect_serial.config(text="Disconnect")
            else:
                self.lbl_serial_status.config(text="Disconnected 🔴", foreground=config.ACCENT_DANGER)
                self.btn_connect_serial.config(text="Connect")
        self.root.after(0, update)

    def _log_entry(self, tag, text):
        timestamp = time.strftime("%H:%M:%S")
        entry = f"[{timestamp}] [{tag}] {text}\n"
        self.log_text.insert(tk.END, entry)
        self.log_text.see(tk.END)

    def _clear_log(self):
        self.log_text.delete('1.0', tk.END)

    def _update_loop(self):
        if not self.is_running:
            return

        # 1. Read latest video frame
        ret, frame, fps = self.camera_stream.read()

        candidate_gesture = "NONE"
        stable_cmd = config.CMD_STOP
        is_stable = False
        detected = False
        confidence = 0.0

        if ret and frame is not None:
            min_conf = float(self.slider_conf.get())
            # 2. Process Hand Detection & Landmark Extraction
            detected, landmarks_list, pixel_landmarks, confidence, hand_type = self.hand_detector.process_frame(frame)

            # 3. Classify Gesture & Apply Temporal Smoothing
            candidate_gesture, stable_cmd, is_stable = self.gesture_classifier.process(
                detected, landmarks_list, confidence, min_confidence=min_conf
            )

            # 4. Evaluate Car Safety Controller & Send Command
            active_car_cmd = self.car_controller.update_gesture_command(
                stable_cmd, is_stable, detected, ret
            )

            # 5. Draw sleek landmarks & bounding box on frame
            gesture_meta = config.GESTURE_METADATA.get(candidate_gesture, config.GESTURE_METADATA["NONE"])
            # Convert hex color to BGR for OpenCV
            hex_col = gesture_meta[2].lstrip('#')
            bgr_col = tuple(int(hex_col[i:i+2], 16) for i in (4, 2, 0))
            
            frame = self.hand_detector.draw_landmarks(frame, pixel_landmarks, gesture_meta[0], bgr_col)

            # 6. Render frame to canvas (reuse image ID for performance)
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(rgb_frame)
            imgtk = ImageTk.PhotoImage(image=img)
            
            if not hasattr(self, '_canvas_img_id') or self._canvas_img_id is None:
                self._canvas_img_id = self.video_canvas.create_image(0, 0, anchor=tk.NW, image=imgtk)
            else:
                self.video_canvas.itemconfig(self._canvas_img_id, image=imgtk)
            self.video_canvas.imgtk = imgtk
        else:
            # Camera failure handling -> trigger STOP
            active_car_cmd = self.car_controller.update_gesture_command(
                config.CMD_STOP, True, False, False
            )

        # 7. Update UI Labels & Status Cards
        g_meta = config.GESTURE_METADATA.get(candidate_gesture, config.GESTURE_METADATA["NONE"])
        self.lbl_gesture_val.config(text=f"{g_meta[1]} {g_meta[0]}", foreground=g_meta[2])

        c_meta = config.GESTURE_METADATA.get(active_car_cmd, config.GESTURE_METADATA[config.CMD_STOP])
        self.lbl_command_val.config(text=f"{c_meta[1]} {c_meta[0]} [{active_car_cmd}]", foreground=c_meta[2])

        conf_pct = confidence * 100.0 if detected else 0.0
        self.lbl_conf_val.config(text=f"{conf_pct:.1f}%")
        self.lbl_fps_val.config(text=f"{fps:.1f} FPS")

        # Schedule next UI update (~60 FPS => 16ms)
        self.root.after(16, self._update_loop)

    def _on_close(self):
        self.is_running = False
        if self.car_controller:
            self.car_controller.stop()
        if self.serial_manager:
            self.serial_manager.disconnect()
        if self.camera_stream:
            self.camera_stream.stop()
        self.root.destroy()

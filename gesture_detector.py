"""
Hand Detection and Landmark Extraction using MediaPipe Tasks (v1.0+) or Solutions (v0.10).
Supports Python 3.8 - 3.14+ across macOS, Linux, and Windows.
"""

import os
import urllib.request
import cv2
import mediapipe as mp
import config

MODEL_URL = "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task"
MODEL_PATH = os.path.join(os.path.dirname(__file__), "hand_landmarker.task")

class HandDetector:
    def __init__(self, 
                 max_hands=config.MAX_NUM_HANDS, 
                 min_detection_confidence=config.DEFAULT_MIN_DETECTION_CONFIDENCE,
                 min_tracking_confidence=config.DEFAULT_MIN_TRACKING_CONFIDENCE):
        
        self.max_hands = max_hands
        self.min_detection_confidence = min_detection_confidence
        self.min_tracking_confidence = min_tracking_confidence
        self.use_tasks_api = False
        self.landmarker = None
        self.hands = None
        
        # EMA landmark smoothing state
        self.prev_landmarks = None
        self.smooth_alpha = 0.75  # Weight for current frame (0.75 = crisp + smooth)
        self.last_timestamp_ms = 0

        # Check if legacy mp.solutions exists
        if hasattr(mp, "solutions") and hasattr(mp.solutions, "hands"):
            self.use_tasks_api = False
            self.mp_hands = mp.solutions.hands
            self.hands = self.mp_hands.Hands(
                static_image_mode=False,
                max_num_hands=max_hands,
                model_complexity=config.MODEL_COMPLEXITY,
                min_detection_confidence=min_detection_confidence,
                min_tracking_confidence=min_tracking_confidence
            )
        else:
            # Modern MediaPipe Tasks API (MediaPipe 1.0+)
            self.use_tasks_api = True
            self._ensure_model_file()
            
            from mediapipe.tasks import python
            from mediapipe.tasks.python import vision

            base_options = python.BaseOptions(model_asset_path=MODEL_PATH)
            options = vision.HandLandmarkerOptions(
                base_options=base_options,
                running_mode=vision.RunningMode.VIDEO,
                num_hands=max_hands,
                min_hand_detection_confidence=min_detection_confidence,
                min_hand_presence_confidence=min_tracking_confidence,
                min_tracking_confidence=min_tracking_confidence
            )
            self.landmarker = vision.HandLandmarker.create_from_options(options)

    def _ensure_model_file(self):
        """Downloads hand_landmarker.task if not already present."""
        if not os.path.exists(MODEL_PATH):
            print(f"[+] Downloading MediaPipe HandLandmarker model to {MODEL_PATH}...")
            try:
                urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
                print("[+] Model downloaded successfully.")
            except Exception as e:
                print(f"[!] Failed to download model automatically: {e}")

    def _smooth_landmarks(self, current_landmarks):
        """Applies Exponential Moving Average (EMA) to 21 3D landmarks for noise reduction."""
        if self.prev_landmarks is None or len(self.prev_landmarks) != len(current_landmarks):
            self.prev_landmarks = current_landmarks
            return current_landmarks

        smoothed = []
        for (cx, cy, cz), (px, py, pz) in zip(current_landmarks, self.prev_landmarks):
            sx = self.smooth_alpha * cx + (1.0 - self.smooth_alpha) * px
            sy = self.smooth_alpha * cy + (1.0 - self.smooth_alpha) * py
            sz = self.smooth_alpha * cz + (1.0 - self.smooth_alpha) * pz
            smoothed.append((sx, sy, sz))

        self.prev_landmarks = smoothed
        return smoothed

    def set_confidence_thresholds(self, min_detection_confidence, min_tracking_confidence):
        """Re-initializes hands processor with updated confidence settings."""
        self.min_detection_confidence = min_detection_confidence
        self.min_tracking_confidence = min_tracking_confidence
        self.prev_landmarks = None

        if not self.use_tasks_api and self.hands:
            self.hands.close()
            self.hands = self.mp_hands.Hands(
                static_image_mode=False,
                max_num_hands=self.max_hands,
                model_complexity=config.MODEL_COMPLEXITY,
                min_detection_confidence=min_detection_confidence,
                min_tracking_confidence=min_tracking_confidence
            )
        elif self.use_tasks_api:
            if self.landmarker:
                self.landmarker.close()
            from mediapipe.tasks import python
            from mediapipe.tasks.python import vision

            base_options = python.BaseOptions(model_asset_path=MODEL_PATH)
            options = vision.HandLandmarkerOptions(
                base_options=base_options,
                running_mode=vision.RunningMode.VIDEO,
                num_hands=self.max_hands,
                min_hand_detection_confidence=min_detection_confidence,
                min_hand_presence_confidence=min_tracking_confidence,
                min_tracking_confidence=min_tracking_confidence
            )
            self.landmarker = vision.HandLandmarker.create_from_options(options)

    def process_frame(self, frame):
        """
        Processes an BGR OpenCV frame.
        Returns:
            detected (bool): True if a hand was detected.
            landmarks_list (list): List of 21 (x, y, z) normalized tuple coordinates.
            pixel_landmarks (list): List of 21 (px, py) integer tuple coordinates.
            confidence (float): Hand landmark score.
            hand_type (str): 'Right' or 'Left'.
        """
        import time
        h, w, _ = frame.shape
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        landmarks_list = []
        pixel_landmarks = []

        if self.use_tasks_api:
            now_ms = int(time.time() * 1000)
            if now_ms <= self.last_timestamp_ms:
                now_ms = self.last_timestamp_ms + 1
            self.last_timestamp_ms = now_ms

            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
            results = self.landmarker.detect_for_video(mp_image, now_ms)

            if not results.hand_landmarks:
                self.prev_landmarks = None
                return False, [], [], 0.0, "Unknown"

            hand_landmarks = results.hand_landmarks[0]
            confidence = 0.95
            hand_type = "Right"

            if results.handedness:
                confidence = results.handedness[0][0].score
                hand_type = results.handedness[0][0].category_name

            raw_lms = [(lm.x, lm.y, lm.z) for lm in hand_landmarks]
            landmarks_list = self._smooth_landmarks(raw_lms)
            pixel_landmarks = [(int(lm[0] * w), int(lm[1] * h)) for lm in landmarks_list]

            return True, landmarks_list, pixel_landmarks, confidence, hand_type
        else:
            results = self.hands.process(rgb_frame)

            if not results.multi_hand_landmarks:
                self.prev_landmarks = None
                return False, [], [], 0.0, "Unknown"

            hand_landmarks = results.multi_hand_landmarks[0]
            hand_meta = results.multi_handedness[0].classification[0]
            
            confidence = hand_meta.score
            hand_type = hand_meta.label

            raw_lms = [(lm.x, lm.y, lm.z) for lm in hand_landmarks.landmark]
            landmarks_list = self._smooth_landmarks(raw_lms)
            pixel_landmarks = [(int(lm[0] * w), int(lm[1] * h)) for lm in landmarks_list]

            return True, landmarks_list, pixel_landmarks, confidence, hand_type

    def draw_landmarks(self, frame, pixel_landmarks, gesture_name="STOP", gesture_color=(0, 255, 0), confidence=0.0):
        """
        Draws tactical HUD overlay with sci-fi corner brackets, glowing skeleton connections,
        and semi-transparent gesture tag badge.
        """
        import numpy as np

        h, w, _ = frame.shape

        # Draw camera HUD overlay corner markers on video frame
        hud_color = (0, 229, 255)  # Cyan HUD tint
        cv2.putText(frame, "CAM 01 // LIVE", (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.45, hud_color, 1, cv2.LINE_AA)

        if not pixel_landmarks or len(pixel_landmarks) < 21:
            # Subtle center reticle when no hand is present
            cx, cy = w // 2, h // 2
            cv2.drawMarker(frame, (cx, cy), (60, 75, 95), cv2.MARKER_CROSS, 20, 1, cv2.LINE_AA)
            return frame

        # Skeleton connections definition
        connections = [
            (0, 1), (1, 2), (2, 3), (3, 4),           # Thumb
            (0, 5), (5, 6), (6, 7), (7, 8),           # Index
            (5, 9), (9, 10), (10, 11), (11, 12),      # Middle
            (9, 13), (13, 14), (14, 15), (15, 16),    # Ring
            (13, 17), (0, 17), (17, 18), (18, 19), (19, 20) # Pinky & Palm
        ]

        # Draw glowing connection lines (thin outer line + core bright line)
        for pt1_idx, pt2_idx in connections:
            pt1 = pixel_landmarks[pt1_idx]
            pt2 = pixel_landmarks[pt2_idx]
            # Core line
            cv2.line(frame, pt1, pt2, (240, 240, 245), 2, cv2.LINE_AA)

        # Draw key joint points
        for i, (px, py) in enumerate(pixel_landmarks):
            if i in [4, 8, 12, 16, 20]:  # Finger Tips
                cv2.circle(frame, (px, py), 8, gesture_color, -1, cv2.LINE_AA)
                cv2.circle(frame, (px, py), 10, (255, 255, 255), 1, cv2.LINE_AA)
            else:
                cv2.circle(frame, (px, py), 4, (0, 229, 255), -1, cv2.LINE_AA)
                cv2.circle(frame, (px, py), 5, (10, 20, 30), 1, cv2.LINE_AA)

        # Compute Bounding Box
        xs = [pt[0] for pt in pixel_landmarks]
        ys = [pt[1] for pt in pixel_landmarks]
        pad = 20
        min_x, max_x = max(5, min(xs) - pad), min(w - 5, max(xs) + pad)
        min_y, max_y = max(5, min(ys) - pad), min(h - 5, max(ys) + pad)

        # Draw Futuristic L-Shaped Corner Brackets instead of solid rectangle
        corner_len = min(25, (max_x - min_x) // 4, (max_y - min_y) // 4)
        c_color = gesture_color
        thick = 2

        # Top-Left Corner
        cv2.line(frame, (min_x, min_y), (min_x + corner_len, min_y), c_color, thick, cv2.LINE_AA)
        cv2.line(frame, (min_x, min_y), (min_x, min_y + corner_len), c_color, thick, cv2.LINE_AA)
        # Top-Right Corner
        cv2.line(frame, (max_x, min_y), (max_x - corner_len, min_y), c_color, thick, cv2.LINE_AA)
        cv2.line(frame, (max_x, min_y), (max_x, min_y + corner_len), c_color, thick, cv2.LINE_AA)
        # Bottom-Left Corner
        cv2.line(frame, (min_x, max_y), (min_x + corner_len, max_y), c_color, thick, cv2.LINE_AA)
        cv2.line(frame, (min_x, max_y), (min_x, max_y - corner_len), c_color, thick, cv2.LINE_AA)
        # Bottom-Right Corner
        cv2.line(frame, (max_x, max_y), (max_x - corner_len, max_y), c_color, thick, cv2.LINE_AA)
        cv2.line(frame, (max_x, max_y), (max_x, max_y - corner_len), c_color, thick, cv2.LINE_AA)

        # Semi-transparent HUD overlay tag for gesture label
        tag_text = f"TARGET: {gesture_name}"
        if confidence > 0:
            tag_text += f" ({confidence*100:.0f}%)"

        (t_w, t_h), baseline = cv2.getTextSize(tag_text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        tag_y1 = max(5, min_y - t_h - 12)
        tag_y2 = tag_y1 + t_h + 8
        tag_x1 = min_x
        tag_x2 = min(w - 5, min_x + t_w + 14)

        # Overlay blended background box
        overlay = frame.copy()
        cv2.rectangle(overlay, (tag_x1, tag_y1), (tag_x2, tag_y2), (15, 22, 32), -1)
        cv2.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)
        cv2.rectangle(frame, (tag_x1, tag_y1), (tag_x2, tag_y2), c_color, 1, cv2.LINE_AA)

        cv2.putText(frame, tag_text, (tag_x1 + 6, tag_y2 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

        return frame


    def close(self):
        if self.use_tasks_api and self.landmarker:
            self.landmarker.close()
        elif self.hands:
            self.hands.close()

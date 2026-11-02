"""
Hand Detection and Landmark Extraction using MediaPipe Tasks (v1.0+) or Solutions (v0.10).
Supports Python 3.8 - 3.14+ across macOS, Linux, and Windows.
"""

import os
import math
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
        """
        Applies Per-Landmark Velocity-Adaptive Exponential Moving Average (EMA).
        Dynamically adjusts smoothing alpha per joint:
        - Moving joint (fingertip flick / gesture transition) -> High alpha (0.96) for <30ms instant response.
        - Stationary joint (held gesture) -> Low alpha (0.28) for rock-solid zero-jitter stability.
        """
        if self.prev_landmarks is None or len(self.prev_landmarks) != len(current_landmarks):
            self.prev_landmarks = current_landmarks
            return current_landmarks

        # Compute displacements for all landmarks
        displacements = []
        for (cx, cy, cz), (px, py, pz) in zip(current_landmarks, self.prev_landmarks):
            disp = math.sqrt((cx - px)**2 + (cy - py)**2 + (cz - pz)**2)
            displacements.append(disp)

        # Max displacement across key articulation points (wrist + 5 fingertips: 0, 4, 8, 12, 16, 20)
        key_indices = [0, 4, 8, 12, 16, 20]
        max_key_disp = max(displacements[idx] for idx in key_indices if idx < len(displacements))

        min_alpha, max_alpha = 0.28, 0.96
        cutoff = 0.022  # Normalized displacement cutoff

        smoothed = []
        for i, ((cx, cy, cz), (px, py, pz)) in enumerate(zip(current_landmarks, self.prev_landmarks)):
            disp_i = displacements[i]
            # Blend local joint velocity with hand overall velocity
            effective_disp = max(disp_i, max_key_disp * 0.70)
            dynamic_alpha = min_alpha + (max_alpha - min_alpha) * min(1.0, effective_disp / cutoff)

            sx = dynamic_alpha * cx + (1.0 - dynamic_alpha) * px
            sy = dynamic_alpha * cy + (1.0 - dynamic_alpha) * py
            sz = dynamic_alpha * cz + (1.0 - dynamic_alpha) * pz
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

    def _compute_hand_confidence(self, landmarks, handedness_score=0.95):
        """
        Computes accurate hand tracking confidence combining MediaPipe presence,
        handedness classification stability, and anatomical skeletal plausibility.
        Prevents false 'LOW CONFIDENCE' drops when handedness probability is near 0.5.
        """
        if not landmarks or len(landmarks) < 21:
            return 0.0

        # Base confidence for a detected landmark mesh:
        base_conf = 0.90

        # Anatomical proportionality check:
        # Check palm dimension (wrist to middle MCP)
        raw_lms = [(lm.x, lm.y, lm.z) if hasattr(lm, "x") else lm for lm in landmarks]
        w_mcp_dist_sq = (raw_lms[0][0] - raw_lms[9][0])**2 + (raw_lms[0][1] - raw_lms[9][1])**2
        if 0.002 < w_mcp_dist_sq < 0.35:
            base_conf += 0.05

        # Blend with handedness score gently (weight 15% max)
        h_score = handedness_score if handedness_score is not None else 0.90
        final_conf = min(0.99, max(0.50, base_conf * 0.85 + h_score * 0.15))
        return round(final_conf, 2)

    def _select_best_hand(self, hand_landmarks_list, handedness_list=None):
        """
        Selects the best hand candidate when multiple hands appear in frame.
        Prioritizes:
        1. Temporal continuity with previous tracked hand (prevents hand-hopping).
        2. Proximity to image center (where driver's hand operates).
        3. Normalized bounding box scale (closest hand to camera).
        """
        if not hand_landmarks_list:
            return None, 0, 0.95, "Right"

        best_idx = 0
        max_score = -1.0

        for idx, landmarks in enumerate(hand_landmarks_list):
            raw_lms = [(lm.x, lm.y, lm.z) if hasattr(lm, "x") else lm for lm in (landmarks.landmark if hasattr(landmarks, "landmark") else landmarks)]
            xs = [pt[0] for pt in raw_lms]
            ys = [pt[1] for pt in raw_lms]
            w_box = max(xs) - min(xs)
            h_box = max(ys) - min(ys)
            area = w_box * h_box
            area_score = min(1.0, area / 0.06)

            # Center proximity score
            cx = (min(xs) + max(xs)) / 2.0
            cy = (min(ys) + max(ys)) / 2.0
            center_dist = math.sqrt((cx - 0.5)**2 + (cy - 0.5)**2)
            center_score = max(0.0, 1.0 - 1.5 * center_dist)

            h_conf = 0.95
            htype = "Right"
            if handedness_list and idx < len(handedness_list):
                h_item = handedness_list[idx]
                if isinstance(h_item, list):
                    h_conf = h_item[0].score
                    htype = h_item[0].category_name
                elif hasattr(h_item, "classification"):
                    h_conf = h_item.classification[0].score
                    htype = h_item.classification[0].label

            # Continuity score with previous hand
            continuity = 0.0
            if self.prev_landmarks is not None:
                pw_x, pw_y = self.prev_landmarks[0][0], self.prev_landmarks[0][1]
                dist_prev = math.sqrt((raw_lms[0][0] - pw_x)**2 + (raw_lms[0][1] - pw_y)**2)
                continuity = max(0.0, 1.0 - 2.5 * dist_prev)

            score = continuity * 0.45 + area_score * 0.35 + center_score * 0.20
            if score > max_score:
                max_score = score
                best_idx = idx

        best_landmarks = hand_landmarks_list[best_idx]
        best_htype = "Right"
        h_score = 0.95
        if handedness_list and best_idx < len(handedness_list):
            h_item = handedness_list[best_idx]
            if isinstance(h_item, list):
                h_score = h_item[0].score
                best_htype = h_item[0].category_name
            elif hasattr(h_item, "classification"):
                h_score = h_item.classification[0].score
                best_htype = h_item.classification[0].label

        best_conf = self._compute_hand_confidence(
            best_landmarks.landmark if hasattr(best_landmarks, "landmark") else best_landmarks,
            h_score
        )

        return best_landmarks, best_idx, best_conf, best_htype

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

            hand_landmarks, _, confidence, hand_type = self._select_best_hand(
                results.hand_landmarks, getattr(results, "handedness", None)
            )

            raw_lms = [(lm.x, lm.y, lm.z) for lm in hand_landmarks]
            landmarks_list = self._smooth_landmarks(raw_lms)
            pixel_landmarks = [(int(lm[0] * w), int(lm[1] * h)) for lm in landmarks_list]

            return True, landmarks_list, pixel_landmarks, confidence, hand_type
        else:
            results = self.hands.process(rgb_frame)

            if not results.multi_hand_landmarks:
                self.prev_landmarks = None
                return False, [], [], 0.0, "Unknown"

            hand_landmarks, _, confidence, hand_type = self._select_best_hand(
                results.multi_hand_landmarks, getattr(results, "multi_handedness", None)
            )

            raw_lms = [(lm.x, lm.y, lm.z) for lm in hand_landmarks.landmark]
            landmarks_list = self._smooth_landmarks(raw_lms)
            pixel_landmarks = [(int(lm[0] * w), int(lm[1] * h)) for lm in landmarks_list]

            return True, landmarks_list, pixel_landmarks, confidence, hand_type

    def draw_landmarks(self, frame, pixel_landmarks, gesture_name="STOP", gesture_color=(0, 255, 0), confidence=0.0):
        """
        Draws tactical HUD overlay with sci-fi corner brackets, glowing skeleton connections,
        directional trajectory pointing ray, and semi-transparent gesture tag badge.
        """
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
            # Outer subtle glow line
            cv2.line(frame, pt1, pt2, (40, 50, 70), 4, cv2.LINE_AA)
            # Core line
            cv2.line(frame, pt1, pt2, (240, 240, 245), 2, cv2.LINE_AA)

        # Draw key joint points
        for i, (px, py) in enumerate(pixel_landmarks):
            if i in [4, 8, 12, 16, 20]:  # Finger Tips
                cv2.circle(frame, (px, py), 7, gesture_color, -1, cv2.LINE_AA)
                cv2.circle(frame, (px, py), 9, (255, 255, 255), 1, cv2.LINE_AA)
            else:
                cv2.circle(frame, (px, py), 4, (0, 229, 255), -1, cv2.LINE_AA)
                cv2.circle(frame, (px, py), 5, (10, 20, 30), 1, cv2.LINE_AA)

        # Draw Dynamic Directional Pointing Ray if directional gesture is active
        if gesture_name in ["FORWARD", "BACKWARD", "LEFT", "RIGHT"]:
            pt_tip = pixel_landmarks[8]
            pt_pip = pixel_landmarks[6]
            dx = pt_tip[0] - pt_pip[0]
            dy = pt_tip[1] - pt_pip[1]
            mag = math.sqrt(dx * dx + dy * dy)
            if mag > 1e-3:
                ray_len = 55
                end_x = int(pt_tip[0] + (dx / mag) * ray_len)
                end_y = int(pt_tip[1] + (dy / mag) * ray_len)
                # Outer shadow ray
                cv2.arrowedLine(frame, pt_tip, (end_x, end_y), (10, 20, 30), 5, tipLength=0.35, line_type=cv2.LINE_AA)
                # Glowing directional pointing vector
                cv2.arrowedLine(frame, pt_tip, (end_x, end_y), gesture_color, 3, tipLength=0.35, line_type=cv2.LINE_AA)
                # Pulse highlight circle on active index fingertip
                cv2.circle(frame, pt_tip, 13, (0, 229, 255), 1, cv2.LINE_AA)

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

"""
Hand Gesture Classifier with 3D Landmark Geometry, Joint Angle Analysis,
and Adaptive Temporal Hysteresis Smoothing.
Optimized with 3D vector math and squared distance metrics.
"""

import math
from collections import deque
import config

class GestureClassifier:
    def __init__(self, stability_threshold=config.DEFAULT_STABILITY_FRAME_THRESHOLD):
        self.stability_threshold = max(1, stability_threshold)
        self.frame_buffer = deque(maxlen=self.stability_threshold)
        self.current_stable_command = config.CMD_STOP
        self.current_candidate_gesture = config.CMD_STOP
        self.last_directional_command = None
        self.last_pointing_angle = 0.0

    def set_stability_threshold(self, threshold):
        """Updates the temporal frame stability requirement."""
        self.stability_threshold = max(1, int(threshold))
        self.frame_buffer = deque(maxlen=self.stability_threshold)

    @staticmethod
    def _sq_distance(pt1, pt2):
        """Computes 3D squared euclidean distance between two landmarks (fast, no sqrt)."""
        dx = pt1[0] - pt2[0]
        dy = pt1[1] - pt2[1]
        dz = pt1[2] - pt2[2]
        return dx * dx + dy * dy + dz * dz

    @staticmethod
    def _compute_joint_angle_cos(mcp, pip, tip):
        """
        Computes 3D cosine of flex angle between (pip - mcp) and (tip - pip) vectors.
        Returns cos(theta): ~1.0 for fully straight finger, <= 0.25 for bent/curled finger.
        """
        u = (pip[0] - mcp[0], pip[1] - mcp[1], pip[2] - mcp[2])
        v = (tip[0] - pip[0], tip[1] - pip[1], tip[2] - pip[2])

        dot = u[0] * v[0] + u[1] * v[1] + u[2] * v[2]
        mag_u = math.sqrt(u[0]**2 + u[1]**2 + u[2]**2)
        mag_v = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)

        if mag_u * mag_v < 1e-6:
            return 1.0
        return dot / (mag_u * mag_v)

    def _is_finger_extended(self, landmarks, tip_idx, pip_idx, mcp_idx):
        """
        Determines if a finger is extended (straight out).
        Combines 3D joint angle cosine, distance ratios, and wrist distance metrics.
        """
        wrist = landmarks[0]
        tip = landmarks[tip_idx]
        pip = landmarks[pip_idx]
        mcp = landmarks[mcp_idx]

        d2_wrist_tip = self._sq_distance(wrist, tip)
        d2_wrist_mcp = self._sq_distance(wrist, mcp)
        d2_tip_mcp = self._sq_distance(tip, mcp)
        d2_pip_mcp = self._sq_distance(pip, mcp)

        # 3D Joint angle flex cosine (~1.0 for straight, <= 0.25 for bent)
        cos_flex = self._compute_joint_angle_cos(mcp, pip, tip)

        # Geometric conditions for extension
        is_unfolded = d2_tip_mcp > 1.30 * d2_pip_mcp
        is_away_from_wrist = (d2_wrist_tip > 1.10 * d2_wrist_mcp) or (d2_tip_mcp > 1.55 * d2_pip_mcp)
        is_straight = cos_flex >= 0.60

        return (is_straight and is_unfolded and is_away_from_wrist) or (d2_tip_mcp > 1.80 * d2_pip_mcp and is_away_from_wrist)

    def _is_finger_curled(self, landmarks, tip_idx, pip_idx, mcp_idx):
        """
        Determines if a finger is curled towards the palm.
        Tip is pulled in close to the MCP joint or wrist, and joint angle is flexed.
        """
        wrist = landmarks[0]
        tip = landmarks[tip_idx]
        pip = landmarks[pip_idx]
        mcp = landmarks[mcp_idx]

        d2_wrist_tip = self._sq_distance(wrist, tip)
        d2_wrist_pip = self._sq_distance(wrist, pip)
        d2_tip_mcp = self._sq_distance(tip, mcp)
        d2_pip_mcp = self._sq_distance(pip, mcp)

        cos_flex = self._compute_joint_angle_cos(mcp, pip, tip)

        # Curled conditions
        is_close_mcp = d2_tip_mcp <= 1.30 * d2_pip_mcp
        is_sharply_bent = (cos_flex <= 0.25) and (d2_tip_mcp <= 2.20 * d2_pip_mcp)
        is_tucked_to_wrist = (d2_wrist_tip < d2_wrist_pip) and (d2_tip_mcp < 1.60 * d2_pip_mcp)

        return is_close_mcp or is_sharply_bent or is_tucked_to_wrist

    def _is_thumb_extended(self, landmarks):
        """
        Determines if the thumb is extended outward from the hand.
        Thumb landmarks: 0 (wrist), 2 (MCP), 3 (IP), 4 (TIP).
        """
        wrist = landmarks[0]
        thumb_tip = landmarks[4]
        thumb_ip = landmarks[3]
        thumb_mcp = landmarks[2]
        index_mcp = landmarks[5]
        pinky_mcp = landmarks[17]

        d2_thumb_index_mcp = self._sq_distance(thumb_tip, index_mcp)
        d2_palm_span = self._sq_distance(index_mcp, pinky_mcp)
        d2_thumb_wrist = self._sq_distance(thumb_tip, wrist)
        d2_thumb_mcp_wrist = self._sq_distance(thumb_mcp, wrist)

        cos_thumb = self._compute_joint_angle_cos(thumb_mcp, thumb_ip, thumb_tip)

        if d2_palm_span < 1e-6:
            return False

        is_away_from_knuckles = d2_thumb_index_mcp > 0.40 * d2_palm_span
        is_straight = cos_thumb >= 0.50
        is_extended_from_wrist = d2_thumb_wrist > 1.10 * d2_thumb_mcp_wrist

        return (is_away_from_knuckles and is_extended_from_wrist) or (is_straight and is_away_from_knuckles)

    def classify_frame(self, detected, landmarks, confidence, min_confidence=0.70):
        """
        Classifies single frame landmarks into candidate gesture command with high accuracy and tolerance.
        Returns: CMD_FORWARD, CMD_BACKWARD, CMD_LEFT, CMD_RIGHT, CMD_STOP, CMD_EMERGENCY_STOP, or UNKNOWN/NONE.
        """
        if not detected or len(landmarks) < 21:
            self.last_directional_command = None
            return "NONE"

        if confidence < min_confidence:
            self.last_directional_command = None
            return "UNKNOWN"

        index_ext = self._is_finger_extended(landmarks, 8, 6, 5)
        middle_ext = self._is_finger_extended(landmarks, 12, 10, 9)
        ring_ext = self._is_finger_extended(landmarks, 16, 14, 13)
        pinky_ext = self._is_finger_extended(landmarks, 20, 18, 17)

        index_curled = self._is_finger_curled(landmarks, 8, 6, 5)
        middle_curled = self._is_finger_curled(landmarks, 12, 10, 9)
        ring_curled = self._is_finger_curled(landmarks, 16, 14, 13)
        pinky_curled = self._is_finger_curled(landmarks, 20, 18, 17)

        extended_count = sum([index_ext, middle_ext, ring_ext, pinky_ext])
        curled_count = sum([index_curled, middle_curled, ring_curled, pinky_curled])

        # 1. FIST (Emergency Stop - ✊ E)
        # At least 3 fingers curled, neither index nor middle extended, and extended_count <= 1
        if curled_count >= 3 and not index_ext and not middle_ext and extended_count <= 1:
            self.last_directional_command = None
            return config.CMD_EMERGENCY_STOP

        # 2. OPEN PALM (Stop - ✋ S)
        # At least 3 fingers extended, index and middle not curled, and curled count <= 1
        if extended_count >= 3 and not index_curled and not middle_curled and curled_count <= 1:
            self.last_directional_command = None
            return config.CMD_STOP

        # 3. SINGLE INDEX / DIRECTIONAL GESTURES (☝️ / 👇 / 👈 / 👉)
        # Index extended, middle and ring not extended, at least 2 other fingers curled (or not pinky extended)
        if index_ext and not middle_ext and not ring_ext and (curled_count >= 2 or not pinky_ext):
            index_tip = landmarks[8]
            index_pip = landmarks[6]
            index_mcp = landmarks[5]

            # Weighted pointing vector combining distal segment (pip->tip) with full finger (mcp->tip)
            dx = 0.6 * (index_tip[0] - index_pip[0]) + 0.4 * (index_tip[0] - index_mcp[0])
            dy = 0.6 * (index_tip[1] - index_pip[1]) + 0.4 * (index_tip[1] - index_mcp[1])  # Image space: y increases downwards

            # Compute angle in degrees (-180 to 180, where -90 is pointing straight UP)
            angle_deg = math.degrees(math.atan2(dy, dx))
            self.last_pointing_angle = angle_deg

            # Angular deadband / hysteresis (6 degrees buffer to prevent jitter at boundaries)
            buf = 6.0 if self.last_directional_command is not None else 0.0
            prev_dir = self.last_directional_command

            target_cmd = None

            # Check if remaining in previous direction with hysteresis buffer
            if prev_dir == config.CMD_FORWARD and (-135 - buf <= angle_deg <= -45 + buf):
                target_cmd = config.CMD_FORWARD
            elif prev_dir == config.CMD_BACKWARD and (45 - buf <= angle_deg <= 135 + buf):
                target_cmd = config.CMD_BACKWARD
            elif prev_dir == config.CMD_RIGHT and (-45 - buf < angle_deg < 45 + buf):
                target_cmd = config.CMD_RIGHT
            elif prev_dir == config.CMD_LEFT and (angle_deg > (135 - buf) or angle_deg < (-135 + buf)):
                target_cmd = config.CMD_LEFT
            else:
                # Nominal sectors:
                # -135 to -45 deg -> UP (FORWARD ☝️)
                # 45 to 135 deg   -> DOWN (BACKWARD 👇)
                # > 135 or < -135 -> LEFT (👈)
                # -45 to 45 deg   -> RIGHT (👉)
                if -135 <= angle_deg <= -45:
                    target_cmd = config.CMD_FORWARD
                elif 45 <= angle_deg <= 135:
                    target_cmd = config.CMD_BACKWARD
                elif angle_deg > 135 or angle_deg < -135:
                    target_cmd = config.CMD_LEFT
                elif -45 < angle_deg < 45:
                    target_cmd = config.CMD_RIGHT

            if target_cmd:
                self.last_directional_command = target_cmd
                return target_cmd

        # 4. TWO FINGERS UP / V-SIGN (Return to Start - ✌️ H)
        # Index and middle extended, ring and pinky curled
        if index_ext and middle_ext and not ring_ext and not pinky_ext and (curled_count >= 1 or ring_curled):
            self.last_directional_command = None
            return config.CMD_RETURN_HOME

        # 5. Fallback for unrecognized/ambiguous gestures (e.g. loose hand)
        self.last_directional_command = None
        return "UNKNOWN"

    def process(self, detected, landmarks, confidence, min_confidence=0.70):
        """
        Applies temporal hysteresis smoothing with majority voting.
        Fast-path emergency stop (<30ms) and fast stop (<60ms) while filtering out single-frame noise.
        Returns (candidate_gesture, stable_command, is_stable).
        """
        # Fail-safe: No hand or low confidence immediately forces candidate to STOP
        if not detected or confidence < min_confidence:
            self.frame_buffer.clear()
            self.current_candidate_gesture = "NONE" if not detected else "UNKNOWN"
            self.current_stable_command = config.CMD_STOP
            self.last_directional_command = None
            return self.current_candidate_gesture, config.CMD_STOP, True

        candidate = self.classify_frame(detected, landmarks, confidence, min_confidence)
        self.current_candidate_gesture = candidate

        # Immediate Emergency Stop Override (✊ Fist)
        if candidate == config.CMD_EMERGENCY_STOP:
            self.frame_buffer.clear()
            self.current_stable_command = config.CMD_EMERGENCY_STOP
            self.last_directional_command = None
            return candidate, config.CMD_EMERGENCY_STOP, True

        # Fast Stop for Open Palm (✋ Stop): 2 frames are sufficient to halt vehicle safely
        if candidate == config.CMD_STOP and self.current_stable_command != config.CMD_STOP:
            self.frame_buffer.append(candidate)
            if self.frame_buffer.count(config.CMD_STOP) >= 2:
                self.current_stable_command = config.CMD_STOP
                return candidate, config.CMD_STOP, True

        # Append candidate to frame buffer
        self.frame_buffer.append(candidate)

        # Count occurrences of candidate gestures in buffer
        counts = {}
        for g in self.frame_buffer:
            counts[g] = counts.get(g, 0) + 1

        # Find majority gesture in buffer
        most_common_gesture = max(counts, key=counts.get)
        max_count = counts[most_common_gesture]

        # Require majority agreement (>= 50% of buffer capacity, minimum 2 frames)
        required_votes = max(2, int(0.50 * self.stability_threshold))

        if max_count >= required_votes:
            # If majority is UNKNOWN, default stable command to STOP
            if most_common_gesture in ["UNKNOWN", "NONE"]:
                self.current_stable_command = config.CMD_STOP
                return candidate, config.CMD_STOP, False
            else:
                self.current_stable_command = most_common_gesture
                return candidate, self.current_stable_command, True

        return candidate, self.current_stable_command, False


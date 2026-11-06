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
        Classifies single frame landmarks.
        Only keeps the RETURN TO START (✌️) gesture. All other hand gestures are ignored.
        Returns: CMD_RETURN_HOME ('H'), UNKNOWN, or NONE.
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

        ring_curled = self._is_finger_curled(landmarks, 16, 14, 13)
        pinky_curled = self._is_finger_curled(landmarks, 20, 18, 17)
        curled_count = sum([ring_curled, pinky_curled])

        # SOLE ACTIVE GESTURE: Return To Start (✌️ Two Fingers / V-Sign)
        # Index and Middle extended, Ring and Pinky curled
        if index_ext and middle_ext and not ring_ext and not pinky_ext and (curled_count >= 1 or ring_curled):
            self.last_directional_command = None
            return config.CMD_RETURN_HOME

        # All other hand gestures are safely ignored
        self.last_directional_command = None
        return "UNKNOWN"

    def process(self, detected, landmarks, confidence, min_confidence=0.70):
        """
        Applies temporal hysteresis smoothing.
        Exclusively activates RETURN TO START (✌️) while safely ignoring all other gestures.
        Returns (candidate_gesture, stable_command, is_stable).
        """
        if not detected or confidence < min_confidence:
            self.frame_buffer.clear()
            self.current_candidate_gesture = "NONE" if not detected else "UNKNOWN"
            self.current_stable_command = config.CMD_STOP
            self.last_directional_command = None
            return self.current_candidate_gesture, config.CMD_STOP, True

        candidate = self.classify_frame(detected, landmarks, confidence, min_confidence)
        self.current_candidate_gesture = candidate

        # Fast Return-to-Home activation (2 consecutive frames)
        if candidate == config.CMD_RETURN_HOME:
            self.frame_buffer.append(candidate)
            if self.frame_buffer.count(config.CMD_RETURN_HOME) >= 2:
                self.current_stable_command = config.CMD_RETURN_HOME
                return candidate, config.CMD_RETURN_HOME, True
            return candidate, config.CMD_STOP, False

        # All other postures keep vehicle stopped / in autonomous state
        self.frame_buffer.clear()
        self.current_stable_command = config.CMD_STOP
        return candidate, config.CMD_STOP, True


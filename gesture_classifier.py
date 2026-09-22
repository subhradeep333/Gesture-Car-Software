"""
Hand Gesture Classifier with Landmark Geometry and Temporal Hysteresis Smoothing.
Optimized with squared distance metrics to eliminate math.sqrt CPU overhead.
"""

from collections import deque
import config

class GestureClassifier:
    def __init__(self, stability_threshold=config.DEFAULT_STABILITY_FRAME_THRESHOLD):
        self.stability_threshold = max(1, stability_threshold)
        self.frame_buffer = deque(maxlen=self.stability_threshold)
        self.current_stable_command = config.CMD_STOP
        self.current_candidate_gesture = config.CMD_STOP

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

    def _is_finger_curled(self, landmarks, tip_idx, pip_idx, mcp_idx, wrist_idx=0):
        """
        Determines if a finger is curled towards the palm.
        Returns True if tip squared distance to wrist is smaller than PIP or MCP threshold.
        """
        d2_tip_wrist = self._sq_distance(landmarks[tip_idx], landmarks[wrist_idx])
        d2_pip_wrist = self._sq_distance(landmarks[pip_idx], landmarks[wrist_idx])
        d2_mcp_wrist = self._sq_distance(landmarks[mcp_idx], landmarks[wrist_idx])
        
        # Tip is folded closer to wrist than PIP or MCP (using squared ratio 1.15^2 = 1.3225)
        return d2_tip_wrist < d2_pip_wrist or (d2_tip_wrist / d2_mcp_wrist) < 1.3225

    def classify_frame(self, detected, landmarks, confidence, min_confidence=0.70):
        """
        Classifies single frame landmarks into raw candidate gesture command.
        """
        if not detected or len(landmarks) < 21:
            return "NONE"

        if confidence < min_confidence:
            return "UNKNOWN"

        wrist = landmarks[0]
        thumb_tip = landmarks[4]
        index_tip = landmarks[8]
        index_mcp = landmarks[5]
        pinky_mcp = landmarks[17]

        # Fast finger curl states using squared distance calculations
        index_curled = self._is_finger_curled(landmarks, 8, 7, 5)
        middle_curled = self._is_finger_curled(landmarks, 12, 11, 9)
        ring_curled = self._is_finger_curled(landmarks, 16, 15, 13)
        pinky_curled = self._is_finger_curled(landmarks, 20, 19, 17)

        # Thumb curl state (squared distance ratio: 1.2^2 = 1.44)
        d2_thumb_pinkymcp = self._sq_distance(thumb_tip, pinky_mcp)
        d2_indexmcp_pinkymcp = self._sq_distance(index_mcp, pinky_mcp)
        thumb_curled = d2_thumb_pinkymcp < (1.44 * d2_indexmcp_pinkymcp)

        # 1. FIST (Emergency Stop - E)
        if index_curled and middle_curled and ring_curled and pinky_curled:
            return config.CMD_EMERGENCY_STOP

        # 2. OPEN PALM (Stop - S)
        if not index_curled and not middle_curled and not ring_curled and not pinky_curled:
            return config.CMD_STOP

        # 3. SINGLE INDEX FINGER EXTENDED (FORWARD, BACKWARD, LEFT, RIGHT)
        # Index must NOT be curled, while Middle, Ring, Pinky MUST be curled
        if not index_curled and middle_curled and ring_curled and pinky_curled:
            # Direction vector of index finger relative to MCP base
            dx = index_tip[0] - index_mcp[0]
            dy = index_tip[1] - index_mcp[1]  # Note: y is inverted in image coordinates (0 at top)

            abs_dx = abs(dx)
            abs_dy = abs(dy)

            # Determine dominant direction
            if abs_dy > abs_dx * 0.8:
                if dy < 0:
                    return config.CMD_FORWARD      # ☝️ Pointing UP
                else:
                    return config.CMD_BACKWARD     # 👇 Pointing DOWN
            elif abs_dx > abs_dy * 0.8:
                if dx < 0:
                    return config.CMD_LEFT         # 👈 Pointing LEFT (Image space)
                else:
                    return config.CMD_RIGHT        # 👉 Pointing RIGHT (Image space)

        # Default fallback for unrecognized gesture
        return config.CMD_STOP

    def process(self, detected, landmarks, confidence, min_confidence=0.70):
        """
        Applies temporal hysteresis smoothing to candidate gesture.
        Returns (candidate_gesture, stable_command, is_stable).
        """
        # Fail-safe: No hand or low confidence immediately forces candidate to STOP
        if not detected or confidence < min_confidence:
            self.frame_buffer.clear()
            self.current_candidate_gesture = "NONE" if not detected else "UNKNOWN"
            self.current_stable_command = config.CMD_STOP
            return self.current_candidate_gesture, config.CMD_STOP, True

        candidate = self.classify_frame(detected, landmarks, confidence, min_confidence)
        self.current_candidate_gesture = candidate

        # Immediate Emergency Stop override (Fist)
        if candidate == config.CMD_EMERGENCY_STOP:
            self.frame_buffer.clear()
            self.current_stable_command = config.CMD_EMERGENCY_STOP
            return candidate, config.CMD_EMERGENCY_STOP, True

        # Append candidate to buffer
        self.frame_buffer.append(candidate)

        # Require all entries in buffer to match candidate before committing state change
        if len(self.frame_buffer) == self.stability_threshold:
            if all(g == candidate for g in self.frame_buffer):
                self.current_stable_command = candidate
                return candidate, self.current_stable_command, True

        return candidate, self.current_stable_command, False

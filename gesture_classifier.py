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

    def _is_finger_extended(self, landmarks, tip_idx, pip_idx, mcp_idx, palm_size_sq):
        """
        Determines if a finger is extended using MCP-relative joint distance ratio:
        Scale-invariant & 100% direction-independent across all pointing angles (UP, DOWN, LEFT, RIGHT).
        """
        d2_tip_mcp = self._sq_distance(landmarks[tip_idx], landmarks[mcp_idx])
        d2_pip_mcp = self._sq_distance(landmarks[pip_idx], landmarks[mcp_idx])

        return (d2_tip_mcp > 1.2 * d2_pip_mcp) and (d2_tip_mcp > 0.35 * palm_size_sq)

    def _is_finger_curled(self, landmarks, tip_idx, pip_idx, mcp_idx, palm_size_sq):
        """
        Determines if a finger is curled towards the palm using MCP-relative joint distance ratio.
        """
        d2_tip_mcp = self._sq_distance(landmarks[tip_idx], landmarks[mcp_idx])
        d2_pip_mcp = self._sq_distance(landmarks[pip_idx], landmarks[mcp_idx])

        return (d2_tip_mcp <= 1.2 * d2_pip_mcp) or (d2_tip_mcp <= 0.42 * palm_size_sq)

    def classify_frame(self, detected, landmarks, confidence, min_confidence=0.70):
        """
        Classifies single frame landmarks into raw candidate gesture command with high precision.
        """
        if not detected or len(landmarks) < 21:
            return "NONE"

        if confidence < min_confidence:
            return "UNKNOWN"

        wrist = landmarks[0]
        thumb_tip = landmarks[4]
        index_tip = landmarks[8]
        index_mcp = landmarks[5]
        middle_mcp = landmarks[9]
        pinky_mcp = landmarks[17]

        # Calculate scale-invariant palm size squared (wrist to middle MCP)
        palm_size_sq = self._sq_distance(wrist, middle_mcp)

        # Precise finger state evaluation using scale-invariant MCP-relative joint distance ratios
        index_ext = self._is_finger_extended(landmarks, 8, 6, 5, palm_size_sq)
        middle_ext = self._is_finger_extended(landmarks, 12, 10, 9, palm_size_sq)
        ring_ext = self._is_finger_extended(landmarks, 16, 14, 13, palm_size_sq)
        pinky_ext = self._is_finger_extended(landmarks, 20, 18, 17, palm_size_sq)

        index_curled = self._is_finger_curled(landmarks, 8, 6, 5, palm_size_sq)
        middle_curled = self._is_finger_curled(landmarks, 12, 10, 9, palm_size_sq)
        ring_curled = self._is_finger_curled(landmarks, 16, 14, 13, palm_size_sq)
        pinky_curled = self._is_finger_curled(landmarks, 20, 18, 17, palm_size_sq)

        # 1. FIST (Emergency Stop - E)
        if index_curled and middle_curled and ring_curled and pinky_curled:
            return config.CMD_EMERGENCY_STOP

        # 2. OPEN PALM (Stop - S)
        if index_ext and middle_ext and ring_ext and pinky_ext:
            return config.CMD_STOP

        # 3. SINGLE INDEX FINGER EXTENDED (FORWARD, BACKWARD, LEFT, RIGHT)
        # Index MUST be extended while Middle, Ring, Pinky MUST be curled
        if index_ext and middle_curled and ring_curled and pinky_curled:
            # Vector from index MCP to index tip
            dx = index_tip[0] - index_mcp[0]
            dy = index_tip[1] - index_mcp[1]  # Note: y is inverted in image coordinates (0 at top)

            abs_dx = abs(dx)
            abs_dy = abs(dy)

            # Determine dominant direction smoothly even for tilted/angled hand positions
            if abs_dy >= abs_dx:
                if dy < 0:
                    return config.CMD_FORWARD      # ☝️ Pointing UP
                else:
                    return config.CMD_BACKWARD     # 👇 Pointing DOWN
            else:
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

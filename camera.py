"""
Threaded Camera Stream for low-latency frame acquisition.
Optimized for macOS AVFoundation with 1-frame buffer to eliminate video lag.
"""

import time
import threading
import cv2
import config

class CameraStream:
    def __init__(self, src=config.CAMERA_INDEX, width=config.FRAME_WIDTH, height=config.FRAME_HEIGHT):
        self.src = src
        self.width = width
        self.height = height
        
        self.cap = None
        self.frame = None
        self.ret = False
        self.is_running = False
        self.thread = None
        self.lock = threading.Lock()
        
        # FPS calculation
        self.fps = 0.0
        self._frame_count = 0
        self._fps_start_time = time.time()

    def start(self):
        if self.is_running:
            return True
            
        # Try AVFOUNDATION backend first on macOS, fallback to default
        self.cap = cv2.VideoCapture(self.src, cv2.CAP_AVFOUNDATION)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.src)

        if not self.cap.isOpened():
            self.is_running = False
            return False

        # Set ultra-low latency hardware parameters
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Flush old frames to prevent video lag!
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, config.TARGET_FPS)

        self.ret, self.frame = self.cap.read()
        if not self.ret:
            self.cap.release()
            return False

        self.is_running = True
        self.thread = threading.Thread(target=self._update_loop, daemon=True)
        self.thread.start()
        return True

    def _update_loop(self):
        while self.is_running:
            if self.cap is None or not self.cap.isOpened():
                time.sleep(0.005)
                continue

            ret, frame = self.cap.read()
            if not ret:
                with self.lock:
                    self.ret = False
                time.sleep(0.005)
                continue

            # Fast inline resize only if resolution mismatches
            if frame.shape[1] != self.width or frame.shape[0] != self.height:
                frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_NEAREST)

            with self.lock:
                self.frame = frame
                self.ret = True
                self._frame_count += 1
                
                # FPS update every second
                now = time.time()
                elapsed = now - self._fps_start_time
                if elapsed >= 1.0:
                    self.fps = self._frame_count / elapsed
                    self._frame_count = 0
                    self._fps_start_time = now

            # Yield 1ms for ultra-responsive frame grabbing
            time.sleep(0.001)

    def read(self):
        with self.lock:
            if not self.ret or self.frame is None:
                return False, None, self.fps
            return True, self.frame.copy(), self.fps  # Return thread-safe copy to prevent graphics mutation corruption

    def stop(self):
        self.is_running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.cap and self.cap.isOpened():
            self.cap.release()
        self.cap = None
        self.ret = False
        self.frame = None

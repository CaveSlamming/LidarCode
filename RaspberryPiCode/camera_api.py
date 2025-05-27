# camera_api.py
import cv2
import time
from typing import Tuple, Optional

class CameraAPI:
    def __init__(self, left_index: Optional[int] = None, right_index: Optional[int] = None, upside_down: bool = True):
        """
        Initialize CameraAPI.

        Args:
            left_index: Index for the left camera (physical port).
            right_index: Index for the right camera.
            upside_down: If True, rotate both images 180° and swap their roles.
        """
        self.left_index = left_index
        self.right_index = right_index
        self.upside_down = upside_down
        self.left_cap: Optional[cv2.VideoCapture] = None
        self.right_cap: Optional[cv2.VideoCapture] = None

    def detect_cameras(self, max_tests: int = 4) -> Tuple[int, int]:
        found = []
        for i in range(max_tests):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                found.append(i)
                cap.release()
                if len(found) >= 2:
                    break
        if len(found) < 2:
            raise RuntimeError(f"Could not auto-detect two cameras (found: {found})")
        return found[0], found[1]

    def connect(self) -> None:
        if self.left_index is None or self.right_index is None:
            self.left_index, self.right_index = self.detect_cameras()

        self.left_cap = cv2.VideoCapture(self.left_index)
        self.right_cap = cv2.VideoCapture(self.right_index)
        time.sleep(0.1)  # Allow camera to warm up

        if not self.left_cap.isOpened() or not self.right_cap.isOpened():
            raise RuntimeError(f"Failed to open cameras at {self.left_index}, {self.right_index}")

    def capture(self) -> Tuple:
        if self.left_cap is None or self.right_cap is None:
            raise RuntimeError("Cameras not connected. Call connect() first.")

        ret_l, frame_l = self.left_cap.read()
        ret_r, frame_r = self.right_cap.read()
        if not ret_l or not ret_r:
            raise RuntimeError("Failed to read from one or both cameras.")

        if self.upside_down:
            frame_l = cv2.rotate(frame_l, cv2.ROTATE_180)
            frame_r = cv2.rotate(frame_r, cv2.ROTATE_180)
            frame_l, frame_r = frame_r, frame_l  # Swap due to upside-down mount

        return frame_l, frame_r

    def release(self) -> None:
        if self.left_cap:
            self.left_cap.release()
        if self.right_cap:
            self.right_cap.release()

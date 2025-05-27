import cv2
import time
from typing import Optional, Tuple

class CameraAPI:
    def __init__(self, left_index: int, right_index: int, upside_down: bool = True, exposure: int = -6, gain: int = 10):
        self.left_index = left_index
        self.right_index = right_index
        self.upside_down = upside_down
        self.exposure = exposure
        self.gain = gain
        self.left_cap: Optional[cv2.VideoCapture] = None
        self.right_cap: Optional[cv2.VideoCapture] = None

    def connect(self) -> None:
        self.left_cap = cv2.VideoCapture(self.left_index)
        self.right_cap = cv2.VideoCapture(self.right_index)
        time.sleep(0.1)
        for cap in [self.left_cap, self.right_cap]:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure)
            cap.set(cv2.CAP_PROP_GAIN, self.gain)

    def _is_blurry(self, img, threshold=100) -> bool:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return cv2.Laplacian(gray, cv2.CV_64F).var() < threshold

    def capture(self) -> Tuple:
        frames = []
        for cap in [self.left_cap, self.right_cap]:
            best = None
            best_score = 0
            for _ in range(3):
                ret, frame = cap.read()
                if not ret:
                    continue
                score = cv2.Laplacian(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()
                if score > best_score:
                    best_score = score
                    best = frame
            if best is None:
                raise RuntimeError("Failed to capture from camera")
            frames.append(cv2.rotate(best, cv2.ROTATE_180) if self.upside_down else best)

        return frames[1], frames[0] if self.upside_down else frames[0], frames[1]

    def disconnect(self):
        if self.left_cap:
            self.left_cap.release()
        if self.right_cap:
            self.right_cap.release()

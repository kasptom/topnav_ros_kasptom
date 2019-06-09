#!/usr/bin/python
"""
    QR code (from video) scanner based on examples from
    https://github.com/opencv/opencv
    https://github.com/Zbar/Zbar
"""

from constants.preview import CAM_PREVIEW_HEIGHT, CAM_PREVIEW_WIDTH
from interface_camera import ICamera
from pi_videostream import PiVideoStream


class CameraV2(ICamera):
    def __init__(self):
        self.vs = None

    def open(self):
        self.vs = PiVideoStream(resolution=(CAM_PREVIEW_WIDTH, CAM_PREVIEW_HEIGHT), framerate=32)
        self.vs.start()

    def is_opened(self):
        return self.vs.is_opened()

    def close(self):
        self.vs.stop()

    def get_frame(self):
        """
        Get frame from camera (ignore return status) in gray scale
        :return: image read from camera
        :rtype: PIL.Image
        """
        frame = self.vs.read()
        # use more lightweight format
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # return gray
        return frame

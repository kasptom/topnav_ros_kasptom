#!/usr/bin/python
"""
    QR code (from video) scanner based on examples from
    https://github.com/opencv/opencv
    https://github.com/Zbar/Zbar
"""
import cv2


class Camera:
    def __init__(self, camera_id):
        self.camera_id = camera_id
        self.capture = None

    def open(self):
        self.capture = cv2.VideoCapture(self.camera_id)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    def is_opened(self):
        return self.capture is not None and self.capture.isOpened()

    def close(self):
        self.capture.release()

    def get_frame(self):
        """
        Get frame from camera (ignore return status) in gray scale
        :return: image read from camera
        :rtype: PIL.Image
        """
        frame = self.capture.read()[1]
        # use more lightweight format
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # return gray
        return frame

#!/usr/bin/python
import cv2

from webcam_video_stream import WebcamVideoStream
from interface_camera import ICamera
from constants.preview import CAM_PREVIEW_WIDTH, CAM_PREVIEW_HEIGHT


class CameraV3(ICamera):
    def __init__(self, camera_id):
        self.vs = None
        self.capture = None
        self.camera_id = camera_id

    def open(self):
        self.capture = cv2.VideoCapture(self.camera_id)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_PREVIEW_WIDTH)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_PREVIEW_HEIGHT)
        self.vs = WebcamVideoStream(self.capture)
        self.vs.start()

    def is_opened(self):
        return self.capture is not None and self.capture.isOpened()

    def close(self):
        self.vs.stop()
        self.capture.release()

    def get_frame(self):
        """
        Get frame from camera (ignore return status) in gray scale
        :return: image read from camera
        :rtype: PIL.Image
        """
        frame = self.vs.read()
        return frame

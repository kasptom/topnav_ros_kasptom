#!/usr/bin/python

import rospy
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
# from camera.camera import Camera
# from camera.camera_v2 import CameraV2
from camera.camera_v3 import CameraV3
from constants.parameter_names import ROS_LAUNCH_PARAM_CAMERA_ID, CAMERA_DEFAULT_ID, ROS_LAUNCH_PARAM_CAMERA_WIDTH, \
    ROS_LAUNCH_PARAM_CAMERA_HEIGHT
from constants.preview import CAM_PREVIEW_WIDTH, CAM_PREVIEW_HEIGHT

TOPIC_NAME_CAMERA = "capo/camera1/image_raw"


# http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29


class Capo2CameraPublisher:
    def __init__(self):
        self.publisher = rospy.Publisher(TOPIC_NAME_CAMERA, Image, queue_size=1)
        self.bridge_object = CvBridge()
        self.camera_id = 0
        self.camera_width = CAM_PREVIEW_WIDTH
        self.camera_height = CAM_PREVIEW_HEIGHT

    def start(self):
        self.camera_id = rospy.get_param(ROS_LAUNCH_PARAM_CAMERA_ID, CAMERA_DEFAULT_ID)
        self.camera_width = rospy.get_param(ROS_LAUNCH_PARAM_CAMERA_WIDTH, CAM_PREVIEW_WIDTH)
        self.camera_height = rospy.get_param(ROS_LAUNCH_PARAM_CAMERA_HEIGHT, CAM_PREVIEW_HEIGHT)
        print 'starting camera with id=%d (%dx%d)' % (self.camera_id, self.camera_width, self.camera_height)

        rospy.init_node("capo2_camera_publisher", anonymous=True)
        rate = rospy.Rate(10)  # 10hz
        # camera = Camera(camera_id=0)
        # camera = CameraV2()
        camera = CameraV3(camera_id=self.camera_id, camera_width=self.camera_width, camera_height=self.camera_height)

        camera.open()

        while not rospy.is_shutdown() and camera.is_opened():
            image = camera.get_frame()
            img_message = self.bridge_object.cv2_to_imgmsg(image, 'bgr8')
            self.publisher.publish(img_message)
            rate.sleep()


if __name__ == '__main__':
    try:
        publisher = Capo2CameraPublisher()
        publisher.start()
    except rospy.ROSInterruptException:
        raise

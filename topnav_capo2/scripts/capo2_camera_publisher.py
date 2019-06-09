#!/usr/bin/python

import rospy
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from camera.camera import Camera
# from camera.camera_v2 import CameraV2

TOPIC_NAME_CAMERA = "capo/camera1/image_raw"


# http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29


class Capo2CameraPublisher:
    def __init__(self):
        self.publisher = rospy.Publisher(TOPIC_NAME_CAMERA, Image, queue_size=1)
        self.bridge_object = CvBridge()

    def start(self):
        rospy.init_node("capo2_camera_publisher", anonymous=True)
        rate = rospy.Rate(10)  # 10hz
        camera = Camera(camera_id=0)
        # camera = CameraV2()

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

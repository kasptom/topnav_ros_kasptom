#!/usr/bin/python
import sys
from datetime import datetime
import time

import rospy
from gazebo_msgs.msg import ModelStates
from topnav_msgs.msg import FeedbackMsg
from topnav_msgs.msg import GuidelineMsg

from constants.topic_names import GAZEBO_MODEL_STATES_TOPIC, TOPNAV_FEEDBACK_TOPIC, TOPNAV_GUIDELINES_TOPIC

ROBOT_MODEL_NAME = 'capo'


class PositionTracker:
    def __init__(self, robot_model_name=ROBOT_MODEL_NAME):
        self.csv_position_log = None
        self.prev_log_time = None
        self.current_feedback = 'N/A'
        self.current_guideline = 'N/A'
        self.robot_model_name = robot_model_name

        self.model_states_subscriber = rospy.Subscriber(GAZEBO_MODEL_STATES_TOPIC, ModelStates, queue_size=1,
                                                        callback=self.log_capo_position)
        self.topnav_feedback_subscriber = rospy.Subscriber(TOPNAV_FEEDBACK_TOPIC, FeedbackMsg, queue_size=1,
                                                           callback=self.log_topnav_feedback)

        self.topnav_guideline_subscriber = rospy.Subscriber(TOPNAV_GUIDELINES_TOPIC, GuidelineMsg, queue_size=1,
                                                            callback=self.log_topnav_guideline)

    def __del__(self):
        self.csv_position_log.close()

    def start(self):
        with open("capo_position%s.csv" % self._get_timestamp(), mode='w+') as log_file:
            self.prev_log_time = time.time()
            self.csv_position_log = log_file

            rospy.init_node("capo2_controller", anonymous=True)
            rate = rospy.Rate(1)  # 10hz

            while not rospy.is_shutdown():
                rate.sleep()

    def log_capo_position(self, message_model_states):
        if time.time() - self.prev_log_time <= 1.0:
            return

        self.prev_log_time = time.time()
        robot_model_name_idx = message_model_states.name.index(self.robot_model_name)
        robot_pose = message_model_states.pose[robot_model_name_idx]
        robot_twist = message_model_states.twist[robot_model_name_idx]

        csv_string = self.get_csv_string(robot_pose, robot_twist, time.time())
        print(csv_string)
        self.csv_position_log.write('%s\n' % csv_string)

    def log_topnav_feedback(self, topnav_feedback):
        self.current_feedback = topnav_feedback

    def log_topnav_guideline(self, topnav_guideline):
        self.current_guideline = topnav_guideline

    def _get_timestamp(self):
        now = datetime.now()
        timestamp_str = now.strftime('%Y-%m-%d %H:%M:%s')
        return timestamp_str

    def get_csv_string(self, robot_pose, robot_twist, timestamp):
        pos = robot_pose.position
        orn = robot_pose.orientation
        lin = robot_twist.linear
        ang = robot_twist.angular

        return "T: %.2f, P, %.3f, %.3f,%.3f, O, %.3f, %.3f,%.3f," \
               " L, %.3f, %.3f,%.3f, A, %.3f, %.3f,%.3f, FB: %s, GL: %s" % \
               (timestamp, pos.x, pos.y, pos.z,
                orn.x, orn.y, orn.z,
                lin.x, lin.y, lin.z,
                ang.x, ang.y, ang.z,
                self.current_feedback,
                self.current_guideline)


if __name__ == '__main__':
    try:
        if len(sys.argv) > 1:
            tracker = PositionTracker(robot_model_name=sys.argv[1])
        else:
            tracker = PositionTracker()
        tracker.start()
    except rospy.ROSInterruptException:
        raise

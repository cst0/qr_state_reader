#!/usr/bin/env python2.7

import rospy

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from qr_state_reader.srv import ReadEnvironment, ReadEnvironmentRequest, ReadEnvironmentResponse

BRIDGE = CvBridge()
DETECTOR = cv2.QRCodeDetector()

MOST_RECENT_READING = ''

def image_read(image):
    cv_image = BRIDGE.imgmsg_to_cv2(image)
    text, points, _, DETECTOR.detectAndDecode(cv_image)

    if text is not None and len(text) != 0:
        MOST_RECENT_READING = text
        MOST_RECENT_TIME = rospy.Time.now()

def handle_read_env(req):
    req.empty  # input is empty, ignore
    return ReadEnvironmentResponse(ReadEnvironmentResponse.TREE)

def add_two_ints_server():
    rospy.init_node('read_environment_server')
    s = rospy.Service('read_environment', ReadEnvironment, handle_read_env)
    print("Ready to add two ints.")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()

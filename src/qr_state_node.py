#!/usr/bin/env python2.7
import rospy
import time
import sys

import cv2
from cv_bridge import CvBridge
from rospy.rostime import Duration
from sensor_msgs.msg import Image

from qr_state_reader.srv import ReadEnvironment, ReadEnvironmentRequest, ReadEnvironmentResponse

BRIDGE = CvBridge()
DETECTOR = cv2.QRCodeDetector()

MOST_RECENT_READING = ''
MOST_RECENT_TIME = 0

def image_read(image):
    cv_image = BRIDGE.imgmsg_to_cv2(image)
    res = (DETECTOR.detectAndDecode(cv_image))
    text, points, _ = res
    if text is not None and len(text) != 0:
        rospy.loginfo_throttle_identical(0.5, 'New image: '+str(text))
        global MOST_RECENT_READING
        MOST_RECENT_READING = text
        global MOST_RECENT_TIME
        MOST_RECENT_TIME = time.time()

def handle_read_env(req):
    req.empty  # input is empty, ignore
    if (time.time() - MOST_RECENT_TIME) > 10:
        rospy.loginfo("Requested information from too long ago!")
        return ReadEnvironmentResponse(ReadEnvironmentResponse.NONE)

    if MOST_RECENT_READING == 'CRAFTINGTABLE': return ReadEnvironmentResponse(ReadEnvironmentResponse.CRAFTING_TABLE)
    if MOST_RECENT_READING == 'TREE1': return ReadEnvironmentResponse(ReadEnvironmentResponse.TREE1)
    if MOST_RECENT_READING == 'TREE2': return ReadEnvironmentResponse(ReadEnvironmentResponse.TREE2)
    if MOST_RECENT_READING == 'TREE3': return ReadEnvironmentResponse(ReadEnvironmentResponse.TREE3)
    if MOST_RECENT_READING == 'TREE4': return ReadEnvironmentResponse(ReadEnvironmentResponse.TREE4)
    if MOST_RECENT_READING == 'ROCK1': return ReadEnvironmentResponse(ReadEnvironmentResponse.ROCK1)
    if MOST_RECENT_READING == 'ROCK2': return ReadEnvironmentResponse(ReadEnvironmentResponse.ROCK2)

    rospy.loginfo("Reading didn't match any known inputs!")
    return ReadEnvironmentResponse(ReadEnvironmentResponse.NONE)

def read_env_server():
    rospy.init_node('read_environment_server')
    global MOST_RECENT_TIME
    MOST_RECENT_TIME = time.time()
    rospy.Subscriber('/camera/color/image_raw', Image, image_read)
    s = rospy.Service('read_environment', ReadEnvironment, handle_read_env)
    print("Ready to read your env.")
    rospy.spin()
    s.shutdown()

if __name__ == "__main__":
    read_env_server()

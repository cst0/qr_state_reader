#!/usr/bin/env python2.7
import rospy
import time
import sys

sys.path.insert(1, '/home/turtlebot/.local/lib/python2.7/site-packages/cv2/')
#__requires__="opencv-python==4.2.0.32"
#import pkg_resources
#pkg_resources.require("opencv-python==4.2.0.32")
import cv2

print('opencv version '+str(cv2.__version__))
from cv_bridge import CvBridge
from rospy.rostime import Duration
from sensor_msgs.msg import Image

from qr_state_reader.srv import ReadEnvironment, ReadEnvironmentRequest, ReadEnvironmentResponse

BRIDGE = CvBridge()
DETECTOR = cv2.QRCodeDetector()

IMAGE = None
MOST_RECENT_TIME = 0

def image_read(image):
    global IMAGE
    IMAGE = image

def handle_read_env(req):
    req.empty  # input is empty, ignore
    try:
        return read_env()
    except:
        rospy.logerr("failed to read environment. We'll try again...")
        return read_env()

def read_env():
    global IMAGE
    if IMAGE is None:
        return ReadEnvironmentResponse(ReadEnvironmentResponse.NONE)

    most_recent_reading = ''
    start_reads = time.time()
    sample_timing = 2
    while time.time() - start_reads < sample_timing:
        cv_image = BRIDGE.imgmsg_to_cv2(IMAGE)
        res = (DETECTOR.detectAndDecode(cv_image))
        text, points, _ = res
        if text is not None and len(text) != 0:
            rospy.loginfo("qr text reads: "+str(text))
            most_recent_reading = text
            break
#    if (time.time() - MOST_RECENT_TIME) > 10:
#        rospy.loginfo("Requested information from too long ago!")
#        return ReadEnvironmentResponse(ReadEnvironmentResponse.NONE)

    if most_recent_reading == 'CRAFTINGTABLE': return ReadEnvironmentResponse(ReadEnvironmentResponse.CRAFTING_TABLE)
    if most_recent_reading == 'TREE1': return ReadEnvironmentResponse(ReadEnvironmentResponse.TREE1)
    if most_recent_reading == 'TREE2': return ReadEnvironmentResponse(ReadEnvironmentResponse.TREE2)
    if most_recent_reading == 'TREE3': return ReadEnvironmentResponse(ReadEnvironmentResponse.TREE3)
    if most_recent_reading == 'TREE4': return ReadEnvironmentResponse(ReadEnvironmentResponse.TREE4)
    if most_recent_reading == 'ROCK1': return ReadEnvironmentResponse(ReadEnvironmentResponse.ROCK1)
    if most_recent_reading == 'ROCK2': return ReadEnvironmentResponse(ReadEnvironmentResponse.ROCK2)

    rospy.loginfo("Reading didn't match any known inputs!")
    return ReadEnvironmentResponse(ReadEnvironmentResponse.NONE)

def read_env_server():
    rospy.init_node('read_environment_server')
    global MOST_RECENT_TIME
    MOST_RECENT_TIME = time.time()
    rospy.Subscriber('/camera/color/image_raw', Image, image_read)
    s = rospy.Service('/qr_state_reader/read_environment', ReadEnvironment, handle_read_env)
    print("Ready to read your env.")
    rospy.spin()
    s.shutdown()

if __name__ == "__main__":
    read_env_server()

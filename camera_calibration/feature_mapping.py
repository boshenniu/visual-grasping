import numpy as np
import math
import sys
import os
import time

sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
import cv2
from cv_bridge import CvBridge, CvBridgeError
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy


import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image



class feature_mapping():
    def __init__(self):
        self.camera_info_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_image_callback)
        self.pub = rospy.Publisher('changed_image', Image)
        self.bridge = CvBridge()

    def color_image_callback(self, image):
        print(len(image.data))
        print(image.height)
        print(image.width)
        print(image.step)
        pub_image = image
        self.pub.publish(pub_image)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        cv2.imshow("image", cv_image)
        cv2.waitKey(3)

        self.image = image



if __name__ == "__main__":
    rospy.init_node("feature_mapping_test", anonymous=True)
    f = feature_mapping()
    
    rospy.spin()

    
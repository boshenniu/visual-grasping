import numpy as np
import math
import sys
import os
import time

sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy

from cv_bridge import CvBridge, CvBridgeError
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image



class feature_mapping():
    def __init__(self):
        self.camera_info_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_image_callback)
        self.image_pub = rospy.Publisher('changed_image', Image)
        self.bridge = CvBridge()

    def color_image_callback(self, image):
        print(len(image.data))
        print(image.height)
        print(image.width)
        print(image.step)
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        img = self.bridge.imgmsg_to_cv2(image, "bgr8")
        gray= cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        sift = cv2.xfeatures2d.SIFT_create()
        kp = sift.detect(gray,None)

        out = np.array([])
        out = cv2.drawKeypoints(img, kp, outImage=out, color=(0, 0, 255), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        try:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(out, "bgr8"))
        except CvBridgeError as e:
          print(e)

        self.image = img



if __name__ == "__main__":
    rospy.init_node("feature_mapping_test", anonymous=True)
    f = feature_mapping()
    
    rospy.spin()

    
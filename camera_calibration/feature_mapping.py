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
        self.usb_imgs = []

        usb_img_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),"usb_img","1.jpg")
        self.usb_imgs.append(cv2.imread(usb_img_path))
        # print(usb_img_path)
        # print(self.usb_imgs[0].shape)
        # exit()

    def color_image_callback(self, image):
        print(len(image.data))
        print(image.height)
        print(image.width)
        print(image.step)
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        sift = cv2.cornerHarris()

        img1 = self.usb_imgs[0]
        gray= cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)
        kp1, des1 = sift.detectAndCompute(gray,None)

        img2 = self.bridge.imgmsg_to_cv2(image, "bgr8")
        gray= cv2.cvtColor(img2,cv2.COLOR_BGR2GRAY)
        kp2, des2 = sift.detectAndCompute(gray,None)

        MIN_MATCH_COUNT = 10
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)

        flann = cv2.FlannBasedMatcher(index_params, search_params)

        matches = flann.knnMatch(des1,des2,k=2)

        # store all the good matches as per Lowe's ratio test.
        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)

        if len(good)>MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            matchesMask = mask.ravel().tolist()

            h,w,_ = img1.shape
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts,M)

            img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)

        else:
            print("Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT))
            matchesMask = None


        draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                   singlePointColor = None,
                   matchesMask = matchesMask, # draw only inliers
                   flags = 2)

        img3 = cv2.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)

        
        out = np.array([])
        # out = cv2.drawKeypoints(img, kp, outImage=out, color=(0, 0, 255), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        out = img3
        try:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(out, "bgr8"))
        except CvBridgeError as e:
          print(e)

        self.image = img2




if __name__ == "__main__":
    rospy.init_node("feature_mapping_test", anonymous=True)
    f = feature_mapping()
    
    rospy.spin()

    
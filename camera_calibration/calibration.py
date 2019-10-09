import numpy as np
import math
import sys
import os
import time

sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import CameraInfo

from kortex_api.autogen.client_stubs.ActuatorConfigClientRpc import ActuatorConfigClient
from kortex_api.autogen.client_stubs.ActuatorCyclicClientRpc import ActuatorCyclicClient
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.DeviceConfigClientRpc import DeviceConfigClient
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
from kortex_api.autogen.messages import Session_pb2, ActuatorConfig_pb2, Base_pb2, BaseCyclic_pb2, Common_pb2
from kortex_api.RouterClient import RouterClientSendOptions

import argparse
import utilities

def cartesian_action_movement(base, target_pos):
    
    print("Cartesian action executing ...")
    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""

    cartesian_pose = action.reach_pose.target_pose

    cartesian_pose.x = target_pos[0]     # (meters)
    cartesian_pose.y = target_pos[1]     # (meters)
    cartesian_pose.z = target_pos[2]     # (meters)
    cartesian_pose.theta_x = 180 # (degrees)
    cartesian_pose.theta_y = 0 # (degrees)
    cartesian_pose.theta_z = 90 # (degrees)

    base.ExecuteAction(action)

    # print("Waiting 5 seconds for movement to finish ...")
    time.sleep(5)

    # print("Cartesian movement completed")


class CameraCalibration:
    def __init__(self, router, router_real_time):
        self.camera_mtr = None
        self.world3D_list = []
        self.camera3D_list = []
        self.camera3D = None
        self.camera3D_count = -100
        self.world3D_count = 0
        self.joint_angle_list = None
        self.pos_list = None
        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)
        self.ar_tag_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.ar_pose_callback)
        
        device_manager = DeviceManagerClient(router)
        self.actuator_config = ActuatorConfigClient(router)
        self.base = BaseClient(router)
        self.base_cyclic = BaseCyclicClient(router_real_time)
        self.base_feedback = BaseCyclic_pb2.Feedback()

    def camera_info_callback(self, camera_info):
        self.camera_mtr = np.array([camera_info.K]).reshape(3,3)
    def ar_pose_callback(self, pose):
        if len(pose.markers) != 0:
            position = pose.markers[0].pose.pose.position
            self.camera3D = [position.x, position.y, position.z]
            self.camera3D_count = time.time()
    def move_arm(self):
        # Import the utilities helper module
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
        import utilities
        # Parse arguments
        args = utilities.parseConnectionArguments()
        # Create connection to the device and get the router
        with utilities.DeviceConnection.createTcpConnection(args) as router:
            # Create required services
            base = BaseClient(router)

            cartesian_action_movement(base, [0.3, 0, 0.2])
            print("moving to home position")

            input("press Enter to continue")

            # for count, joint_angle in enumerate(self.joint_angle_list):
            for count, pos in enumerate(self.pos_list):

                if count > 5:
                    break
                # actuate
                # print("-----------------")
                # print(joint_angle)
                # example_angular_action_movement(base, joint_angle)
                cartesian_action_movement(base, pos)

                self.base_feedback = self.SendCallWithRetry(self.base_cyclic.RefreshFeedback, 3)

                joint_angles = [self.base_feedback.actuators[i].position for i in range(7)]

                print("============")
                print("data sampe:")
                print(count,'/',len(self.pos_list))
                print("joint_angles")
                print(joint_angles)
                print("ARTag")
                print(utilities.calculateARTagPosition(joint_angles))
                print("camera")
                print(self.camera3D)
                # input("press Enter")
                # rospy.sleep(5)

                self.world3D_count = time.time()
                print("delay")
                print(self.camera3D_count - self.world3D_count)
                delay_tol = 0.1
                if self.camera3D is None or abs(self.camera3D_count - self.world3D_count) > delay_tol:
                    continue
                self.world3D_list.append(utilities.calculateARTagPosition(joint_angles))
                self.camera3D_list.append(self.camera3D)

        print("============")
        print("valid data count:")
        print(len(self.world3D_list))

        
    def calculate_mtr(self):
        camera2D = self.camera_mtr.dot(np.array(self.camera3D_list).T)
        camera2D /= camera2D[-1:,:]
        camera2D = camera2D[:-1,:].T

        # print("-------------------")
        # print(np.array(self.world3D_list).shape)
        # print(camera2D.shape)

        ret, rvecs, tvecs = cv2.solvePnP(np.array(self.world3D_list)/1000., camera2D, self.camera_mtr, None)

        dst, jacobian = cv2.Rodrigues(rvecs)
        print("Rotation:")
        print(dst)
        print("Position:")
        print(tvecs)

    @staticmethod
    def SendCallWithRetry(call, retry,  *args):
        i = 0
        arg_out = []
        while i < retry:
            try:
                arg_out = call(*args)
                break
            except:
                i = i + 1
                continue
        if i == retry:
            print("Failed to communicate")
        return arg_out

def main():

    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    
    rospy.init_node("camera_calibration_kinova_test", anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument("--cyclic_time", type=float, help="delay, in seconds, between cylic control call", default=0.001)
    parser.add_argument("--duration", type=int, help="example duration, in seconds (0 means infinite)", default=30)
    parser.add_argument("--print_stats", default=True, help="print stats in command line or not (0 to disable)", type=lambda x: (str(x).lower() not in ['false', '0', 'no']))
    args = utilities.parseConnectionArguments(parser)

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        with utilities.DeviceConnection.createUdpConnection(args) as router_real_time:

            myCameraCalibration = CameraCalibration(router, router_real_time)

            # myCameraCalibration.joint_angle_list = [
            #         [30.534, 13.041, 166.953, 230.935, 1.967, 330.464, 100.346],
            #         [23.646, 37.306, 169.927, 257.298, 8.416, 328.065, 92.203],
            #         [355.536, 23.103, 171.337, 247.35, 10.246, 323.596, 63.933],
            #         [348.496, 29.868, 172.386, 259.591, 10.864, 317.762, 58.541]]

            center = np.array([0.3, 0, 0.2])
            myCameraCalibration.pos_list =  []
            for dx in [-0.1, 0, 0.1]:
                for dy in [-0.1, 0, 0.1]:
                    for dz in [-0.1, 0, 0.1]:
                        myCameraCalibration.pos_list.append(center + [dx, dy, dz])

            myCameraCalibration.move_arm()
            myCameraCalibration.calculate_mtr()
            rospy.spin()

if __name__ == "__main__":
    main()

    
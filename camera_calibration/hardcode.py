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
from GripperController import GripperController
import argparse
import utilities

def cartesian_action_movement(base, target_pos):
    
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

    print("Waiting 5 seconds for movement to finish ...")
    time.sleep(5)

    # print("Cartesian movement completed")


class KinovaController:
    def __init__(self, router, router_real_time):
        self.camera_mtr = None
        self.world3D_list = []
        self.camera3D_list = []
        self.camera3D = None
        self.camera3D_count = -100
        self.world3D_count = 0
        self.joint_angle_list = None
  
        device_manager = DeviceManagerClient(router)
        self.actuator_config = ActuatorConfigClient(router)
        self.base = BaseClient(router)
        self.base_cyclic = BaseCyclicClient(router_real_time)
        self.base_feedback = BaseCyclic_pb2.Feedback()

        self.gripper = GripperController(router, router_real_time)

       
    def set_single_level_servoing_mode(self):
        # Save servoing mode before changing it
        self.previous_servoing_mode = self.base.GetServoingMode()
        # Set base in single level servoing mode
        servoing_mode_info = Base_pb2.ServoingModeInformation()
        # print("Setting single level servoi
        print("Current Mode:")
        print(servoing_mode_info)
        servoing_mode_info.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(servoing_mode_info)
        print("Waiting 5 second for changing to single level servoing mode...")
        time.sleep(5)
        print("After setting single level")
        print(servoing_mode_info)


    def move_arm(self, pos_list):
        self.set_single_level_servoing_mode()
        # for count, joint_angle in enumerate(self.joint_angle_list):
        for count, pos in enumerate(pos_list):
            print("Moving to ",pos)
            cartesian_action_movement(self.base, pos)
            self.base_feedback = self.SendCallWithRetry(self.base_cyclic.RefreshFeedback, 3)
            joint_angles = [self.base_feedback.actuators[i].position for i in range(7)]


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
    
    rospy.init_node("kinova_controller", anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument("--cyclic_time", type=float, help="delay, in seconds, between cylic control call", default=0.001)
    parser.add_argument("--duration", type=int, help="example duration, in seconds (0 means infinite)", default=30)
    parser.add_argument("--print_stats", default=True, help="print stats in command line or not (0 to disable)", type=lambda x: (str(x).lower() not in ['false', '0', 'no']))
    args = utilities.parseConnectionArguments(parser)

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        with utilities.DeviceConnection.createUdpConnection(args) as router_real_time:

            myKinovaController = KinovaController(router, router_real_time)

            hover_shift = np.array([0, 0, 0.1])
            usb_pos = np.array([0.5, 0, 0.1])
            pcb_pos = np.array([0.3, 0, 0.1])

            home = [0.3, 0, 0.2]
            #We are using single level servoing for arm control and low level servoing for gripper control
            #Switching between servoing mode causes robot viberating. We should change the servoing mode of gripper to single level servoing either.
            #TODO: Fix the servoing mode inconsistency before running this code. Current code may cause robot self-collision.
            myKinovaController.move_arm([home])
            myKinovaController.gripper.open_gripper()
            myKinovaController.move_arm([usb_pos+hover_shift, usb_pos])
            myKinovaController.gripper.close_gripper()
            myKinovaController.move_arm([usb_pos+hover_shift, pcb_pos+hover_shift, pcb_pos+(hover_shift/2)])
            myKinovaController.gripper.open_gripper()
            myKinovaController.move_arm([home])
            print("Complete")
            exit()
            
            rospy.spin()


if __name__ == "__main__":
    main()

    
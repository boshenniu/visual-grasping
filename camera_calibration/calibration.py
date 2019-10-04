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

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2, Common_pb2

deg_to_rad = np.pi/180

def rotXR(r):
    return np.array([[1, 0, 0, 0], [0, math.cos(r), -math.sin(r), 0], [0, math.sin(r), math.cos(r), 0], [0, 0, 0, 1]])

def rotYP(p):
    return np.array([[math.cos(p), 0, math.sin(p), 0], [0, 1, 0, 0], [-math.sin(p), 0, math.cos(p), 0], [0, 0, 0, 1]])

def rotZY(y):
    return np.array([[math.cos(y), -math.sin(y), 0, 0], [math.sin(y), math.cos(y), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

def transXYZ(x,y,z):
    return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])


def calculateARTagPosition(joint_angle):
    tag_pos_in_frame7 = np.array([0., -73.61, -19.5, 1.])
    
    TM_0_1 = rotXR(180*deg_to_rad).dot(transXYZ(0,0,-156.4))
    TM_1_2 = rotXR(90*deg_to_rad).dot(transXYZ(0,-128.4,-5.4))
    TM_2_3 = rotXR(-90*deg_to_rad).dot(transXYZ(0,6.4,-210.4))
    TM_3_4 = rotXR(90*deg_to_rad).dot(transXYZ(0,-210.4,-6.4))
    TM_4_5 = rotXR(-90*deg_to_rad).dot(transXYZ(0,6.4,-208.4))
    TM_5_6 = rotXR(90*deg_to_rad).dot(transXYZ(0,-105.9,0))
    TM_6_7 = rotXR(-90*deg_to_rad).dot(transXYZ(0,0,-105.9))
    TMs = [TM_0_1, TM_1_2, TM_2_3, TM_3_4, TM_4_5, TM_5_6, TM_6_7]

    TM = np.eye(4)
    for i in range(len(TMs)):
        TM = TM.dot(TMs[i]).dot(rotZY(joint_angle[i]*deg_to_rad))
    

    return TM.dot(tag_pos_in_frame7)[0:3]

# calculateARTagPosition([0,0,0,0,0,0,0])

def example_move_to_home_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        sys.exit(0)

    base.ExecuteActionFromReference(action_handle)
    time.sleep(10) # Leave time to action to complete

def example_angular_action_movement(base, target_joint_angle):
    
    

    print("Starting angular action movement ...")
    action = Base_pb2.Action()
    action.name = "Example angular action movement"
    action.application_data = ""

    actuator_count = base.GetActuatorCount()

    for joint_id in range(actuator_count.count):
        joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
        joint_angle.joint_identifier = joint_id
        joint_angle.value = target_joint_angle[joint_id]

    print("Executing action")
    # print(target_joint_angle)
    base.ExecuteAction(action)

    print("Waiting 5 seconds for movement to finish ...")
    time.sleep(5)

    print("Angular movement completed")

class CameraCalibration:
    def __init__(self):
        self.camera_mtr = None
        self.world3D_list = []
        self.camera3D_list = []
        self.camera3D = None
        self.joint_angle_list = None
        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)
        self.ar_tag_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.ar_pose_callback)
    def camera_info_callback(self, camera_info):
        self.camera_mtr = np.array([camera_info.K]).reshape(3,3)
    def ar_pose_callback(self, pose):
        if len(pose.markers) != 0:
            position = pose.markers[0].pose.pose.position
            self.camera3D = [position.x, position.y, position.z]
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

            example_move_to_home_position(base)

            # example_angular_action_movement(base, [0., 0., 0., 0., 0., 0., 0.])
            # input("press Enter")

        
            for count, joint_angle in enumerate(self.joint_angle_list):
                # actuate
                # print("-----------------")
                # print(joint_angle)
                example_angular_action_movement(base, joint_angle)

                print(calculateARTagPosition(joint_angle))
                input("press Enter")
                # rospy.sleep(5)
                self.world3D_list.append(calculateARTagPosition(joint_angle))
                self.camera3D_list.append(self.camera3D)
        
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

def main():
    rospy.init_node("camera_calibration_kinova_test", anonymous=True)
    myCameraCalibration = CameraCalibration()
    myCameraCalibration.joint_angle_list = [
			[30.534, 13.041, 166.953, 230.935, 1.967, 330.464, 100.346],
			[23.646, 37.306, 169.927, 257.298, 8.416, 328.065, 92.203],
			[355.536, 23.103, 171.337, 247.35, 10.246, 323.596, 63.933],
			[348.496, 29.868, 172.386, 259.591, 10.864, 317.762, 58.541]]
    myCameraCalibration.move_arm()
    myCameraCalibration.calculate_mtr()
    rospy.spin()

if __name__ == "__main__":
    main()

    
import numpy as np
import math
import sys

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
    tag_pos_in_frame7 = np.array([0, -73.61, -19.5, 1])
    
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
        TM = TM.dot(TMs[i]).dot(rotXR(joint_angle[i]*deg_to_rad))
    
    return TM.dot(tag_pos_in_frame7)[0:3]

# calculateARTagPosition([0,0,0,0,0,0,0])

def example_angular_action_movement(base, joint_angle):
    
    print("Starting angular action movement ...")
    action = Base_pb2.Action()
    action.name = "Example angular action movement"
    action.application_data = ""

    actuator_count = base.GetActuatorCount()

    for joint_id in range(actuator_count.count):
        joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
        joint_angle.joint_identifier = joint_id
        joint_angle.value = joint_angle[joint_id]

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting 20 seconds for movement to finish ...")
    time.sleep(20)

    print("Angular movement completed")

class CameraCalibration:
    def __init__(self):
        self.camera_mtr
        self.world3D_list = []
        self.camera3D_list = []
        self.camera3D
        self.joint_angle_list
        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, camera_info_callback)
        self.ar_tag_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, ar_pose_callback)
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
        
            for count, joint_angle in enumerate(self.joint_angle_list):
                # actuate
                example_angular_action_movement(base, joint_angle)
                # rospy.sleep(5)
                self.world3D_list.append(calculateARTagPosition(joint_angle))
                self.camera3D_list.append(self.camera3D)
        
    def calculate_mtr(self):
        camera2D = camera_mtr.dot(np.array(self.camera3D_list).T)
        camera2D /= camera2D[-1:,:]
        camera2D = camera2D[:-1,:].T
        ret, rvecs, tvecs = cv2.solvePnP(self.world3D_list, camera2D, self.camera_mtr, None)
        dst, jacobian = cv2.Rodrigues(rvecs)
        print("Rotation:")
        print(dst)
        print("Position:")
        print(tvecs)

def main():
    rospy.init_node("camera_calibration_kinova_test", anonymous=True)
    myCameraCalibration = CameraCalibration()
    myCameraCalibration.joint_angle_list = [
                    [],
                    [],
                    [],
                    [],
                    [],
                    [],
                    [],
                    [],
                    [],
                    [],
                    [],
                    [],
                    [],
                    [],
                    []]
    myCameraCalibration.move_arm()
    myCameraCalibration.calculate_mtr()
    rospy.spin()

if __name__ == "__main__":
    main()

    
import argparse
import numpy as np
import math

from kortex_api.TCPTransport import TCPTransport
from kortex_api.UDPTransport import UDPTransport
from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.messages import Session_pb2

def parseConnectionArguments(parser = argparse.ArgumentParser()):
    parser.add_argument("--ip", type=str, help="IP address of destination", default="192.168.1.10")
    parser.add_argument("-u", "--username", type=str, help="username to login", default="admin")
    parser.add_argument("-p", "--password", type=str, help="password to login", default="admin")
    return parser.parse_args()


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

class DeviceConnection:
    
    TCP_PORT = 10000
    UDP_PORT = 10001

    @staticmethod
    def createTcpConnection(args): 
        """
        returns RouterClient required to create services and send requests to device or sub-devices,
        """

        return DeviceConnection(args.ip, port=DeviceConnection.TCP_PORT, credentials=(args.username, args.password))

    @staticmethod
    def createUdpConnection(args): 
        """        
        returns RouterClient that allows to create services and send requests to a device or its sub-devices @ 1khz.
        """

        return DeviceConnection(args.ip, port=DeviceConnection.UDP_PORT, credentials=(args.username, args.password))

    def __init__(self, ipAddress, port=TCP_PORT, credentials = ("","")):

        self.ipAddress = ipAddress
        self.port = port
        self.credentials = credentials

        self.sessionManager = None

        # Setup API
        self.transport = TCPTransport() if port == DeviceConnection.TCP_PORT else UDPTransport()
        self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)

    # Called when entering 'with' statement
    def __enter__(self):
        
        self.transport.connect(self.ipAddress, self.port)

        if (self.credentials[0] != ""):
            session_info = Session_pb2.CreateSessionInfo()
            session_info.username = self.credentials[0]
            session_info.password = self.credentials[1]
            session_info.session_inactivity_timeout = 10000   # (milliseconds)
            session_info.connection_inactivity_timeout = 2000 # (milliseconds)

            self.sessionManager = SessionManager(self.router)
            print("Logging as", self.credentials[0], "on device", self.ipAddress)
            self.sessionManager.CreateSession(session_info)

        return self.router

    # Called when exiting 'with' statement
    def __exit__(self, exc_type, exc_value, traceback):
    
        if self.sessionManager != None:

            router_options = RouterClientSendOptions()
            router_options.timeout_ms = 1000 
            
            self.sessionManager.CloseSession(router_options)

        self.transport.disconnect()

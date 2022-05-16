import socket
import time
import numpy as np
import sys
import math
import random
import matplotlib.pyplot as plt
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import cv2
from Main import *

def open():
    command = 'CMD_SERVO#0#170\n'
    s.send(command.encode('utf-8'))
    
def close():
    command = 'CMD_SERVO#0#85\n'
    s.send(command.encode('utf-8'))
    
def duck():
    return

def stop():
    command = 'CMD_MOTOR#0#0#0#0\n'
    s.send(command.encode('utf-8'))



positions = {}
rotations = {}

home = [0, 4.5]

IP_ADDRESS = '192.168.0.204'
clientAddress = socket.gethostbyname(socket.gethostname())
optitrackServerAddress = "192.168.0.4"
robot_id = 4

robot = mywindow()
robot.on_btn_Connect()



duck = 0 #boolean for if duck is detected

north = [1.7,3.62]
south = [0,-3.1]
east = [-4.2,0]
north_east = [5.5,3.6]
north_west = [-4.22,3.65]
south_east = [5.4,-3.07]
south_west = [-4.1,-2.99]
origin = [0.,0.]

def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    positions[robot_id] = position
    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)
    rotations[robot_id] = rotz

streaming_client = NatNetClient()
streaming_client.set_client_address(clientAddress)
streaming_client.set_server_address(optitrackServerAddress)
streaming_client.set_use_multicast(True)
streaming_client.rigid_body_listener = receive_rigid_body_frame
is_running = streaming_client.run()

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')

safe = [5.43, 0.]
b = [0.,0.]
g = [0.,0.]
kω = 700
kd = 1700
kb = 0

def move_to(g):
    print('Heading to %.1f , %.1f\n',g[0],g[1])
    while is_running:
        if robot_id in positions:
            p = [positions[robot_id][0], positions[robot_id][1]]
            pbmagcubed = (np.sqrt( (p[0]-b[0])**2 + (p[1]-b[1])**2) )**3
            q = ((g[0] - p[0] ) + kb * ( (p[0] - b[0] ) /pbmagcubed)
                , (g[1] - p[1] ) + kb * ( (p[1] - b[1] ) /pbmagcubed) )
            
            α = np.arctan2(q[1], q[0])
            distance = np.sqrt((g[0]-p[0])**2+(g[1]-p[1])**2)
            theta_r = np.radians(rotations[robot_id])
            
            ω = np.degrees(kω*np.arctan2(np.sin(α-theta_r),np.cos(α-theta_r)))
            v = kd*(distance)    
            
            u = np.array([-v + ω,-v - ω])
            u[u > 1500] = 1500
            u[u < -1500] = -1500
                    
            
            command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
            s.send(command.encode('utf-8'))
            
            if( distance < .8):
                return

            # Wait for .1 second.
            time.sleep(.1)
            
def rescue():
    return


#Explore Phase

move_to([0.,0.])
print('Cleared')
stop()






#First Connect to robot CHECK

#Define Constant obstacles

#Explore and look for ducks

#When duck is found initiate a method for ducks
#    Open Gripper
#    Orient to duck
#    Drive forward
#    Close gripper
#    Return to safe zone
#    repeat
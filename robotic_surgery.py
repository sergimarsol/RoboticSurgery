# Biomedical Engneering - Robotics
# Robotic Surgery Project
# Sergi Marsol

import serial
import time
import math
import keyboard
import tkinter as tk
from enum import Enum
import threading

# RoboDK API: import the robolink library (bridge with RoboDK)
from robolink import *
# Robot toolbox: import the robodk library (robotics toolbox)
from robodk import *

# ------------------------------------------------------------------------------
# Connection
# ------------------------------------------------------------------------------

# Establish the connection on a specific port
arduino = serial.Serial('COM3', 115200, timeout=1)

# Lets bring some time to the system to stablish the connetction
time.sleep(2)

# Establish a link with the simulator
RDK = robolink.Robolink()

# ------------------------------------------------------------------------------
# Simulator setup
# ------------------------------------------------------------------------------

# Retrieve all items (object in the robodk tree)
# Define the "robot" variable with our robot (UR5e)
robot = RDK.Item('UR5e')
base = RDK.Item('UR5e Base')

# Define the "tcp" variable with the TCP of Endowrist needle
tcp_tool = RDK.Item('Endowrist')
gripper = RDK.Item('Gripper')
needle = RDK.Item('Needle')
Init_target = RDK.Item('Init')

gripper_init = TxyzRxyz_2_Pose([0, -10, 609, pi, 0, 0])
gripper.setPose(gripper_init)
needle_init = TxyzRxyz_2_Pose([0, 0, 0, 0, 0, 0])
needle.setParent(gripper) # First time important setParent for absolute POSE
needle.setPose(needle_init)

robot.MoveL(Init_target)
print("Pose obtained")
robot.setSpeed(5)

# Create the main Tk window
root = tk.Tk()
root.title("Keyboard checking")
# Create a label to display the text
text_label = tk.Label(root, text="", wraplength=300)  # Adjust wraplength as needed
text_label.pack(padx=20, pady=20)

RUN_ON_ROBOT = False

if RUN_ON_ROBOT:
    robot.setConnectionParams('192.168.1.5',30000,'/', 'anonymous','')# Update connection parameters if required:
    #time.sleep(5)
    success = robot.ConnectSafe('192.168.1.5') # Try to connect once
    #time.sleep(5)
    status, status_msg = robot.ConnectedState()
    if status != ROBOTCOM_READY: # Stop if the connection did not succeed
        #print(status_msg)
        raise Exception("Failed to connect: " + status_msg)

    # This will set to run the API programs on the robot and the simulator (online programming)
    RDK.setRunMode(RUNMODE_RUN_ROBOT)

else:

    RDK.setRunMode(RUNMODE_SIMULATE)  # This will run the API program on the simulator (offline programming)  

# Move robot to Init POSE
print("POSE Init: ")
robot.MoveL(Init_target)
print("Pose obtained")
robot.setSpeed(10)

# ------------------------------------------------------------------------------
# Reference frame is fixed to TCP
#
# Data comunication
# ------------------------------------------------------------------------------
#

class Command(Enum):
    GET_RPW = b'\x01'


def check_keyboard():
    global R  # Use the global variable P
    global Rr
    while True:
        # Requesting data to Ardino
        arduino.write(Command.GET_RPW.value)

        # Storing received data
        roll_str = arduino.readline().strip()
        pitch_str = arduino.readline().strip()
        yaw_str = arduino.readline().strip()
        s1_str = arduino.readline().strip()
        s2_str = arduino.readline().strip()
        torque_yaw_str=arduino.readline().strip()
        torque_pitch_str=arduino.readline().strip()
        torque_roll1_str=arduino.readline().strip()
        torque_roll2_str=arduino.readline().strip()

        # Convert variable values from string to float
        roll = float(roll_str)
        pitch = float(pitch_str)
        yaw = float(yaw_str)
        s1 = bool(int(s1_str))
        s2 = bool(int(s2_str))
        torque_yaw=float(torque_yaw_str)
        torque_pitch=float(torque_pitch_str)
        torque_roll1=float(torque_roll1_str)
        torque_roll2=float(torque_roll2_str)

        print("R= "+str(round(roll,0))+"\t   P= "+str(round(pitch))+"\t   W= "+str(round(yaw))+"\tTw= "+str(round(1000*torque_yaw))+"\t   Tp= "+str(round(1000*torque_pitch))+"\t   Tr1= "+str(round(1000*torque_roll1))+"\t   Tr2= "+str(round(1000*torque_roll2)))
        
        #Read the position of the robot
        robot_pose=robot.Pose()
        Xr, Yr, Zr, rr, pr, yr = Pose_2_TxyzRxyz(robot_pose)
        
        #Read the position of the gripper
        gripper_pose=gripper.Pose()
        Xg, Yg, Zg, rg, pg, yg = Pose_2_TxyzRxyz(gripper_pose)

        # Convert from degrees to radians R,P,Y angles
        R = math.radians(roll)
        P = math.radians(pitch)
        W = math.radians(yaw)  

        if keyboard.is_pressed("e"):
            print("Mode 1. Robot orientation")
            tcp_pose = transl(Xr,Yr,Zr) * rotz(W) * roty(P) * rotx(R)
            if robot.MoveL_Test(robot.Joints(),tcp_pose)==0:
                robot.MoveL(tcp_pose,True)
                text_label.config(text="Mode 2. Robot orientation: R= {:.0f}".format(R*180/pi))
            else:
                print('Robot cannot reach the position')
                text_label.config(text="Mode 2. Robot orientation: Robot cannot reach the position")
            time.sleep(1)
        elif keyboard.is_pressed("u"):
            print("Gripper move up")
            text_label.config(text="Gripper move up")
            approach = robot.Pose()*transl(0,0,10)
            robot.MoveL(approach,False)
            time.sleep(1)
        elif keyboard.is_pressed("d"):
            print("Gripper move down")
            text_label.config(text="Gripper move down")
            approach = robot.Pose()*transl(0,0,-10)
            robot.MoveL(approach,False)
            time.sleep(1)
        elif not s2 and s1:
            print("Open Gripper")
            text_label.config(text="Open Gripper")
            needle.setParentStatic(base)
            time.sleep(1)
        elif not s1 and s2:
            print("Close Gripper")
            text_label.config(text="Close Gripper")
            needle.setParentStatic(gripper) # Static to maintain pose
            time.sleep(1)
        elif not s1 and not s2:     #Create the mode 2 configuration
            print("Mode 2. Gripper orientation")
            gripper_pose =  transl(Xg,Yg,Zg) * rotx(pi) * rotz(W) * roty(P) * rotx(R)
            #Set new Pose
            text_label.config(text="Mode 2. Gripper orientation: R= {:.0f}".format(R*180/pi))
            #text_label.config(text="Mode 2. Gripper orientation: R= "+str(R))
            gripper.setPose(gripper_pose)
            time.sleep(1)
        #time.sleep(0.1)
        else:
            print ("Waiting")
            text_label.config(text="Waiting")

try:
    # Discard initial ESP32 message
    arduino.reset_input_buffer()
    keyboard_thread = threading.Thread(target=check_keyboard)
    keyboard_thread.daemon = True
    keyboard_thread.start()

    # Start the GUI event loop
    root.mainloop()


except KeyboardInterrupt:
    print("Communication stopped.")
    pass

# ------------------------------------------------------------------------------
# Disconnect Arduino
# ------------------------------------------------------------------------------
print("Disconnecting Arduino...")
arduino.close()
print("Disconnecting UR5e...")
robot.Disconnect()
print("Communication stopped.")
needle.setParentStatic(base)
print("Open gripper.")

print("Disconnecting UR5e...")
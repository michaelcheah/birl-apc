#!/usr/bin/env python
# Demos
import serial
import socket
import time
import random
import copy
import numpy as np
import math

import interface_cmds as ic
import object_grasping as og
import ur_waypoints as uw
import vision_copy as vc

def smooth_rotate(c,ser_ee,ser_vac,orientation=0,angle_of_attack=0):
    # Select tcp_2, for rotations around the grasping point
    ic.socket_send(c,sCMD=101)

    # Convert to radians
    angle_of_attack=angle_of_attack*math.pi/180.0
    orientation=orientation*math.pi/180.0
    thetay=135.0*math.pi/180.0
    
    # Cartesian rotation matrices to match grabbing_joints rotation
    x_rot = np.matrix([[ 1.0, 0.0, 0.0],
             [ 0.0, math.cos(math.pi/2), -math.sin(math.pi/2)],
             [ 0.0, math.sin(math.pi/2), math.cos(math.pi/2)]]) # x_rot[rows][columns]
    y_rot = np.matrix([[ math.cos(thetay), 0.0, -math.sin(thetay)],
             [ 0.0, 1.0, 0.0],
             [ math.sin(thetay), 0.0, math.cos(thetay)]]) # y_rot[rows][columns]
    z_rot = np.matrix([[ math.cos(0.0), -math.sin(0.0), 0.0],
             [ math.sin(0.0), math.cos(0.0), 0.0],
             [ 0.0, 0.0, 1.0]]) # z_rot[rows][columns]

    # Create rotation matrix for current position
    R=z_rot*y_rot*x_rot

    # Axis rotation matricies for grasping position, rotate around x-axis by aoa, then z-axis by ori
    x_rot = np.matrix([[ 1.0, 0.0, 0.0],
             [ 0.0, math.cos(angle_of_attack), -math.sin(angle_of_attack)],
             [ 0.0, math.sin(angle_of_attack), math.cos(angle_of_attack)]]) # x_rot[rows][columns]
    z_rot = np.matrix([[ math.cos(orientation), -math.sin(orientation), 0.0],
             [ math.sin(orientation), math.cos(orientation), 0.0],
             [ 0.0, 0.0, 1.0]]) # z_rot[rows][columns]

    # Cartesian rotation matrix of desired orientation
    R=z_rot*x_rot*R

    # Cartesian to axis-angle
    theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1.0)/2)
    multi = 1 / (2 * math.sin(theta))
    rx = multi * (R[2, 1] - R[1, 2]) * theta * 180/math.pi
    ry = multi * (R[0, 2] - R[2, 0]) * theta * 180/math.pi
    rz = multi * (R[1, 0] - R[0, 1]) * theta * 180/math.pi
    print rx, ry, rz

    # Rotate around tool centre point defined by tcp_2
    current_Pose, current_Grip = ic.get_position(c,ser_ee,ser_vac,CMD=1)
    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":rx,"ry":ry,"rz":rz}
    msg = ic.safe_ur_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Speed=0.15,CMD=8)

def throwing_demo(c,ser_ee,ser_vac):
    demand_Grip = dict(uw.end_effector_home)
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.throwing_demo_joints1),Grip=demand_Grip,CMD=2)

    inp = raw_input("Continue?")
    time.sleep(3)
    demand_Grip["act"]=75
    demand_Grip["servo"]=0
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.throwing_demo_joints1),Grip=demand_Grip,CMD=2)

    time.sleep(3)
    demand_Grip["servo"]=80
    try:
        # Send formatted CMD
        c.send("("+str(uw.throwing_demo_joints2["x"])+","+str(uw.throwing_demo_joints2["y"])+","+str(uw.throwing_demo_joints2["z"])+","+str(uw.throwing_demo_joints2["rx"])+","+str(uw.throwing_demo_joints2["ry"])+","+str(uw.throwing_demo_joints2["rz"])+","+str(9)+","+str(0)+")");
    except socket.error as socketerror:
        print ".......................Some kind of error :(......................."

    time.sleep(0.5)
    ser_ee.flush
    # Set Grabber Servo position, min = 22, max = 127
    ser_ee.write("G" + chr(demand_Grip["servo"]) + "\n")
    # Wait for end effector arduino to finish
    while True:
        ipt = ser_ee.readline()
        print ipt
        if ipt == "done\r\n":
            break
    ser_ee.flush

    try:
        msg=c.recv(1024)
        print msg
        print ""
    except socket.error as socketerror:
        print ".......................Some kind of error :(......................."

    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.throwing_demo_joints1),Grip=demand_Grip,CMD=2)

def cal_test(p):
    #p1, inverse = pix3world_cal([225.0,649.0],[741.0,1209.0],[264.0,1237.0])
    p1, inverse = pix3world_cal([0.0,0.0],[183.0,191.0],[17.0,204.0])
    x, y = pix3world(p1, inverse, p)
    return x, y
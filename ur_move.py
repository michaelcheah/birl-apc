#!/usr/bin/env python
# End effector example:
# Send cmd code (G for grabber servo, A for DC motor actuator, V for vacuum pump)
# Then value:
# G: open 0-120 closed
# A: open 0-90 closed
# V: grab g/r release
import serial
import socket
import time
import random
import copy
import math

import cv2
from matplotlib import pyplot as plt
import imutils

from interface_cmds import *
from object_grasping import *
from ur_waypoints import *
from demos import *
from vision_copy import *
from naughts_and_crosses_demo import *

def initialize():
    #HOST = "169.254.103.235" # The remote host
    HOST = "192.168.1.105" # The remote host
    PORT = 30000 # The same port as used by the server

    print "Starting Program"

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT)) # Bind to the port
    s.listen(5) # Now wait for client connection.
    c, addr = s.accept() # Establish connection with client.

    print "Connected to UR"
   
    ser_ee = serial.Serial('COM5',9600)  # open serial port
    ser_vac = serial.Serial('COM3',9600)  # open serial port
    while ser_ee.is_open==False & ser_vac.is_open==False:
        print "Waiting for serial"
    print(ser_ee.name)         # check which port was really used
    print(ser_vac.name)         # check which port was really used
    time.sleep(2)
    print "Ready"

    return c, ser_ee, ser_vac

def main():
    c, ser_ee, ser_vac = initialize()
    # loop
    print c.recv(1024)
    inp = raw_input("Continue?")
    msg = safe_ur_move(c,ser_ee,ser_vac,Pose=copy.deepcopy(grab_home_joints),CMD=2)
    while True:
        task = raw_input("task: ")
        if task == "tally":
            nc_win = int(raw_input("win: "))
            nc_loss = int(raw_input("loss: "))
            nc_draw = int(raw_input("draw: "))
            update_tally(c,ser_ee,ser_vac,win=nc_win,loss=nc_loss,draw=nc_draw)
        if task == "gc":
            p = [0,0]
            p[0] = float(raw_input("pc_x :"))
            p[1] = float(raw_input("pc_y :"))
            X, Y = cal_test(p)
            cc = [X,Y]
            p[0] = float(raw_input("pe_x :"))
            p[1] = float(raw_input("pe_y :"))
            X, Y = cal_test(p)
            ce = [X,Y]
            X, Y, Z, ori, aoa, Size = get_grasping_coords(cc,ce,50)
            print X, Y, Z, ori, aoa, Size
            print grab_stow(c,ser_ee,ser_vac,X,Y,z=Z,orientation=ori,angle_of_attack=aoa,shelf=0,size=Size)
        if task == 'cal':
            p = [0,0]
            p[0] = float(raw_input("x :"))
            p[1] = float(raw_input("y :"))
            x, y = cal_test(p)
            print "world x = ",x
            print "world y = ",y
        if task == "circle":
            z = int(raw_input("z: "))
            sq = int(raw_input("sq: "))
            nc_robot_move(c,ser_ee,ser_vac,sq,z)
        if task == "nc":
            nc_win = int(raw_input("robot wins: "))
            nc_draw = int(raw_input("draws: "))
            nc_loss = int(raw_input("human wins: "))
            while True:
                st = int(raw_input("Robot/Human starts (0/1): "))
                game = naughts_crosses(c,ser_ee,ser_vac,start=st)
                print game
                for i in range(0,5):
                    current_Pose, current_Grip = get_position(c,ser_ee,ser_vac,CMD=1)
                    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2]-10,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
                    msg = safe_ur_move_only(c, ser_ee, ser_vac, Pose=demand_Pose, CMD=4)

                    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
                    msg = safe_ur_move_only(c, ser_ee, ser_vac, Pose=demand_Pose, CMD=4)
                if game == "................Robot Victory!................":
                    nc_win = nc_win+1
                    update_tally(c,ser_ee,ser_vac,win=nc_win)
                elif game == ".................Human victory................":
                    nc_loss = nc_loss+1
                    update_tally(c,ser_ee,ser_vac,loss=nc_loss)
                elif game == ".....................Draw.....................":
                    nc_draw = nc_draw+1
                    update_tally(c,ser_ee,ser_vac,draw=nc_draw)
        if task == "throw":
            throwing_demo(c,ser_ee,ser_vac)
        if task == "cam":
            check_camera()
        if task == "sr":
            # Select tcp_2, for rotations around the grasping point
            orientation=float(raw_input("ori: "))
            angle_of_attack=89.9
            socket_send(c,sCMD=101)

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

            # Move to grabbing waypoint
            demand_Grip = copy.deepcopy(end_effector_home)
            #demand_Grip["act"]=size-15
            msg = safe_ur_move(c,ser_ee,ser_vac,Pose=copy.deepcopy(grabbing_joints_waypoint),Grip=demand_Grip,CMD=2)

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
            current_Pose, current_Grip = get_position(c,ser_ee,ser_vac,CMD=1)
            demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":rx,"ry":ry,"rz":rz}
            msg = safe_ur_move(c,ser_ee,ser_vac,Pose=copy.deepcopy(demand_Pose),Grip=demand_Grip,CMD=8)
            '''current_Pose, current_Grip = get_position(c,ser_ee,ser_vac,CMD=1)
            demand_Pose = {"x": 0, "y": -600, "z":400, "rx": current_Pose[3], "ry": current_Pose[4], "rz": current_Pose[5]-1}
            msg = safe_ur_move(c,ser_ee,ser_vac,Pose=demand_Pose,CMD=4)
            while 1:
                ori = random.uniform(-30, 30)
                aoa = random.uniform(-30, 89)
                smooth_rotate(c,ser_ee,ser_vac,ori,aoa)
                time.sleep(1)
            '''
        if task == "wp":
            #y = float(raw_input("y: "))
            #z = float(raw_input("z: "))
            #shelf = int(raw_input("shelf: "))
            msg = safe_ur_move(c,ser_ee,ser_vac,Pose=copy.deepcopy(grabbing_joints_waypoint),CMD=2)

            '''current_Pose, current_Grip = get_position(c,ser_ee,ser_vac,CMD=1)
            demand_Pose = {"x": current_Pose[0], "y": current_Pose[1]+30, "z":current_Pose[2]+47, "rx": current_Pose[3], "ry": current_Pose[4], "rz": current_Pose[5]}
            msg = safe_ur_move(c,ser_ee,ser_vac,Pose=copy.deepcopy(demand_Pose),CMD=4)

            current_Pose, current_Grip = get_position(c,ser_ee,ser_vac,CMD=3)
            demand_Pose = {"x": current_Pose[0], "y": current_Pose[1], "z":current_Pose[2], "rx": current_Pose[3], "ry": current_Pose[4], "rz": current_Pose[5]}
            print "joints: ", demand_Pose
            '''
            #current_Pose, current_Grip = get_position(c,ser_ee,ser_vac,CMD=1)
            #demand_Pose = {"x": current_Pose[0], "y": current_Pose[1], "z":current_Pose[2], "rx": current_Pose[3], "ry": current_Pose[4], "rz": current_Pose[5]}
            #print "pose: ", demand_Pose
            
        if task == "grab_pick":
            #msg = safe_ur_move(c,ser_ee,ser_vac,Pose={"x": -6.24, "y": -80.55, "z": 124.09, "rx": -135.90, "ry": 85.69, "rz": -112.83},Grip=demand_Grip,CMD=2)
            #x = float(raw_input("x: "))
            #y = float(raw_input("y: "))
            #shelf = int(raw_input("shelf: "))
            #y = float(raw_input("y: "))
            #Z = float(raw_input("z: "))
            #ori = float(raw_input("ori: "))
            #print grab_pick(c,ser_ee,ser_vac,y,z=Z,orientation=ori)
            y = float(raw_input("y: "))
            Z = float(raw_input("z: "))
            height = int(raw_input("height: "))
            print grab_pick(c,ser_ee,ser_vac,y,z=Z,orientation=0,object_height=height)
        if task == "pose_trans":
            #theta = float(raw_input("theta: "))
            #current_Pose, current_Grip = get_position(c,ser_ee,ser_vac,CMD=1)
            #demand_Pose = {"x": current_Pose[0], "y": current_Pose[1], "z":current_Pose[2], "rx": theta, "ry": 0.0, "rz": 0.0}
            msg = safe_ur_move(c,ser_ee,ser_vac,Pose=copy.deepcopy(grab_home_joints),CMD=2)
            demand_Pose = {"x": 0.0, "y": 0.0, "z":0.0, "rx": 45.0, "ry": 0.0, "rz": 0.0}
            print "pose_trans: ", socket_send(c,Pose=copy.deepcopy(demand_Pose),CMD=8)
            #msg = safe_ur_move(c,ser_ee,ser_vac,Pose=copy.deepcopy(grabbing_joints_waypoint2),CMD=2)
        if task == "grab_stow":
            #msg = safe_ur_move(c,ser_ee,ser_vac,Pose={"x": -6.24, "y": -80.55, "z": 124.09, "rx": -135.90, "ry": 85.69, "rz": -112.83},Grip=demand_Grip,CMD=2)
            #x = float(raw_input("x: "))
            #y = float(raw_input("y: "))
            #shelf = int(raw_input("shelf: "))
            aoa = float(raw_input("aoa: "))
            ori = float(raw_input("ori: "))
            Shelf = int(raw_input("shelf: "))
            print grab_stow(c,ser_ee,ser_vac,-400,-400,orientation=ori,angle_of_attack=aoa,shelf=Shelf)
        #if task == "m":
        #    print matrix_multiply([[1,2,3],[4,5,6],[7,8,9]],[1,1,1])
        if task == "tilt":
            demand_Grip = copy.deepcopy(end_effector_home)
            demand_Grip["tilt"] = int(raw_input("tilt: "))
            msg = safe_ur_move(c,ser_ee,ser_vac,Pose=copy.deepcopy(grab_home_joints),Grip=demand_Grip,CMD=2)
        if task == "demo":
            demand_Grip = copy.deepcopy(end_effector_home)
            while True:
                msg = safe_ur_move(c,ser_ee,ser_vac,Pose=copy.deepcopy(grab_home_joints),Grip=demand_Grip,CMD=2)
                demand_Grip["tilt"] = 1
                msg = safe_ur_move(c,ser_ee,ser_vac,Pose=copy.deepcopy(grab_home_joints),Grip=demand_Grip,CMD=2)
                demand_Grip["act"] = 30
                msg = safe_ur_move(c,ser_ee,ser_vac,Pose=copy.deepcopy(grab_home_joints),Grip=demand_Grip,CMD=2)
                demand_Grip["servo"] = 127
                msg = safe_ur_move(c,ser_ee,ser_vac,Pose=copy.deepcopy(grab_home_joints),Grip=demand_Grip,CMD=2)
                demand_Grip["vac"] = "g"
                msg = safe_ur_move(c,ser_ee,ser_vac,Pose=copy.deepcopy(grab_home_joints),Grip=demand_Grip,CMD=2)
                demand_Grip["servo"] = 22
                msg = safe_ur_move(c,ser_ee,ser_vac,Pose=copy.deepcopy(grab_home_joints),Grip=demand_Grip,CMD=2)
                demand_Grip["tilt"] = 0
                msg = safe_ur_move(c,ser_ee,ser_vac,Pose=copy.deepcopy(grab_home_joints),Grip=demand_Grip,CMD=2)
                demand_Grip["vac"] = "r"
                msg = safe_ur_move(c,ser_ee,ser_vac,Pose=copy.deepcopy(grab_home_joints),Grip=demand_Grip,CMD=2)
                demand_Grip["act"] = 50
                msg = safe_ur_move(c,ser_ee,ser_vac,Pose=copy.deepcopy(grab_home_joints),Grip=demand_Grip,CMD=2)
        if task == "vac_stow":
            x = float(raw_input("x: "))
            y = float(raw_input("y: "))
            shelf = int(raw_input("shelf: "))
            print vac_stow(c, ser_ee, ser_vac, x, y, shelf)
        if task == "vac_pick":
            y = float(raw_input("y: "))
            z = float(raw_input("z: "))
            shelf = int(raw_input("shelf: "))
            print vac_pick(c, ser_ee, ser_vac, y, z, shelf)
        if task == "shelf":
            shelf = int(raw_input("shelf: "))
            #print safe_ur_move(c,ser_ee,ser_vac,Pose=copy.deepcopy(shelf_joints_waypoint),CMD=2)
            print safe_ur_move(c,ser_ee,ser_vac,Pose=copy.deepcopy(shelf_joints[shelf]),CMD=2)
            #print safe_ur_move(c,ser_ee,ser_vac,Pose=copy.deepcopy(shelf_joints_waypoint),CMD=2)
            #print safe_ur_move(c,ser_ee,ser_vac,Pose=copy.deepcopy(shelf_home_joints),CMD=2)
        if task == "shelf_home":
            print safe_ur_move(c,ser_ee,ser_vac,Pose=copy.deepcopy(shelf_joints_waypoint),CMD=2)
        if task == "home":
            msg = safe_ur_move(c,ser_ee,ser_vac,Pose=copy.deepcopy(grab_home_joints),CMD=2)
        if task == "force":
            print get_force(c)
        if task == "torque":
            print get_torque(c)
        if task == "pose":
            current_Pose, current_Grip = get_position(c,ser_ee,ser_vac,CMD=1)
            print ("current pose: ", current_Pose)
            print ("current grip: ", current_Grip)
        if task == "joints":
            current_Joints, current_Grip = get_position(c,ser_ee,ser_vac,CMD=3)
            print ("current joints: ", current_Joints)
            print ("current grip: ", current_Grip)
        if task == "move":
            demand_Pose = copy.deepcopy(grab_home)
            msg = safe_ur_move(c,ser_ee,ser_vac, CMD=4)
            current_Joints, current_Grip = get_position(c,ser_ee,ser_vac,CMD=3)
            demand_Joints = {"x":current_Joints[0]-90,"y":current_Joints[1],"z":current_Joints[2],"rx":current_Joints[3],"ry":current_Joints[4]+60,"rz":current_Joints[5]-45}
            msg = safe_ur_move(c, ser_ee, ser_vac, Pose=demand_Joints, CMD=2)

#            demand_Pose["x"] = float(raw_input("x: "))
#            demand_Pose["y"] = float(raw_input("y: "))
#            demand_Pose["z"] = float(raw_input("z: "))
#            demand_Pose["rx"] = float(raw_input("rx: "))
#            demand_Pose["ry"] = float(raw_input("ry: "))
            demand_Pose["rz"] = float(raw_input("rz: "))
            current_Joints, current_Grip = get_position(c,ser_ee,ser_vac,CMD=3)
            demand_Joints = {"x":current_Joints[0]+demand_Pose["rz"],"y":current_Joints[1],"z":current_Joints[2],"rx":current_Joints[3],"ry":current_Joints[4],"rz":current_Joints[5]}
            msg = safe_ur_move(c, ser_ee, ser_vac, Pose=demand_Joints, CMD=2)
        if task == "safe_move":
            demand_Pose = copy.deepcopy(grab_home)
            demand_Pose["x"] = random.uniform(-300, -500)
            demand_Pose["y"] = random.uniform(-300, -500)
            demand_Pose["z"] = random.uniform(50, 200)
            safe_ur_move(c,ser_ee,ser_vac,Pose=demand_Pose)
        if task == "grab":
            demand_Grip = copy.deepcopy(end_effector_home)
            demand_Grip["act"] = int(raw_input("act: "))
            demand_Grip["servo"] = int(raw_input("servo: "))
            demand_Grip["tilt"] = int(raw_input("tilt: "))
            demand_Grip["vac"] = raw_input("vac: ")
            msg = safe_ur_move(c,ser_ee,ser_vac,Grip=demand_Grip, CMD=0)
        if task == "banana":
            x = float(raw_input("x: "))
            y = float(raw_input("y: "))
            rz = float(raw_input("rz: "))
            print "Picking banana..."
            banana(c,ser_ee,ser_vac,x,y,rz)
        if task == "fork":
            x = float(raw_input("x: "))
            y = float(raw_input("y: "))
            z = float(raw_input("z: "))
            print "Picking fork..."
            fork(c,ser_ee,ser_vac,x,y,z)
#        current_Joints, current_Grip = get_position(c, ser_ee, ser_vac, CMD=3)
#        print "Joints: ", current_Joints
#        print "Grip: ", current_Grip
#        demand_Joints = {"x":current_Joints[0],"y":current_Joints[1],"z":current_Joints[2],"rx":current_Joints[3],"ry":current_Joints[4],"rz":current_Joints[5]-45}
#        ur_move(c, ser_ee, ser_vac,Pose = demand_Joints, CMD = 2)
#        inp = raw_input("Continue?")
#        current_Joints, current_Grip = get_position(c, ser_ee, ser_vac, CMD=3)
#        print "Joints: ", current_Joints
#        print "Grip: ", current_Grip
#        demand_Joints = {"x":current_Joints[0],"y":current_Joints[1],"z":current_Joints[2],"rx":current_Joints[3],"ry":current_Joints[4],"rz":current_Joints[5]+45}
#        ur_move(c, ser_ee, ser_vac,Pose = demand_Joints, CMD = 2)
    #    item, x, y, rz = object_recognition()

if __name__ == '__main__': main()
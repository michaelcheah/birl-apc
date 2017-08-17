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
    msg = safe_move(c,ser_ee,ser_vac,Pose=dict(grab_home_joints),CMD=2)
    while True:
        task = raw_input("task: ")
        if task == "gp":
            while True:
                ipt = int(raw_input("object 1-10: "))
                if ipt==1:
                    msg = vac_stow(c,ser_ee,ser_vac,-300,-400,1)
                    msg = vac_pick(c,ser_ee,ser_vac,-100,300,2)
                if ipt==2:
                    msg = vac_stow(c,ser_ee,ser_vac,-300,-400,4)
                    msg = vac_pick(c,ser_ee,ser_vac,-500,100,2)
                if ipt==3:
                    msg = vac_stow(c,ser_ee,ser_vac,-300,-400,1)
                    msg = vac_pick(c,ser_ee,ser_vac,-100,300,2)
                if ipt==4:
                    msg = vac_stow(c,ser_ee,ser_vac,-300,-400,1)
                    msg = vac_pick(c,ser_ee,ser_vac,-100,300,2)
                if ipt==5:
                    msg = vac_stow(c,ser_ee,ser_vac,-300,-400,1)
                    msg = vac_pick(c,ser_ee,ser_vac,-100,300,2)
                elif ipt==6:
                    msg = grab_stow(c,ser_ee,ser_vac,-200,-400,z=20,angle_of_attack=89.9,shelf=1,size=6)
                    msg = grab_pick(c,ser_ee,ser_vac,-320,z=300,orientation=0,object_height=48)
                elif ipt==7:
                    msg = grab_stow(c,ser_ee,ser_vac,-200,-400,z=6,angle_of_attack=89.9,shelf=1,size=12)
                elif ipt==8:
                    msg = grab_stow(c,ser_ee,ser_vac,-200,-400,z=15,angle_of_attack=89.9,shelf=1,size=20)
                elif ipt==9:
                    msg = grab_stow(c,ser_ee,ser_vac,-200,-400,z=8,angle_of_attack=89.9,shelf=1,size=25)
                elif ipt==10:
                    msg = grab_stow(c,ser_ee,ser_vac,-200,-400,z=20,angle_of_attack=89.9,shelf=1,size=50)
        if task == "dg":
            while True:
                demand_Pose = {"x": -200, "y": -400.0, "z": random.uniform(20,150), "rx": 0.0, "ry": 180.0, "rz": 0.0}
                msg = safe_ur_move(c,Pose=dict(demand_Pose),CMD=4)
                current_Pose = get_ur_position(c,1)
                for i in range(0,3):
                    if current_Pose[i]==0:
                        ipt=raw_input("Continue?")
        if task == "tr":
            success_rate = [[0,0,0,0,0,0,0,0,0,0],
                             [0,0,0,0,0,0,0,0,0,0]]
            while True:
                #initialize_waypoints()
                ipt = int(raw_input("object 1-10: "))
                if ipt < 6:
                    msg = vac_stow(c,ser_ee,ser_vac,-300,-400,1)
                elif ipt==6:
                    msg = grab_stow(c,ser_ee,ser_vac,-200,-400,z=20,angle_of_attack=89.9,shelf=1,size=6)
                elif ipt==7:
                    msg = grab_stow(c,ser_ee,ser_vac,-200,-400,z=6,angle_of_attack=89.9,shelf=1,size=12)
                elif ipt==8:
                    msg = grab_stow(c,ser_ee,ser_vac,-200,-400,z=15,angle_of_attack=89.9,shelf=1,size=20)
                elif ipt==9:
                    msg = grab_stow(c,ser_ee,ser_vac,-200,-400,z=8,angle_of_attack=89.9,shelf=1,size=25)
                elif ipt==10:
                    msg = grab_stow(c,ser_ee,ser_vac,-200,-400,z=20,angle_of_attack=89.9,shelf=1,size=50)
                ipt2 = int(raw_input("fail/success (0/1): "))
                success_rate[0][ipt-1]=success_rate[0][ipt-1]+1
                if ipt2 == 1:
                    success_rate[1][ipt-1]=success_rate[1][ipt-1]+1
                for i in range(0,10):
                    if success_rate[0][i]!=0:
                        print "object ",i+1,": ",float(success_rate[1][i])," ot of ",float(success_rate[0][i])
        if task == "clean":
            clean_board(c,ser_ee,ser_vac)
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
            X, Y, Z, ori, aoa, Size = get_grasping_coords(cc,ce,25)
            print X, Y, Z, ori, aoa, Size
            print grab_stow(c,ser_ee,ser_vac,X,Y,z=Z,orientation=ori,angle_of_attack=aoa,shelf=0,size=Size)
            print X, Y, Z, ori, aoa, Size
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
                    msg = safe_ur_move(c, ser_ee, ser_vac, Pose=demand_Pose, CMD=4)

                    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
                    msg = safe_ur_move(c, ser_ee, ser_vac, Pose=demand_Pose, CMD=4)
                if game == "................Robot Victory!................":
                    nc_win = nc_win+1
                    update_tally(c,ser_ee,ser_vac,win=nc_win)
                elif game == ".................Human victory................":
                    nc_loss = nc_loss+1
                    update_tally(c,ser_ee,ser_vac,loss=nc_loss)
                elif game == ".....................Draw.....................":
                    nc_draw = nc_draw+1
                    update_tally(c,ser_ee,ser_vac,draw=nc_draw)
        if task == "cam":
            check_camera()
        if task == "grab_pick":
            #shelf = int(raw_input("shelf: "))
            #ori = float(raw_input("ori: "))
            y = float(raw_input("y: "))
            Z = float(raw_input("z: "))
            height = int(raw_input("height: "))
            print grab_pick(c,ser_ee,ser_vac,y,z=Z,orientation=0,object_height=height)
        if task == "demo":
            demand_Grip = dict(end_effector_home)
            while True:
                msg = safe_move(c,ser_ee,ser_vac,Pose=dict(grab_home_joints),Grip=demand_Grip,CMD=2)
                demand_Grip["tilt"] = 1
                msg = safe_move(c,ser_ee,ser_vac,Pose=dict(grab_home_joints),Grip=demand_Grip,CMD=2)
                demand_Grip["act"] = 30
                msg = safe_move(c,ser_ee,ser_vac,Pose=dict(grab_home_joints),Grip=demand_Grip,CMD=2)
                demand_Grip["servo"] = 0
                msg = safe_move(c,ser_ee,ser_vac,Pose=dict(grab_home_joints),Grip=demand_Grip,CMD=2)
                demand_Grip["vac"] = "g"
                msg = safe_move(c,ser_ee,ser_vac,Pose=dict(grab_home_joints),Grip=demand_Grip,CMD=2)
                demand_Grip["servo"] = 80
                msg = safe_move(c,ser_ee,ser_vac,Pose=dict(grab_home_joints),Grip=demand_Grip,CMD=2)
                demand_Grip["tilt"] = 0
                msg = safe_move(c,ser_ee,ser_vac,Pose=dict(grab_home_joints),Grip=demand_Grip,CMD=2)
                demand_Grip["vac"] = "r"
                msg = safe_move(c,ser_ee,ser_vac,Pose=dict(grab_home_joints),Grip=demand_Grip,CMD=2)
                demand_Grip["act"] = 50
                msg = safe_move(c,ser_ee,ser_vac,Pose=dict(grab_home_joints),Grip=demand_Grip,CMD=2)
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
            #print safe_ur_move(c,ser_ee,ser_vac,Pose=dict(shelf_joints_waypoint),CMD=2)
            print safe_move(c,ser_ee,ser_vac,Pose=dict(shelf_joints[shelf]),CMD=2)
            #print safe_ur_move(c,ser_ee,ser_vac,Pose=dict(shelf_joints_waypoint),CMD=2)
            #print safe_ur_move(c,ser_ee,ser_vac,Pose=dict(shelf_home_joints),CMD=2)
        if task == "shelf_home":
            print safe_move(c,ser_ee,ser_vac,Pose=dict(shelf_joints_waypoint),CMD=2)
        if task == "home":
            msg = safe_move(c,ser_ee,ser_vac,Pose=dict(grab_home_joints),CMD=2)
        if task == "force":
            print get_force(c)
        if task == "torque":
            print get_torque(c)
        if task == "pose":
            current_Pose, current_Grip = get_position(c,ser_ee,ser_vac,CMD=1)
            print "current pose: ", current_Pose
            print "current grip: ", current_Grip
        if task == "joints":
            current_Joints, current_Grip = get_position(c,ser_ee,ser_vac,CMD=3)
            print "current joints: ", current_Joints
            print "current grip: ", current_Grip
        if task == "move":
            demand_Pose["x"] = float(raw_input("x: "))
            demand_Pose["y"] = float(raw_input("y: "))
            demand_Pose["z"] = float(raw_input("z: "))
            demand_Pose["rx"] = float(raw_input("rx: "))
            demand_Pose["ry"] = float(raw_input("ry: "))
            demand_Pose["rz"] = float(raw_input("rz: "))
            msg = safe_ur_move(c,ser_ee,ser_vac,Pose=demand_Joints,CMD=4)
        if task == "grab":
            demand_Grip = dict(end_effector_home)
            demand_Grip["act"] = int(raw_input("act: "))
            demand_Grip["servo"] = int(raw_input("servo: "))
            demand_Grip["tilt"] = int(raw_input("tilt: "))
            demand_Grip["vac"] = raw_input("vac: ")
            msg = safe_move(c,ser_ee,ser_vac,Grip=demand_Grip, CMD=0)

if __name__ == '__main__': main()
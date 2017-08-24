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

import interface_cmds as ic
import object_grasping as og
import ur_waypoints as uw
import demos
import vision_copy as vc
import naughts_and_crosses_demo as nc
import led_functions as lf

##################################### Vision Imports ###########################################
import numpy as np
import os
import vision_tools as vt
from vision_tools import normclean2cv2
import kinect_vision as kv
from kinect_vision import PATH_TO_KINECT_IMAGES_DIR
from image_processing import run_calibration, run_calibration_rgb
from image_processing import run_image_processing_v2_depth, run_image_processing_v2_rgb

from tableObject_class import TableObject, match_rgb_with_depth, match_rgb_with_depth_v2

def initialize():
    #HOST = "169.254.103.235" # The remote host
    #HOST = "192.168.1.105" # The remote host
    HOST = "169.254.242.158"
    PORT = 30000 # The same port as used by the server

    print "Starting Program"

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT)) # Bind to the port
    s.listen(5) # Now wait for client connection.
    c, addr = s.accept() # Establish connection with client.

    print "Connected to UR"
   
    ser_ee = serial.Serial('/dev/ttyACM0',9600)  # open serial port
    ser_vac = serial.Serial('/dev/ttyACM1',9600)  # open serial port
    ser_led = serial.Serial('/dev/ttyACM2',9600)  # open serial port
    while ser_ee.isOpen()==False & ser_vac.isOpen()==False & ser_led.isOpen()==False:
        print "Waiting for serial"
    print(ser_ee.name)         # check which port was really used
    print(ser_vac.name)         # check which port was really used
    print(ser_led.name)         # check which port was really used
    time.sleep(2)
    print "Ready"

    return c, ser_ee, ser_vac, ser_led

def main():
    c, ser_ee, ser_vac, ser_led = initialize()
    # loop
    ic.led_serial_send(ser_led,"I",2,0,2,0)
    print c.recv(1024)
    inp = raw_input("Continue?")
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.grab_home_joints),CMD=2)
    
    ##################### Vision Initialise #####################################
    directory = PATH_TO_KINECT_IMAGES_DIR
    
    cali = kv.load_npz_as_array("im_array_cal_FINAL", directory)
    empt = kv.load_npz_as_array("im_array_empty_FINAL", directory)
    
    empt_all = kv.prepare_im_array(empt)
    cali_all = kv.prepare_im_array(cali)

    depth_cali = run_calibration(empt_all, cali_all, adjust=False)
    rgb_cali = run_calibration_rgb(empt_all, cali_all, depth_cali, adjust=False)
    
    while True:
        task = raw_input("task: ")
        
        ################ Calibration Step if needed #######################################
        if task == "calibrate":
            
            calibrate_check1 = raw_input("Prepared for Empty Capture?: ")
            if calibrate_check1 == "yes":
                print "empty"
                empt = kv.capture_frames()
                plt.imshow(empt['ir'])
                plt.show()
                empt_all = kv.prepare_im_array(empt)
                rgb, depth, ir = empt_all
                np.savez(os.path.join(PATH_TO_KINECT_IMAGES_DIR, 'im_array_empty_FINAL'), rgb=rgb, depth=depth, ir=ir)

            calibrate_check2 = raw_input("Prepared for Calibrate Capture?: ")
            if calibrate_check2 == "yes":
                cali = kv.capture_frames()
                print "cali"
                cali_all = kv.prepare_im_array(cali)
                rgb, depth, ir = cali_all

                np.savez(os.path.join(PATH_TO_KINECT_IMAGES_DIR, 'im_array_cal_FINAL'), rgb=rgb, depth=depth, ir=ir)
            
            directory = PATH_TO_KINECT_IMAGES_DIR

            cali = kv.load_npz_as_array("im_array_cal_FINAL", directory)
            empt = kv.load_npz_as_array("im_array_empty_FINAL", directory)

            empt_all = kv.prepare_im_array(empt)
            cali_all = kv.prepare_im_array(cali)

            rgb, depth, ir = empt_all
            rgb, depth, ir = cali_all

            depth_cali = run_calibration(empt_all, cali_all, adjust=True)

            rgb_cali = run_calibration_rgb(empt_all, cali_all, depth_cali, adjust=True)
    
        if task == "gh":
            # Set tool to tcp_5
            ic.socket_send(c,sCMD=104)
    
            # Home
            demand_Pose = dict(uw.grab_home)
            demand_Grip = dict(uw.end_effector_home)
            msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.grab_home_joints),Grip=demand_Grip,CMD=2)

            # Rotate end effector
            current_Joints = ic.get_ur_position(c,3)
            demand_Joints = {"x":current_Joints[0],"y":current_Joints[1],"z":current_Joints[2],"rx":current_Joints[3],"ry":current_Joints[4]+60,"rz":current_Joints[5]-45}
            msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Joints),Grip=demand_Grip,CMD=2)

            # Avoid camera mount
            current_Pose = ic.get_ur_position(c,1)
            demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":150,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
            msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)

            # Rotate base
            current_Joints = ic.get_ur_position(c,3)
            demand_Joints = {"x":current_Joints[0]-90,"y":current_Joints[1],"z":current_Joints[2],"rx":current_Joints[3],"ry":current_Joints[4],"rz":current_Joints[5]}
            msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Joints),Grip=demand_Grip,CMD=2)

            x = float(raw_input("x: "))
            y = float(raw_input("y: "))
            
            # Move to above the object
            current_Pose = ic.get_ur_position(c,1)
            demand_Pose = {"x":x,"y":y,"z":100,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
            msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)
        if task == "ls":
            shelf = int(raw_input("cluster: "))
            ic.led_serial_send(ser_led,"C",shelf,255,255,255)
            for i in range(0,6):
                if i!= shelf:
                    ic.led_serial_send(ser_led,"C",i,0,0,0)
            ic.led_serial_send(ser_led,"S",1,0,0,0)
        if task == "led":
            while True:
                #led_serial_send(ser_led,"I",3,127,0,0)
                cmd = raw_input("cmd: ")
                shelf = int(raw_input("cluster: "))
                r = int(raw_input("r: "))
                g = int(raw_input("g: "))
                b = int(raw_input("b: "))
                ic.led_serial_send(ser_led,cmd,shelf,r,g,b)
        if task == "gp":
            while(1):
                capture_check = raw_input("Ready?: ")
                if capture_check == "yes":
                    test = kv.capture_frames()
                    test_all = kv.prepare_im_array(test)
                    break

            ######## Process Test Image and Retrieve Depth and Contour Information from Depth and RGB Data ##########

            rgb, depth, ir = test_all


            normclean, sorted_family = run_image_processing_v2_depth(test_all, 
                                                                     depth_cali, 
                                                                     show=False)

            rgbnormclean, rgb_family, test_rgbx_img = run_image_processing_v2_rgb(test_all, 
                                                                                  rgb_cali, 
                                                                                  depth_cali, 
                                                                                  show=False)

            ######## Clean the images and convert them so that they are cv2 compatible ############

            depth_normclean = normclean2cv2(normclean)
            rgb_normclean = normclean2cv2(rgbnormclean)
            
            
            test_rgb_img = vt.convert2rgb(test_rgbx_img)
            
            cv2.imwrite("test_rgb_img.jpg", test_rgb_img)

            ####### Create List of Objects and match the rgb and depth data ##########
            object_list = match_rgb_with_depth_v2(sorted_family, rgb_family, depth_normclean, test_rgb_img)
            

            
            #cv2.imwrite("test_rgb_img.jpg", test_rgb_img)
            cv2.imwrite("depth_normclean.jpg", depth_normclean)

            pick_obj = object_list['1']

            if pick_obj.height[0] == 0:
                ipt = 1
                print "Object is cd"
                x_pix = pick_obj.rgb_centre[0]
                y_pix = pick_obj.rgb_centre[1]
            else:
                if pick_obj.circularness > 0.70:
                    ipt = 4
                    print "Object is tape measure"
                    x_pix = pick_obj.centre[0]
                    y_pix = pick_obj.centre[1]
                else:
                    if pick_obj.rgb_aspect > 0.8:
                        ipt = 5
                        print "Object is a box"
                        x_pix = pick_obj.rgb_centre[0]
                        y_pix = pick_obj.rgb_centre[1]
                    else:
                        if pick_obj.rgb_aspect < 0.6:
                            ipt = 3
                            print "Object is eraser"
                            x_pix = pick_obj.centre[0]
                            y_pix = pick_obj.centre[1]
                        else:
                            ipt = 2
                            print "Object is book"
                            x_pix = pick_obj.rgb_centre[0]
                            y_pix = pick_obj.rgb_centre[1]
            
            print "~~~~~~~~~~~~~~ OBJECT ATTRIBUTES ~~~~~~~~~~~~~~~"
            print "Height:       ", pick_obj.height


            print "RGB Aspect:   ", pick_obj.rgb_aspect
            try:
                print "Circularness: ", pick_obj.circularness
            except:
                print "no depth"
            
            circles = depth_cali[4]
            cali_circles_init = circles-circles[0][0]
            cali_circles=[]
            for circ in cali_circles_init[0]:
                cali_circles.append([circ[0]/2, circ[1]/2])
            print x_pix, y_pix
            print cali_circles
            
            p=[x_pix,y_pix]
            
            plt.figure("Circles")
            cv2.circle(test_rgb_img,(int(x_pix),int(y_pix)),3,(0,0,255),1)
            cv2.circle(test_rgb_img,(int(x_pix),int(y_pix)),2,(0,0,255),1)
            plt.imshow(test_rgb_img)
            #plt.show()
            cv2.imwrite("test_rgb_img_centre.jpg", test_rgb_img)
            
            p1, inverse = vc.pix3world_cal(cali_circles[0],cali_circles[2], cali_circles[1])
            x,y = vc.pix3world(p1, inverse, p)
            x = x[0,0]
            y = y[0,0]
            print x,y
            
            ic.led_serial_send(ser_led,"I",2,0,0,0)
            clr = copy.deepcopy(uw.empty_led)
            #ipt = int(raw_input("object 1-10: "))
            #x = float(raw_input("x :"))
            #y = float(raw_input("y :"))
            if ipt==1: #cd
                clr[1]=[255,255,0]
                lf.illuminate_cluster(ser_led,1,colour=clr)
                msg = og.vac_stow(c,ser_ee,ser_vac,x+35,y,1,z=40)
                #msg = og.vac_pick(c,ser_ee,ser_vac,-100,300,2)
            if ipt==2: #book
                clr[4][0]=[255,0,0]
                clr[5][0]=[255,0,0]
                lf.illuminate_cluster(ser_led,1,colour=clr)
                msg = og.vac_stow(c,ser_ee,ser_vac,x,y,4,z=50)
                #msg = og.vac_pick(c,ser_ee,ser_vac,-500,100,2)
            if ipt==3: #erasor
                clr[4][0]=[0,0,255]
                clr[5][0]=[0,0,255]
                lf.illuminate_cluster(ser_led,1,colour=clr)
                msg = og.vac_stow(c,ser_ee,ser_vac,x,y,4,z=80)
                #msg = og.vac_pick(c,ser_ee,ser_vac,-100,300,2)
            if ipt==4: #tape_measure
                clr[1]=[0,255,0]
                lf.illuminate_cluster(ser_led,1,colour=clr)
                msg = og.vac_stow(c,ser_ee,ser_vac,x,y,1,z=90)
                #msg = og.vac_pick(c,ser_ee,ser_vac,-100,300,2)
            if ipt==5: #box
                clr[0]=[255,0,255]
                lf.illuminate_cluster(ser_led,1,colour=clr)
                msg = og.vac_stow(c,ser_ee,ser_vac,x,y,0,z=110)
                #msg = og.vac_pick(c,ser_ee,ser_vac,-100,300,2)
            elif ipt==6: #mug
                msg = og.grab_stow(c,ser_ee,ser_vac,-200,-400,z=20,angle_of_attack=89.9,shelf=1,size=6)
                msg = og.grab_pick(c,ser_ee,ser_vac,-320,z=300,orientation=0,object_height=48)
            elif ipt==7: #torch
                msg = og.grab_stow(c,ser_ee,ser_vac,-200,-400,z=6,angle_of_attack=89.9,shelf=1,size=12)
            elif ipt==8: #duct_tape
                msg = og.grab_stow(c,ser_ee,ser_vac,-200,-400,z=15,angle_of_attack=89.9,shelf=1,size=20)
            elif ipt==9: #banana
                msg = og.grab_stow(c,ser_ee,ser_vac,-200,-400,z=8,angle_of_attack=89.9,shelf=1,size=25)
            elif ipt==10: #tennis_ball
                msg = og.grab_stow(c,ser_ee,ser_vac,-200,-400,z=20,angle_of_attack=89.9,shelf=1,size=50)
            clr = copy.deepcopy(uw.empty_led)
            lf.illuminate_cluster(ser_led,1,colour=clr)
            ic.led_serial_send(ser_led,"I",2,0,2,0)
        if task == "dg":
            while True:
                demand_Pose = {"x": -200, "y": -400.0, "z": random.uniform(20,150), "rx": 0.0, "ry": 180.0, "rz": 0.0}
                msg = ic.safe_ur_move(c,Pose=dict(demand_Pose),CMD=4)
                current_Pose = ic.get_ur_position(c,1)
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
                    msg = og.vac_stow(c,ser_ee,ser_vac,-300,-400,1)
                elif ipt==6:
                    msg = og.grab_stow(c,ser_ee,ser_vac,-200,-400,z=20,angle_of_attack=89.9,shelf=1,size=6)
                elif ipt==7:
                    msg = og.grab_stow(c,ser_ee,ser_vac,-200,-400,z=6,angle_of_attack=89.9,shelf=1,size=12)
                elif ipt==8:
                    msg = og.grab_stow(c,ser_ee,ser_vac,-200,-400,z=15,angle_of_attack=89.9,shelf=1,size=20)
                elif ipt==9:
                    msg = og.grab_stow(c,ser_ee,ser_vac,-200,-400,z=8,angle_of_attack=89.9,shelf=1,size=25)
                elif ipt==10:
                    msg = og.grab_stow(c,ser_ee,ser_vac,-200,-400,z=20,angle_of_attack=89.9,shelf=1,size=50)
                ipt2 = int(raw_input("fail/success (0/1): "))
                success_rate[0][ipt-1]=success_rate[0][ipt-1]+1
                if ipt2 == 1:
                    success_rate[1][ipt-1]=success_rate[1][ipt-1]+1
                for i in range(0,10):
                    if success_rate[0][i]!=0:
                        print "object ",i+1,": ",float(success_rate[1][i])," ot of ",float(success_rate[0][i])
        if task == "clean":
            nc.clean_board(c,ser_ee,ser_vac)
        if task == "tally":
            nc_win = int(raw_input("win: "))
            nc_loss = int(raw_input("loss: "))
            nc_draw = int(raw_input("draw: "))
            nc.update_tally(c,ser_ee,ser_vac,win=nc_win,loss=nc_loss,draw=nc_draw)
        if task == "gc":
            p = [0,0]
            p[0] = float(raw_input("pc_x :"))
            p[1] = float(raw_input("pc_y :"))
            X, Y = demos.cal_test(p)
            cc = [X,Y]
            p[0] = float(raw_input("pe_x :"))
            p[1] = float(raw_input("pe_y :"))
            X, Y = demos.cal_test(p)
            ce = [X,Y]
            X, Y, Z, ori, aoa, Size = og.get_grasping_coords(cc,ce,25)
            print X, Y, Z, ori, aoa, Size
            print og.grab_stow(c,ser_ee,ser_vac,X,Y,z=Z,orientation=ori,angle_of_attack=aoa,shelf=0,size=Size)
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
                    current_Pose, current_Grip = ic.get_position(c,ser_ee,ser_vac,CMD=1)
                    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2]-10,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
                    msg = ic.safe_ur_move(c, ser_ee, ser_vac, Pose=demand_Pose, CMD=4)

                    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
                    msg = ic.safe_ur_move(c, ser_ee, ser_vac, Pose=demand_Pose, CMD=4)
                if game == "................Robot Victory!................":
                    nc_win = nc_win+1
                    nc.update_tally(c,ser_ee,ser_vac,win=nc_win)
                elif game == ".................Human victory................":
                    nc_loss = nc_loss+1
                    nc.update_tally(c,ser_ee,ser_vac,loss=nc_loss)
                elif game == ".....................Draw.....................":
                    nc_draw = nc_draw+1
                    nc.update_tally(c,ser_ee,ser_vac,draw=nc_draw)
        if task == "cam":
            check_camera()
        if task == "grab_pick":
            #shelf = int(raw_input("shelf: "))
            #ori = float(raw_input("ori: "))
            y = float(raw_input("y: "))
            Z = float(raw_input("z: "))
            height = int(raw_input("height: "))
            print og.grab_pick(c,ser_ee,ser_vac,y,z=Z,orientation=0,object_height=height)
        if task == "demo":
            demand_Grip = dict(end_effector_home)
            while True:
                msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.grab_home_joints),Grip=demand_Grip,CMD=2)
                demand_Grip["tilt"] = 1
                msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.grab_home_joints),Grip=demand_Grip,CMD=2)
                demand_Grip["act"] = 30
                msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.grab_home_joints),Grip=demand_Grip,CMD=2)
                demand_Grip["servo"] = 0
                msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.grab_home_joints),Grip=demand_Grip,CMD=2)
                demand_Grip["vac"] = "g"
                msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.grab_home_joints),Grip=demand_Grip,CMD=2)
                demand_Grip["servo"] = 80
                msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.grab_home_joints),Grip=demand_Grip,CMD=2)
                demand_Grip["tilt"] = 0
                msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.grab_home_joints),Grip=demand_Grip,CMD=2)
                demand_Grip["vac"] = "r"
                msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.grab_home_joints),Grip=demand_Grip,CMD=2)
                demand_Grip["act"] = 50
                msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.grab_home_joints),Grip=demand_Grip,CMD=2)
        if task == "vac_stow":
            x = float(raw_input("x: "))
            y = float(raw_input("y: "))
            shelf = int(raw_input("shelf: "))
            print og.vac_stow(c, ser_ee, ser_vac, x, y, shelf)
        if task == "vac_pick":
            y = float(raw_input("y: "))
            z = float(raw_input("z: "))
            shelf = int(raw_input("shelf: "))
            print og.vac_pick(c, ser_ee, ser_vac, y, z, shelf)
        if task == "shelf":
            shelf = int(raw_input("shelf: "))
            #print ic.safe_ur_move(c,ser_ee,ser_vac,Pose=dict(shelf_joints_waypoint),CMD=2)
            print ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.shelf_joints[shelf]),CMD=2)
            #print ic.safe_ur_move(c,ser_ee,ser_vac,Pose=dict(shelf_joints_waypoint),CMD=2)
            #print ic.safe_ur_move(c,ser_ee,ser_vac,Pose=dict(shelf_home_joints),CMD=2)
        if task == "shelf_home":
            print ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.shelf_joints_waypoint),CMD=2)
        if task == "home":
            msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.grab_home_joints),CMD=2)
        if task == "force":
            print get_force(c)
        if task == "torque":
            print get_torque(c)
        if task == "pose":
            current_Pose, current_Grip = ic.get_position(c,ser_ee,ser_vac,CMD=1)
            print "current pose: ", current_Pose
            print "current grip: ", current_Grip
        if task == "joints":
            current_Joints, current_Grip = ic.get_position(c,ser_ee,ser_vac,CMD=3)
            print "current joints: ", current_Joints
            print "current grip: ", current_Grip
        if task == "move":
            demand_Pose["x"] = float(raw_input("x: "))
            demand_Pose["y"] = float(raw_input("y: "))
            demand_Pose["z"] = float(raw_input("z: "))
            demand_Pose["rx"] = float(raw_input("rx: "))
            demand_Pose["ry"] = float(raw_input("ry: "))
            demand_Pose["rz"] = float(raw_input("rz: "))
            msg = ic.safe_ur_move(c,ser_ee,ser_vac,Pose=demand_Joints,CMD=4)
        if task == "grab":
            demand_Grip = dict(uw.end_effector_home)
            demand_Grip["act"] = int(raw_input("act: "))
            demand_Grip["servo"] = int(raw_input("servo: "))
            demand_Grip["tilt"] = int(raw_input("tilt: "))
            demand_Grip["vac"] = raw_input("vac: ")
            msg = ic.safe_move(c,ser_ee,ser_vac,Grip=demand_Grip, CMD=0)

if __name__ == '__main__': main()

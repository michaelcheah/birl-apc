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
from object_data_csv import create_object_df, prepare_object_dict, add_to_csv, create_csv

from tableObject_class import TableObject, match_rgb_with_depth, match_rgb_with_depth_v2

import grasping_points as gp

object_ipt_dict = {'cd':      [1,'rgb'],
                   'book':    [2,'rgb'],
                   'eraser':  [3,'rgb'],
                   'measure': [4,'depth'],
                   'box':     [5,'rgb'],
                   'mug':     [6,'depth'],
                   'torch':   [7,'depth'],
                   'tape':    [8,'depth'],
                   'banana':  [9,'depth'],
                   'ball':    [10,'depth']}

def initialize():
    #HOST = "169.254.103.235" # The remote host
    #HOST = "192.168.1.105" # The remote host
    HOST = "169.254.242.158"
    PORT = 30000 # The same port as used by the server

    print ".......................Starting Program......................."
    print ""

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT)) # Bind to the port
    s.listen(5) # Now wait for client connection.
    c, addr = s.accept() # Establish connection with client.

    print "Connected to UR"
    print ""
   
    ser_ee = serial.Serial('/dev/ttyACM0',9600)  # open serial port
    ser_vac = serial.Serial('/dev/ttyACM1',9600)  # open serial port
    ser_led = serial.Serial('/dev/ttyACM2',9600)  # open serial port
    while ser_ee.isOpen()==False & ser_vac.isOpen()==False & ser_led.isOpen()==False:
        print "Waiting for serial"
    print ser_ee.name, ": ",ser_ee.readline()         # check which port was really used
    print ser_vac.name, ": ",ser_vac.readline()             # check which port was really used
    print ser_led.name, ": ",ser_led.readline()             # check which port was really used
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
        
        if task == "lf":
            lf.illuminate_cluster(ser_led,4,colour=[[0,0,0],[255,103,23],[0,0,0],[0,0,0],[255,103,23],[0,0,0]])
        
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
            socket_send(c,sCMD=104)
    
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
                #ic.led_serial_send(ser_led,"I",3,127,0,0)
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
            
            import object_recognition_tools as ort

            excluded_val = ['centre', 'rgb_centre', 'number of children']
            extras = ['R','G','B', 'centre_offset']

            obj_features_mean, obj_features_std = ort.prepare_obj_features_param(obj_feat_csv = 'object_features.csv', 
                                                                             excluded_val = excluded_val, 
                                                                             extras=extras)

            rec_df = ort.prepare_pick_obj_features_param(object_list, excluded_val, extras)
            cost_list = ort.create_cost_list(obj_features_mean, obj_features_std, rec_df)
            object_list = ort.label_object_list(object_list, cost_list, test_rgb_img, show=True)

            for item in object_list.keys():
                if object_list[item].height[0] == 0:
                    #if abs(object_list[item].rgb_area - 3*obj_features_mean.rgb_area.book)/obj_features_std.rgb_area.book > 1:
                        #print "TOO SMALL/BIG TO BE BOOK"
                        #del object_list[item]
                    if object_list[item].name != 'cd':
                        print "TO BE DELETED: ",item
                        del object_list[item]
                        
            if len(object_list) == 0:
                "THERE IS NOTHING INSIDE!!!!!!!!!!!"
            
            pick_obj = object_list.values()[0]
            
            print "==========================================="
            print "        OBJECT IS: ", pick_obj.name
            print "==========================================="
            
            object_df = create_object_df()
            create_csv("testing_object_features", object_df)
            for item in object_list.keys():
                object_dict = prepare_object_dict(object_list[item], object_df)
                object_df = object_df.append(object_dict, ignore_index=True)

            add_to_csv("testing_object_features.csv", object_df)
            
            ########### Find Circles for Pix3world transformation ################
            circles = depth_cali[4]
            cali_circles_init = circles-circles[0][0]
            cali_circles=[]
            for circ in cali_circles_init[0]:
                cali_circles.append([circ[0]/2, circ[1]/2])

            print cali_circles
            
            p1, inverse = vc.pix3world_cal(cali_circles[0],cali_circles[2], cali_circles[1])
            
            
            ################ Find the suction or grasping points in pixels ###########
            ipt = object_ipt_dict[pick_obj.name][0]
            
            if ipt>5:
                print "OBJECT TO BE PICKED BY GRASPING"
                first_node, node1, node2 = gp.first_grasping_point(pick_obj)
                current_line = gp.find_perpendicular_line(node1, node2[0])
                possible_pairs = gp.find_possible_cross_pairs(pick_obj, first_node, current_line)
                possible_pairs = gp.remove_duplicates(possible_pairs, node1, node2[0])
                possible_second_node, possible_grasp_centre = gp.find_second_grasping_point(possible_pairs, 
                                                                                            first_node, 
                                                                                            pick_obj)
                second_node, grasp_centre = gp.determine_best_grasping_point(possible_second_node, 
                                                                             possible_grasp_centre,
                                                                             first_node)
                
                if ipt==7:
                    first_node, second_node = gp.fix_torch_orientation(pick_obj, rgb_normclean, first_node, second_node)
                    
                gp.display_grasping_points(test_rgb_img, first_node, second_node, grasp_centre, pick_obj, show=True)
                
                p_cc = [first_node[0], first_node[1]]
                X, Y = vc.pix3world(p1, inverse, p_cc)
                cc = X,Y
                
                p_ce = [second_node[0], second_node[1]]
                X, Y = vc.pix3world(p1, inverse, p_ce)
                ce = X,Y
                
                X, Y, Z, ori, aoa, Size = og.get_grasping_coords(cc,ce,25)
                
            else:
                print "OBJECT TO BE PICKED BY SUCTION"
                if object_ipt_dict[pick_obj.name][1] == 'rgb':
                    x_pix = pick_obj.rgb_centre[0]
                    y_pix = pick_obj.rgb_centre[1]
                else:
                    x_pix = pick_obj.centre[0]
                    y_pix = pick_obj.centre[1]
                
                p = [x_pix, y_pix]
                x,y = vc.pix3world(p1, inverse, p)
                x = x[0,0]
                y = y[0,0]
                
                plt.figure("Circles")
                show_img = copy.copy(test_rgb_img)
                cv2.circle(show_img,(int(x_pix),int(y_pix)),3,(0,0,255),1)
                cv2.circle(show_img,(int(x_pix),int(y_pix)),2,(0,0,255),1)
                plt.imshow(show_img)
                #plt.show()
                cv2.imwrite("test_rgb_img_centre.jpg", show_img)
                
            
            print "~~~~~~~~~~~~~~ OBJECT ATTRIBUTES ~~~~~~~~~~~~~~~"
            if ipt>5:
                print "GRASPING"
            else:
                print "SUCTION"
            print "Height:       ", pick_obj.height
            print "RGB Aspect:   ", pick_obj.rgb_aspect
            try:
                print "Circularness: ", pick_obj.circularness
            except:
                print "no depth"
            
            #ipt = int(raw_input("object 1-10: "))
            #x = float(raw_input("x :"))
            #y = float(raw_input("y :"))
            ic.led_serial_send(ser_led,"I",2,0,0,0)
            clr = copy.deepcopy(uw.empty_led)
            #ipt = int(raw_input("object 1-10: "))
            #x = float(raw_input("x :"))
            #y = float(raw_input("y :"))
            if ipt==1: #cd
                clr[0]=[255,255,0]
                lf.illuminate_cluster(ser_led,1,colour=clr)
                msg,sx,sy,sz = og.vac_stow(c,ser_ee,ser_vac,ser_led,x+35,y,0,z=50)
                msg = og.vac_pick(c,ser_ee,ser_vac,ser_led,sy,sz,4,x=sx)
            if ipt==2: #book
                clr[4]=[255,0,0]
                clr[5]=[255,0,0]
                lf.illuminate_cluster(ser_led,1,colour=clr)
                msg,sx,sy,sz = og.vac_stow(c,ser_ee,ser_vac,ser_led,x,y,4,z=60,yoff=-100)
                msg = og.vac_pick(c,ser_ee,ser_vac,ser_led,sy,sz,4,x=sx)
            if ipt==3: #erasor
                clr[1]=[0,0,255]
                lf.illuminate_cluster(ser_led,1,colour=clr)
                msg,sx,sy,sz = og.vac_stow(c,ser_ee,ser_vac,ser_led,x,y,1,z=80,yoff=-21)
                msg = og.vac_pick(c,ser_ee,ser_vac,ser_led,y,z,4,x=X)
            if ipt==4: #tape_measure
                clr[2]=[0,255,0]
                lf.illuminate_cluster(ser_led,1,colour=clr)
                msg,,sx,sy,sz = og.vac_stow(c,ser_ee,ser_vac,ser_led,x,y,2,z=90,yoff=-21)
                msg = og.vac_pick(c,ser_ee,ser_vac,ser_led,sy,sz,4,x=sx)
            if ipt==5: #box
                clr[3]=[255,0,255]
                lf.illuminate_cluster(ser_led,1,colour=clr)
                msg,sx,sy,sz = og.vac_stow(c,ser_ee,ser_vac,ser_led,x,y,3,z=110,yoff=-21)
                msg = og.vac_pick(c,ser_ee,ser_vac,ser_led,sy,sz,4,x=sx)
            elif ipt==6: #mug
                clr[0]=[255,64,0]
                lf.illuminate_cluster(ser_led,1,colour=clr)
                msg,sx,sy,sz = og.grab_stow(c,ser_ee,ser_vac,ser_led,X,Y,z=20,angle_of_attack=89.9,orientation=ori,shelf=0,size=6)
                msg = og.grab_pick(c,ser_ee,ser_vac,ser_led,sy-20,z=sz,orientation=0,object_height=65.0,n=2)
            elif ipt==7: #torch
                clr[1]=[0,255,255]
                lf.illuminate_cluster(ser_led,1,colour=clr)
                msg,sx,sy,sz = og.grab_stow(c,ser_ee,ser_vac,ser_led,X,Y,z=6,angle_of_attack=89.9,orientation=ori,shelf=1,size=12,xoff=0,zoff=0,obj=ipt)
                msg = og.grab_pick(c,ser_ee,ser_vac,ser_led,sz,z=sz,orientation=0,object_height=19.0,xoff=0,zoff=0,n=2,obj=ipt,stored_x=sx)
            elif ipt==8: #duct_tape
                clr[2]=[64,255,0]
                lf.illuminate_cluster(ser_led,1,colour=clr)
                msg,sx,sy,sz = og.grab_stow(c,ser_ee,ser_vac,ser_led,X,Y,z=15,angle_of_attack=89.9,orientation=ori,shelf=2,size=20)
                msg = og.grab_pick(c,ser_ee,ser_vac,ser_led,sy-32,z=sz,orientation=0,object_height=48.0,xoff=10,n=2)
            elif ipt==9: #banana
                clr[3]=[64,64,255]
                lf.illuminate_cluster(ser_led,1,colour=clr)
                msg,sx,sy,sz = og.grab_stow(c,ser_ee,ser_vac,ser_led,X,Y,z=8,angle_of_attack=89.9,orientation=ori,shelf=3,size=25,xoff=20,zoff=-30)
                msg = og.grab_pick(c,ser_ee,ser_vac,ser_led,sy-17,z=sz,orientation=0,object_height=33.0,xoff=30,n=2)
            elif ipt==10: #tennis_ball
                clr[4]=[255,0,64]
                clr[5]=[255,0,64]
                lf.illuminate_cluster(ser_led,1,colour=clr)
                msg,sx,sy,sz = og.grab_stow(c,ser_ee,ser_vac,ser_led,X,Y,z=20,angle_of_attack=89.9,orientation=ori,shelf=4,size=50,zoff=-45,obj=ipt)
                msg = og.grab_pick(c,ser_ee,ser_vac,ser_led,sy,z=sz,orientation=0,object_height=65.0,n=2,obj=ipt,stored_x=sx)
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
                game = nc.naughts_crosses(c,ser_ee,ser_vac,start=st)
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
            demand_Grip = dict(uw.end_effector_home)
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
            print ic.vac_stow(c, ser_ee, ser_vac, x, y, shelf)
        if task == "vac_pick":
            y = float(raw_input("y: "))
            z = float(raw_input("z: "))
            shelf = int(raw_input("shelf: "))
            print og.vac_pick(c, ser_ee, ser_vac, y, z, shelf)
        if task == "shelf":
            shelf = int(raw_input("shelf: "))
            #print ic.safe_ur_move(c,ser_ee,ser_vac,Pose=dict(uw.shelf_joints_waypoint),CMD=2)
            print ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.shelf_joints[shelf]),CMD=2)
            #print ic.safe_ur_move(c,ser_ee,ser_vac,Pose=dict(uw.shelf_joints_waypoint),CMD=2)
            #print ic.safe_ur_move(c,ser_ee,ser_vac,Pose=dict(shelf_home_joints),CMD=2)
        if task == "shelf_home":
            print ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.shelf_joints_waypoint),CMD=2)
        if task == "home":
            msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.grab_home_joints),CMD=2)
        if task == "force":
            print ic.get_force(c)
        if task == "torque":
            print ic.get_torque(c)
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
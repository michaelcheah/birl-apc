#!/usr/bin/env python
# Demos
import serial
import socket
import time
import random
import copy
import numpy as np
import math

from interface_cmds import *
from object_grasping import *
from ur_waypoints import *
from vision_copy import *

def naughts_crosses(c,ser_ee,ser_vac,start=0):
    msg = safe_ur_move(c,ser_ee,ser_vac,Pose=dict(naughts_crosses_cam_joints),CMD=2)

    z_height = z_cal(c,ser_ee,ser_vac)

    print z_height

    #ipt = raw_input("Continue?")

    time.sleep(0.2)
    empty_grid,top_left,bottom_right = cal_grid_im("empty.jpg")

    grid_state=[0,0,0,
                0,0,0,
                0,0,0]

    if start==0:
        #draw
        #set grid_state
        grid_state, sq = nc_pick_move(dict(grid_state)) 
        nc_robot_move(c,ser_ee,ser_vac,sq,z_height)
        print "completed move"
        print grid_state[0:3]
        print grid_state[3:6]
        print grid_state[6:9]

        msg = safe_ur_move(c,ser_ee,ser_vac,Pose=dict(naughts_crosses_cam_joints),CMD=2)
        time.sleep(0.2)

    grid_ref = get_grid_im("grid_ref.jpg",top_left,bottom_right)

    current_Pose, current_Grip = get_position(c,ser_ee,ser_vac,CMD=1)
    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2]-10,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = safe_ur_move(c, ser_ee, ser_vac, Pose=demand_Pose, CMD=4)

    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = safe_ur_move(c, ser_ee, ser_vac, Pose=demand_Pose, CMD=4)
    
    while min(grid_state) == 0:
        # Wait for player move
        print "waiting for player move"
        grid_state = get_nc_state(empty_grid,grid_ref,top_left,bottom_right,dict(grid_state))
        print grid_state[0:3]
        print grid_state[3:6]
        print grid_state[6:9]
        # Check game state
        win = nc_finish(dict(grid_state))
        if win==1:
            return "................Robot Victory!................"
        if win==2: 
            return ".................Human victory................"
        if min(grid_state) != 0:
            break

        #draw
        #set grid_state
        grid_state, sq = nc_pick_move(dict(grid_state)) 
        nc_robot_move(c,ser_ee,ser_vac,sq,z_height)
        print "completed move"
        print grid_state[0:3]
        print grid_state[3:6]
        print grid_state[6:9]
        msg = safe_ur_move(c,ser_ee,ser_vac,Pose=dict(naughts_crosses_cam_joints),CMD=2)
        time.sleep(0.2)
        grid_ref = get_grid_im("grid_ref.jpg",top_left,bottom_right)
        # Check game state
        win = nc_finish(dict(grid_state))
        if win==1:
            return "Robot Victory!"
        if win==2: 
            return "fleshbag victory"

        current_Pose, current_Grip = get_position(c,ser_ee,ser_vac,CMD=1)
        demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2]-10,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
        msg = safe_ur_move(c, ser_ee, ser_vac, Pose=demand_Pose, CMD=4)

        demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
        msg = safe_ur_move(c, ser_ee, ser_vac, Pose=demand_Pose, CMD=4)

    return ".....................Draw....................."

def get_nc_state(empty_grid,ref_image,top_left,bottom_right, grid):
    grid_state = dict(grid)
    change = 0
    n = 0
    new_grid = [0,0,0,
                0,0,0,
                0,0,0]
    grid_im1 = get_grid_im("grid1.jpg",top_left,bottom_right)
    grid1 = read_grid(empty_grid,grid_im1,grid_state)
    time.sleep(1)
    grid_im2 = get_grid_im("grid2.jpg",top_left,bottom_right)
    grid2 = read_grid(empty_grid,grid_im2,grid_state)
    time.sleep(1)
    grid_im3 = get_grid_im("grid3.jpg",top_left,bottom_right)
    grid3 = read_grid(empty_grid,grid_im3,grid_state)
    time.sleep(1)
    while True:
        change = 0
        new_grid=dict(grid_state)
        for i in range(0,9):        
            if grid1[i]==2 and grid2[i]==2 and grid3[i]==2 and grid_state[i]==0:
                new_grid[i] = 2
                change = change+1
        #print "old grid: ",grid_state
        #print "new grid: ",new_grid
        #print "change: ",change
        if change==1: break
        if n==0:
            grid_im1 = get_grid_im("grid1.jpg",top_left,bottom_right)
            grid1 = read_grid(empty_grid,grid_im1,grid_state)
            #print "grid1: ",grid1
            n = n+1
        elif n==1:
            grid_im2 = get_grid_im("grid2.jpg",top_left,bottom_right)
            grid2 = read_grid(empty_grid,grid_im2,grid_state)
            #print "grid2: ",grid2
            n = n+1
        else:
            grid_im3 = get_grid_im("grid3.jpg",top_left,bottom_right)
            grid3 = read_grid(empty_grid,grid_im3,grid_state)
            #print "grid3: ",grid3
            n = 0
        time.sleep(0.5)
    return new_grid

def get_grid_im(name,top_left,bottom_right):
    #plt.figure(1)
    capture_pic(name)
    #plt.show()
    image = cv2.imread(name)
    
    grid_im={}
    for i in range (0,9):
        xoff = (bottom_right[0]-top_left[0])*((i%3)-1)
        yoff = (bottom_right[1]-top_left[1])*(i/3-1)
        grid_im[i]=(image[top_left[1]+yoff+10:bottom_right[1]+yoff-10,top_left[0]+xoff+10:bottom_right[0]+xoff-10])
       

    dim1 = int(np.ceil(np.sqrt(len(grid_im))))
    dim2 = int(np.ceil(float(len(grid_im))/dim1))
    dim = (str(dim1) + str(dim2))
    #print (len(grid_im), dim)
    if len(grid_im)>0:
        plt.figure(1)
        num = 1
        for (key) in grid_im:
            #print key, np.shape(frame[key])
            plt.subplot(int(str(dim)+str(num))), plt.imshow(grid_im[key])
            plt.title(str(key)), plt.xticks([]), plt.yticks([])
            num = num + 1
    #plt.show(1)
    return grid_im

def cal_grid_im(name):
    #plt.figure(1)
    capture_pic(name)
    #plt.show()
    image = cv2.imread(name)
    cntrs, points = extract_contours(image)
    plt.figure(1)
    plt.imshow(image)
    plt.show(1)
    #empty = np.zeros_like(image)
    #cv2.drawContours(empty, points, -1, (255,0,0), -1)
    #plt.figure()
    #plt.imshow(empty)
    #plt.show()
    boxes = find_box(points,minsize=7000,maxsize=9500,squareness=1.3,fill=0.90)

    #print boxes

    for (i,box) in enumerate(boxes):
        cv2.drawContours(image, [box], -1, (0, 255, 0), 2)

    top_left = [400,400]
    bottom_right = [0,0]
    for i in range(0,4):
        if boxes[0][i][0]<top_left[0]: top_left[0]=boxes[0][i][0]
        if boxes[0][i][1]<top_left[1]: top_left[1]=boxes[0][i][1]
        if boxes[0][i][0]>bottom_right[0]: bottom_right[0]=boxes[0][i][0]
        if boxes[0][i][1]>bottom_right[1]: bottom_right[1]=boxes[0][i][1]

    #print top_left
    #print bottom_right

    grid_im={}
    for i in range (0,9):
        xoff = (bottom_right[0]-top_left[0])*((i%3)-1)
        yoff = (bottom_right[1]-top_left[1])*(i/3-1)
        grid_im[i]=(image[top_left[1]+yoff+10:bottom_right[1]+yoff-10,top_left[0]+xoff+10:bottom_right[0]+xoff-10])
       

    dim1 = int(np.ceil(np.sqrt(len(grid_im))))
    dim2 = int(np.ceil(float(len(grid_im))/dim1))
    dim = (str(dim1) + str(dim2))
    #print (len(grid_im), dim)
    if len(grid_im)>0:
        plt.figure(1)
        num = 1
        for (key) in grid_im:
            #print key, np.shape(frame[key])
            plt.subplot(int(str(dim)+str(num))), plt.imshow(grid_im[key])
            plt.title(str(key)), plt.xticks([]), plt.yticks([])
            num = num + 1

    #plt.figure(1)
    #plt.imshow(image)
    #plt.show(1)

    return grid_im, top_left, bottom_right

def read_grid(empty_grid_im,grid_image,grid):
    grid_state = dict(grid)
    for i in range(0,9):
        print "square: ",i
        #print "new mean: ",np.mean(grid_image[i])
        #print "old mean: ",np.mean(empty_grid_im[i])+1
        print -np.mean(grid_image[i])+np.mean(empty_grid_im[i])
        if grid_state[i] == 0 and np.mean(grid_image[i])<np.mean(empty_grid_im[i])-8: 
            grid_state[i]=2
    return grid_state

def z_cal(c,ser_ee,ser_vac):
    current_Pose, current_Grip = get_position(c,ser_ee,ser_vac,CMD=1)
    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":30,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = safe_ur_move(c, ser_ee, ser_vac, Pose=demand_Pose, CMD=4)

    current_Pose, current_Grip = get_position(c,ser_ee,ser_vac,CMD=1)
    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":0,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    object_height = 1000.0*float(safe_ur_move_only(c, ser_ee, ser_vac, Pose=demand_Pose, CMD=5))
    
    msg = safe_ur_move(c,ser_ee,ser_vac,Pose=dict(naughts_crosses_cam_joints),CMD=2)

    return object_height+6

def nc_robot_move(c,ser_ee,ser_vac,square,z):
    #pick square
    msg = socket_send(c,sCMD=102)
    X_CENTRE = 50.0
    Y_CENTRE = -405.0
    GRID_WIDTH = 50
    GRID_HEIGHT = 50

    x = X_CENTRE+(1-square/3)*GRID_HEIGHT
    y = Y_CENTRE+(1-square%3)*GRID_WIDTH
    
    # Cartesian rotation matrices to match grabbing_joints rotation
    x_rot = np.matrix([[ 1.0, 0.0, 0.0],
             [ 0.0, math.cos(math.pi-0.001), -math.sin(math.pi-0.001)],
             [ 0.0, math.sin(math.pi-0.001), math.cos(math.pi-0.001)]]) # x_rot[rows][columns]
    y_rot = np.matrix([[ math.cos(0.0), 0.0, -math.sin(0.0)],
             [ 0.0, 1.0, 0.0],
             [ math.sin(0.0), 0.0, math.cos(0.0)]]) # y_rot[rows][columns]
    z_rot = np.matrix([[ math.cos(math.pi/4), -math.sin(math.pi/4), 0.0],
             [ math.sin(math.pi/4), math.cos(math.pi/4), 0.0],
             [ 0.0, 0.0, 1.0]]) # z_rot[rows][columns]

    # Create rotation matrix for current position
    R=z_rot*y_rot*x_rot

    # Cartesian to axis-angle
    theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1.0)/2)
    multi = 1 / (2 * math.sin(theta))
    rx = multi * (R[2, 1] - R[1, 2]) * theta * 180/math.pi
    ry = multi * (R[0, 2] - R[2, 0]) * theta * 180/math.pi
    rz = multi * (R[1, 0] - R[0, 1]) * theta * 180/math.pi
    #print rx, ry, rz
    #inp = raw_input("Continue?")

    # Move to drawing waypoint
    msg = safe_ur_move_only(c,ser_ee,ser_vac,Pose=dict(naughts_crosses_cam_joints),CMD=2)

    current_Pose, current_Grip = get_position(c,ser_ee,ser_vac,CMD=1)
    demand_Pose = {"x":x,"y":y,"z":z,"rx":rx,"ry":ry,"rz":rz}
    msg = safe_ur_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Speed=1.5,CMD=8)

    # Axis rotation matricies for grasping position, rotate around x-axis by aoa, then z-axis by ori
    #x_rot = np.matrix([[ 1.0, 0.0, 0.0],
    #         [ 0.0, math.cos(angle_of_attack), -math.sin(angle_of_attack)],
    #         [ 0.0, math.sin(angle_of_attack), math.cos(angle_of_attack)]]) # x_rot[rows][columns]
    '''for i in range(0,2):
        z_rot = np.matrix([[ math.cos(math.pi/2), -math.sin(math.pi/2), 0.0],
                 [ math.sin(math.pi/2), math.cos(math.pi/2), 0.0],
                 [ 0.0, 0.0, 1.0]]) # z_rot[rows][columns]
        # Cartesian rotation matrix of desired orientation
        R=z_rot*R
        # Cartesian to axis-angle
        theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1.0)/2)
        multi = 1 / (2 * math.sin(theta))
        rx = multi * (R[2, 1] - R[1, 2]) * theta * 180/math.pi
        ry = multi * (R[0, 2] - R[2, 0]) * theta * 180/math.pi
        rz = multi * (R[1, 0] - R[0, 1]) * theta * 180/math.pi
        #print rx, ry, rz
        #inp = raw_input("Continue?")
        # Rotate around tool centre point defined by tcp_2
        demand_Pose = {"x":x,"y":y,"z":z,"rx":rx,"ry":ry,"rz":rz}
        msg = safe_ur_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),CMD=8)
    '''
    for i in range(0,4):
        z_rot = np.matrix([[ math.cos(-math.pi/2), -math.sin(-math.pi/2), 0.0],
                 [ math.sin(-math.pi/2), math.cos(-math.pi/2), 0.0],
                 [ 0.0, 0.0, 1.0]]) # z_rot[rows][columns]

        # Cartesian rotation matrix of desired orientation
        R=z_rot*R

        # Cartesian to axis-angle
        theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1.0)/2)
        multi = 1 / (2 * math.sin(theta))
        rx = multi * (R[2, 1] - R[1, 2]) * theta * 180/math.pi
        ry = multi * (R[0, 2] - R[2, 0]) * theta * 180/math.pi
        rz = multi * (R[1, 0] - R[0, 1]) * theta * 180/math.pi
        #print rx, ry, rz
        #inp = raw_input("Continue?")

        # Rotate around tool centre point defined by tcp_2
        demand_Pose = {"x":x,"y":y,"z":z,"rx":rx,"ry":ry,"rz":rz}
        msg = safe_ur_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Speed=1.5,CMD=8)

    msg = socket_send(c,sCMD=100)
    msg = safe_ur_move(c,ser_ee,ser_vac,Pose=dict(naughts_crosses_cam_joints),CMD=2)

def nc_pick_move(grid_state):
    new_grid = dict(grid_state)
    remaining_sqs = []
    win=10
    nlose = 10
    for i in range(0,9):
        if grid_state[i]==0:
            remaining_sqs.append(i)
 
    for i in range(0,len(remaining_sqs)):
        if remaining_sqs[i]==0:
            if (grid_state[1]==1 and grid_state[2]==1) or (grid_state[3]==1 and grid_state[6]==1) or (grid_state[4]==1 and grid_state[8]==1): win = 0
            if (grid_state[1]==2 and grid_state[2]==2) or (grid_state[3]==2 and grid_state[6]==2) or (grid_state[4]==2 and grid_state[8]==2): nlose = 0
        if remaining_sqs[i]==1:
            if (grid_state[0]==1 and grid_state[2]==1) or (grid_state[4]==1 and grid_state[7]==1): win = 1
            if (grid_state[0]==2 and grid_state[2]==2) or (grid_state[4]==2 and grid_state[7]==2): nlose = 1
        if remaining_sqs[i]==2:
            if (grid_state[0]==1 and grid_state[1]==1) or (grid_state[5]==1 and grid_state[8]==1) or (grid_state[4]==1 and grid_state[6]==1): win = 2
            if (grid_state[0]==2 and grid_state[1]==2) or (grid_state[5]==2 and grid_state[8]==2) or (grid_state[4]==2 and grid_state[6]==2): nlose = 2
        if remaining_sqs[i]==3:
            if (grid_state[0]==1 and grid_state[6]==1) or (grid_state[4]==1 and grid_state[5]==1): win = 3
            if (grid_state[0]==2 and grid_state[6]==2) or (grid_state[4]==2 and grid_state[5]==2): nlose = 3
        if remaining_sqs[i]==4:
            if (grid_state[1]==1 and grid_state[7]==1) or (grid_state[3]==1 and grid_state[5]==1) or (grid_state[0]==1 and grid_state[8]==1) or (grid_state[2]==1 and grid_state[6]==1): win = 4
            if (grid_state[1]==2 and grid_state[7]==2) or (grid_state[3]==2 and grid_state[5]==2) or (grid_state[0]==2 and grid_state[0]==2) or (grid_state[2]==2 and grid_state[6]==2): nlose = 4
        if remaining_sqs[i]==5:
            if (grid_state[2]==1 and grid_state[8]==1) or (grid_state[3]==1 and grid_state[4]==1): win = 5
            if (grid_state[2]==2 and grid_state[8]==2) or (grid_state[3]==2 and grid_state[4]==2): nlose = 5
        if remaining_sqs[i]==6:
            if (grid_state[0]==1 and grid_state[3]==1) or (grid_state[7]==1 and grid_state[8]==1) or (grid_state[4]==1 and grid_state[2]==1): win = 6
            if (grid_state[0]==2 and grid_state[3]==2) or (grid_state[7]==2 and grid_state[8]==2) or (grid_state[4]==2 and grid_state[2]==2): nlose = 6
        if remaining_sqs[i]==7:
            if (grid_state[6]==1 and grid_state[8]==1) or (grid_state[1]==1 and grid_state[4]==1): win = 7
            if (grid_state[6]==2 and grid_state[8]==2) or (grid_state[1]==2 and grid_state[4]==2): nlose = 7
        if remaining_sqs[i]==8:
            if (grid_state[2]==1 and grid_state[5]==1) or (grid_state[6]==1 and grid_state[7]==1) or (grid_state[0]==1 and grid_state[4]==1): win = 8
            if (grid_state[2]==2 and grid_state[5]==2) or (grid_state[6]==2 and grid_state[7]==2) or (grid_state[0]==2 and grid_state[4]==2): nlose = 8
    if win<10:
        sq=win
    elif nlose<10:
        sq=nlose
    else:
        sq=remaining_sqs[int(random.uniform(0, len(remaining_sqs)-1))]

    new_grid[sq]=1
    return new_grid, sq

def nc_finish(grid_state):
    grid=dict(grid_state)
    win = 0
    for i in range(0,3):
        # Columns
        if grid[i]==1 and grid[i+3]==1 and grid[i+6]==1:
            win = 1
        if grid[i]==2 and grid[i+3]==2 and grid[i+6]==2:
            win = 2
        # Rows
        if grid[3*i]==1 and grid[3*i+1]==1 and grid[3*i+2]==1:
            win = 1
        if grid[3*i]==2 and grid[3*i+1]==2 and grid[3*i+2]==2:
            win = 2
    if (grid[0]==1 and grid[4]==1 and grid[8]==1) or (grid[6]==1 and grid[4]==1 and grid[2]==1):
            win = 1
    if (grid[0]==2 and grid[4]==2 and grid[8]==2) or (grid[6]==2 and grid[4]==2 and grid[2]==2):
            win = 2
    return win

def update_tally(c,ser_ee,ser_vac,win=0,loss=0,draw=0):
    msg = safe_ur_move(c,ser_ee,ser_vac,Pose=dict(naughts_crosses_cam_joints),CMD=2)

    msg = safe_ur_move(c,ser_ee,ser_vac,Pose=dict(tally_home_joints),CMD=2)
    dx=0
    dz=0
    board_offset=-160
    if win!=0:
        dz=(win-1)/5
        dx=win%5            
    elif loss!=0:
        dz=(loss-1)/5
        dx=loss%5
        board_offset=100
    elif draw!=0:
        dz=(draw-1)/5
        dx=draw%5
        board_offset=-30
    current_Pose, current_Grip = get_position(c,ser_ee,ser_vac,CMD=1)
    demand_Pose = {"x":current_Pose[0]+board_offset+15*dx,"y":current_Pose[1],"z":current_Pose[2]-60*dz,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = safe_ur_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),CMD=4)

    demand_Pose["y"]=current_Pose[1]+20
    msg = safe_ur_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),CMD=4)

    if dx==0:
        demand_Pose["x"]=current_Pose[0]+board_offset+15*5
    demand_Pose["z"]=current_Pose[2]-60*dz-40
    msg = safe_ur_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),CMD=8)

    demand_Pose["y"]=current_Pose[1]
    msg = safe_ur_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),CMD=4)

    msg = safe_ur_move(c,ser_ee,ser_vac,Pose=dict(tally_home_joints),CMD=2)

    msg = safe_ur_move(c,ser_ee,ser_vac,Pose=dict(naughts_crosses_cam_joints),CMD=2)

def clean_board(c,ser_ee,ser_vac):
    msg = safe_ur_move(c,ser_ee,ser_vac,Pose=dict(naughts_crosses_cam_joints),CMD=2)

    current_Pose, current_Grip = get_position(c,ser_ee,ser_vac,CMD=1)
    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":30,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = safe_ur_move(c, ser_ee, ser_vac, Pose=demand_Pose, CMD=4)

    current_Pose, current_Grip = get_position(c,ser_ee,ser_vac,CMD=1)
    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":0,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    object_height = 1000.0*float(safe_ur_move(c, ser_ee, ser_vac, Pose=demand_Pose, CMD=5))

    current_Pose, current_Grip = get_position(c,ser_ee,ser_vac,CMD=1)
    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    demand_Grip = {"act": 30, "servo": 80, "tilt": 0, "vac": 'r'}
    msg = safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)

    demand_Grip["servo"]=0
    msg = safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)
    
    # Adjust actuator position
    demand_Grip["act"]=40
    msg = safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)
    '''
    # Open grabber servo
    demand_Grip["servo"]=80
    msg = safe_ur_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)
    
    # Close grabber servo
    demand_Grip["servo"]=0
    msg = safe_ur_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)
    time.sleep(0.2)
    '''
    current_Pose, current_Grip = get_position(c,ser_ee,ser_vac,CMD=1)
    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":40,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = safe_ur_move(c, ser_ee, ser_vac, Pose=dict(demand_Pose), CMD=4)

    demand_Grip["act"]=40
    msg = safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)

    # Rotate around y axis by orientation
    # Convert to radians
    angle_of_attack=89.9*math.pi/180.0
    orientation=15.0*math.pi/180.0
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

    # Create rotation matrix for neutral position
    R=z_rot*y_rot*x_rot

    # Axis rotation matricies for pointing down then rotate around y-axis, rotate around x-axis by 89.9, then y-axis by ori
    x_rot = np.matrix([[ 1.0, 0.0, 0.0],
                       [ 0.0, math.cos(angle_of_attack), -math.sin(angle_of_attack)],
                       [ 0.0, math.sin(angle_of_attack), math.cos(angle_of_attack)]]) # x_rot[rows][columns]
    y_rot = np.matrix([[ math.cos(orientation), 0.0, -math.sin(orientation)],
                       [ 0.0, 1.0, 0.0],
                       [ math.sin(orientation), 0.0, math.cos(orientation)]]) # y_rot[rows][columns]

    # Cartesian rotation matrix of desired orientation
    R=y_rot*x_rot*R

    # Cartesian to axis-angle
    theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1.0)/2)
    multi = 1 / (2 * math.sin(theta))
    rx = multi * (R[2, 1] - R[1, 2]) * theta * 180/math.pi
    ry = multi * (R[0, 2] - R[2, 0]) * theta * 180/math.pi
    rz = multi * (R[1, 0] - R[0, 1]) * theta * 180/math.pi
    print rx, ry, rz

    # Rotate around tool centre point defined by tcp_2
    socket_send(c,sCMD=101)

    current_Pose, current_Grip = get_position(c,ser_ee,ser_vac,CMD=1)
    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":rx,"ry":ry,"rz":rz}
    msg = safe_ur_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),CMD=8)

    socket_send(c,sCMD=102)

    # Clean board
    current_Pose, current_Grip = get_position(c,ser_ee,ser_vac,CMD=1)
    demand_Pose = {"x":current_Pose[0]+100,"y":current_Pose[1]-50,"z":50,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = safe_ur_move(c, ser_ee, ser_vac, Pose=dict(demand_Pose), CMD=4)

    demand_Pose["z"]=object_height+10
    msg = safe_ur_move(c, ser_ee, ser_vac, Pose=dict(demand_Pose), CMD=4)

    demand_Pose["x"]=current_Pose[0]+250
    msg = safe_ur_move(c, ser_ee, ser_vac, Pose=dict(demand_Pose), CMD=8)

    demand_Pose["x"]=current_Pose[0]+100
    msg = safe_ur_move(c, ser_ee, ser_vac, Pose=dict(demand_Pose), CMD=8)

    demand_Pose["y"]=current_Pose[1]+50
    msg = safe_ur_move(c, ser_ee, ser_vac, Pose=dict(demand_Pose), CMD=8)

    demand_Pose["x"]=current_Pose[0]+250
    msg = safe_ur_move(c, ser_ee, ser_vac, Pose=dict(demand_Pose), CMD=8)

    demand_Pose["x"]=current_Pose[0]+100
    msg = safe_ur_move(c, ser_ee, ser_vac, Pose=dict(demand_Pose), CMD=8)

    demand_Pose["z"]=50
    msg = safe_ur_move(c, ser_ee, ser_vac, Pose=dict(demand_Pose), CMD=4)
    
    # Return erasor
    msg = safe_ur_move(c,ser_ee,ser_vac,Pose=dict(naughts_crosses_cam_joints),CMD=2)

    current_Pose, current_Grip = get_position(c,ser_ee,ser_vac,CMD=1)
    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":30,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = safe_ur_move(c, ser_ee, ser_vac, Pose=demand_Pose, CMD=4)

    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":object_height+2,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = safe_ur_move(c, ser_ee, ser_vac, Pose=demand_Pose, CMD=8)

    demand_Grip["act"]=30
    msg = safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)

    demand_Grip["act"]=30
    msg = safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)

    # Open grabber servo
    demand_Grip["servo"]=80
    msg = safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)
    time.sleep(0.2)

    # Adjust actuator position
    demand_Grip["act"]=80
    msg = safe_move(c,ser_ee,ser_vac,Pose=dict(naughts_crosses_cam_joints),Grip=demand_Grip,CMD=2)
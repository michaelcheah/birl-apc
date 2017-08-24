#!/usr/bin/env python
# Grasping strategy for each classified object
import socket
import serial
import time
import math
import numpy as np
import copy

import interface_cmds as ic
import ur_waypoints as uw
import led_functions as lf

# Vacuumable object stowing strategy
# Depends on: x, y of object on table and desired shelf to deposit object
# Returns string
def vac_stow(c,ser_ee,ser_vac,ser_led,x,y,shelf,z=110,yoff=0):
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

    # Move to above the object
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":x,"y":y,"z":110,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)

    demand_Pose["z"]=z
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)

    clr = copy.deepcopy(uw.empty_led)
    clr[4]=[255,0,0]
    clr[5]=[255,0,0]
    lf.illuminate_cluster(ser_led,2,colour=clr)

    # Move down until oject is reached
    demand_Pose = {"x":x,"y":y,"z":0,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    print "sending force_move................................................"
    object_height = 1000.0*float(ic.safe_ur_move(c,Pose=dict(demand_Pose),Speed=0.05,CMD=5))-22.0
    print "object_height: ", object_height

    time.sleep(0.5)
    # Turn on vacuum at current position
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    demand_Grip["vac"]="g"
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)

    time.sleep(2)
    # Move to above object
    demand_Pose = {"x":x,"y":y,"z":100+object_height,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = ic.safe_ur_move(c,Pose=dict(demand_Pose),Speed=0.25,CMD=4)

    # Rotate base to shelves
    current_Joints = ic.get_ur_position(c,3)
    demand_Joints = {"x":90,"y":current_Joints[1],"z":current_Joints[2],"rx":current_Joints[3],"ry":current_Joints[4],"rz":current_Joints[5]}
    msg = ic.safe_ur_move(c,Pose=dict(demand_Joints),CMD=2)

    # Reset tool to tcp_1
    ic.socket_send(c,sCMD=100)
    
    # Move closer to shelves
    msg = ic.safe_ur_move(c,Pose=dict(uw.shelf_joints_waypoint),CMD=2)

    # Move to specific shelf
    msg = ic.safe_ur_move(c,Pose=dict(uw.shelf_joints[shelf]),CMD=2)

    # Set depth on shelf
    if object_height < 57:
        shelf_depth = 816
    elif object_height < 100:
        shelf_depth = 788
    else:
        shelf_depth = 650
        print "Object too tall"

    # Raise end to object_height above the shelf
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1]+yoff,"z":current_Pose[2]+object_height,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = ic.safe_ur_move(c,Pose=dict(demand_Pose),CMD=4)

    # Move to back of shelf
    demand_Pose = {"x":shelf_depth,"y":current_Pose[1]+yoff,"z":current_Pose[2]+object_height,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = ic.safe_ur_move(c,Pose=dict(demand_Pose),CMD=8)

    time.sleep(0.5)

    # Release vacuum at current position
    demand_Grip["vac"]="r"
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)

    clr = copy.deepcopy(uw.empty_led)
    lf.illuminate_cluster(ser_led,2,colour=clr)

    time.sleep(1.2)

    pick_Pose = ic.get_ur_position(c,1)
    x = pick_Pose[0]
    y = pick_Pose[1]
    z = pick_Pose[2]

    # Exit shelf
    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1]+yoff,"z":current_Pose[2]+object_height,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = ic.safe_ur_move(c,Pose=dict(demand_Pose),CMD=4)

    # Return home
    msg = ic.safe_ur_move(c,Pose=dict(uw.shelf_joints[shelf]),CMD=2)

    # Return home
    #msg = ic.safe_ur_move(c,Pose=dict(uw.shelf_joints_waypoint),CMD=2)

    # Return home
    msg = ic.safe_ur_move(c,Pose=dict(uw.grab_home_joints),CMD=2)
    print "object_height: ", object_height
    return "Shelf "+ str(shelf) + " completed",x,y,z

# Flat object picking strategy
# Depends on: y,z of object on shelf and nth object for safe placement
# Returns string
def vac_pick(c,ser_ee,ser_vac,ser_led,y,z,n,x=816.3):
    # Determine closest shelf
    shelf = 0
    if z < 247:
        z = 151.3
        z = 148.0
        if y < -215:
            shelf = 4
        else:
            shelf = 3
    else:
        z = 359.7
        z = 356.0
        if y < -445:
            shelf = 2
        elif y < -215:
            shelf = 1
        else:
            shelf = 0

    # Ensure co-ordinates are within shelf limits
    if shelf == 0:
        if y > -30.0: y = -30.0
        if y < -172.0: y = -172.0
    if shelf == 1:
        if y > -265.0: y = -265.0
        if y < -395.0: y = -395.0
    if shelf == 2:
        if y > -490.0: y = -490.0
        if y < -629.0: y = -629.0
    if shelf == 3:
        if y > -30.0: y = -30.0
        if y < -172.0: y = -172.0
    if shelf == 4:
        if y > -265.0: y = -265.0
        if y < -629.0: y = -629.0
    print "y: ", y
    print "z: ", z

    # Home
    demand_Pose = dict(uw.grab_home)
    demand_Grip = dict(uw.end_effector_home)
    #msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.shelf_home_joints),Grip=demand_Grip,CMD=2)

    # Move to shelves
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.shelf_joints_waypoint),Grip=demand_Grip,CMD=2)

    # Move to shelf
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.shelf_joints[shelf]),Grip=demand_Grip,CMD=2)

    # Align with object
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0],"y":y,"z":z,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    object_pos = dict(demand_Pose)
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)

    # Move above object
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":x,"y":y,"z":z,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=8)

    clr = copy.deepcopy(uw.empty_led)
    clr[4]=[255,0,0]
    clr[5]=[255,0,0]
    lf.illuminate_cluster(ser_led,2,colour=clr)

    # Move down until object is reached
    demand_Pose = {"x":x,"y":y,"z":z-75,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    print "sending force_move................................................"
    object_height = 1000.0*float(ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Speed=0.05,Grip=demand_Grip,CMD=5))
    print "object_height: ", object_height

    # Grab at current position
    demand_Grip["vac"]="g"
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = ic.safe_move(c, ser_ee, ser_vac, Pose=demand_Pose, Grip=demand_Grip, CMD=4)

    time.sleep(2)
    # Lift object
    demand_Pose["z"]=z
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Speed=0.25,Grip=demand_Grip,CMD=4)

    # Exit shelf
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(object_pos),Speed=0.75,Grip=demand_Grip,CMD=4)

    # Move away from shelf
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.shelf_joints_waypoint),Grip=demand_Grip,CMD=2)

    # Rotate base
    current_Joints = ic.get_ur_position(c,3) 
    demand_Joints = {"x":current_Joints[0]-90,"y":current_Joints[1],"z":current_Joints[2],"rx":current_Joints[3],"ry":current_Joints[4],"rz":current_Joints[5]}
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Joints),Grip=demand_Grip,CMD=2)

    # Move to new position
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":-150*(4-n),"y":current_Pose[1],"z":current_Pose[2],"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)

    # Release object
    demand_Grip["vac"]="r"
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)

    clr = copy.deepcopy(uw.empty_led)
    lf.illuminate_cluster(ser_led,2,colour=clr)

    time.sleep(1.2)
    # Return home
    #msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.shelf_home_joints),Grip=demand_Grip,CMD=2)
    # Return home
    msg = ic.safe_ur_move(c,Pose=dict(uw.grab_home_joints),CMD=2)

    return "Completed pick from shelf "+ str(shelf)

# Grabable object stowing strategy
# Depends on: grasping co-ordiantes of object, and desired shelf to deposit object
def grab_stow(c,ser_ee,ser_vac,x,y,z=25,orientation=0,angle_of_attack=0,shelf=0,size=40):
    # Select tcp_2, for rotations around the grasping point
    ic.socket_send(c,sCMD=103)

    object_size = 80-int(size)
    if object_size < 5: object_size=5

    demand_Grip = dict(uw.end_effector_home)
    demand_Grip["act"]=object_size-5
    ser_ee.flush
    print "Sending actuator move"
    ser_ee.write("A" + chr(demand_Grip["act"]) + "\n")

    # Break-up rotations into max 90degrees
    thetaz = 0
    if orientation>90:
        orientation=orientation-90
        thetaz=math.pi/2
    elif orientation<-90:
        orientation=orientation+90
        thetaz=-math.pi/2

    # Avoid singularity at +/-45degrees
    if orientation==45:
        orientation = 44
    elif orientation==-45:
        orientation = -44

    # Convert to radians
    angle_of_attack=angle_of_attack*math.pi/180.0
    orientation=orientation*math.pi/180.0
    thetay=135.0*math.pi/180.0
    
    # Cartesian rotation matrices to match uw.grabbing_joints rotation
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
    msg = ic.safe_ur_move(c,Pose=dict(uw.grabbing_joints_waypoint),CMD=2)

    # Create rotation matrix for current position
    R=z_rot*y_rot*x_rot

    if thetaz!=0:
        # Axis rotation matricies for grasping position, rotate around x-axis by aoa, then z-axis by ori
        x_rot = np.matrix([[ 1.0, 0.0, 0.0],
                 [ 0.0, math.cos(angle_of_attack), -math.sin(angle_of_attack)],
                 [ 0.0, math.sin(angle_of_attack), math.cos(angle_of_attack)]]) # x_rot[rows][columns]
        z_rot = np.matrix([[ math.cos(thetaz), -math.sin(thetaz), 0.0],
                 [ math.sin(thetaz), math.cos(thetaz), 0.0],
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
        current_Pose = ic.get_ur_position(c,1)
        demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":rx,"ry":ry,"rz":rz}
        msg = ic.safe_ur_move(c,Pose=dict(demand_Pose),CMD=8)

        # Axis rotation matricies for grasping position, rotate around x-axis by aoa, then z-axis by ori
        z_rot = np.matrix([[ math.cos(orientation), -math.sin(orientation), 0.0],
                 [ math.sin(orientation), math.cos(orientation), 0.0],
                 [ 0.0, 0.0, 1.0]]) # z_rot[rows][columns]

        # Cartesian rotation matrix of desired orientation
        R=z_rot*R

        # Cartesian to axis-angle
        theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1.0)/2)
        multi = 1 / (2 * math.sin(theta))
        rx = multi * (R[2, 1] - R[1, 2]) * theta * 180/math.pi
        ry = multi * (R[0, 2] - R[2, 0]) * theta * 180/math.pi
        rz = multi * (R[1, 0] - R[0, 1]) * theta * 180/math.pi
        print rx, ry, rz

        # Rotate around tool centre point defined by tcp_2
        current_Pose = ic.get_ur_position(c,1)
        demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":rx,"ry":ry,"rz":rz}
        msg = ic.safe_ur_move(c,Pose=dict(demand_Pose),CMD=8)
    else:
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
        current_Pose = ic.get_ur_position(c,1)
        demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":rx,"ry":ry,"rz":rz}
        msg = ic.safe_ur_move(c,Pose=dict(demand_Pose),CMD=8)

    while True:
        ipt = ser_ee.readline()
        print ipt
        if ipt == "done\r\n":
            break
    timeout = ser_ee.readline()
    print "timeout: ", timeout
    ser_ee.flush

    # Move to above object
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":x,"y":y,"z":z+100,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4,Speed=0.5)

    # Move closer
    #demand_Pose = {"x":x,"y":y,"z":z+50,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    #msg = ic.safe_ur_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4,Speed=0.5)

    # Linear move to grasping position
    demand_Pose = {"x":x,"y":y,"z":z,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=8,Speed=0.4)

    # Grab with reduced chance of collision
    # Partially close grabber servo
    current_Pose, current_Grip = ic.get_position(c,ser_ee,ser_vac,CMD=1)
    demand_Grip = {"act": object_size, "servo": 20, "tilt": current_Grip[2]&0x20, "vac": current_Grip[4]}
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)

    # Adjust actuator position
    demand_Grip["act"]=object_size+2
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)

    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)

    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)

    # Open grabber servo
    #demand_Grip["servo"]=80
    #msg = ic.safe_ur_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)

    # Close grabber servo
    demand_Grip["servo"]=0
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)

    time.sleep(0.5)
    # Lift object
    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":z+100,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = ic.safe_ur_move(c,Pose=dict(demand_Pose),CMD=4)

    ic.socket_send(c,sCMD=101)

    # Move safely towards shelf
    msg = ic.safe_ur_move(c,Pose=dict(uw.grabbing_joints_waypoint),CMD=2)

    # Move safely towards shelf
    current_Joints = ic.get_ur_position(c,3)
    demand_Joints = {"x":90,"y":current_Joints[1],"z":current_Joints[2],"rx":current_Joints[3],"ry":current_Joints[4],"rz":current_Joints[5]}
    msg = ic.safe_ur_move(c,Pose=dict(demand_Joints), CMD=2)

    # Move safely towards shelf
    demand_Joints = dict(uw.shelf_joints_waypoint)
    demand_Joints["rz"] = demand_Joints["rz"]-90
    msg = ic.safe_ur_move(c,Pose=dict(demand_Joints),CMD=2)

    # Move safely towards shelf
    demand_Joints = dict(uw.shelf_joints[shelf])
    demand_Joints["rz"] = demand_Joints["rz"]-90
    msg = ic.safe_ur_move(c,Pose=dict(demand_Joints),CMD=2)

    # Define position on shelf
    object_height = 50
    shelf_depth = 788
    yoff = 35
    
    # Align object with shelf
    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1]+yoff,"z":current_Pose[2]+object_height,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = ic.safe_ur_move(c,Pose=dict(demand_Pose),CMD=4)

    # Move into shelf
    demand_Pose["x"]=shelf_depth
    msg = ic.safe_ur_move(c,Pose=dict(demand_Pose),CMD=4)

    # Move into shelf
    demand_Pose["z"]=current_Pose[2]+object_height-10
    msg = ic.safe_ur_move(c,Pose=dict(demand_Pose),CMD=4)

    # Release object
    demand_Grip["servo"]=80
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,CMD=4)

    time.sleep(0.2)
    # Move back
    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1]+yoff,"z":current_Pose[2]+object_height,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = ic.safe_ur_move(c,Pose=dict(demand_Pose),CMD=4)

    # Return home
    msg = ic.safe_ur_move(c,Pose=dict(uw.shelf_joints[shelf]),CMD=2)

    # Return home
    msg = ic.safe_ur_move(c,Pose=dict(uw.shelf_joints_waypoint),CMD=2)

    # Return home
    msg = ic.safe_ur_move(c,Pose=dict(uw.grab_home_joints),CMD=2)

    # Reset tool to tcp_1
    ic.socket_send(c,sCMD=100)
    
    return "Shelf "+ str(shelf) + " completed"

def grab_pick(c,ser_ee,ser_vac,y,z=12,orientation=0,object_height=30,size=70):
    # curved object picking strategy
    #ic.socket_send(c,sCMD=101)

    shelf = 0
    if z < 247:
        z = 151.3
        if y < -215:
            shelf = 4
        else:
            shelf = 3
    else:
        z = 359.7
        if y < -445:
            shelf = 2
        elif y < -215:
            shelf = 1
        else:
            shelf = 0

    y = y + orientation/2

    if shelf == 0:
        if y > -30.0: y = -30.0
        if y < -172.0: y = -172.0
    if shelf == 1:
        if y > -265.0: y = -265.0
        if y < -395.0: y = -395.0
    if shelf == 2:
        if y > -490.0: y = -490.0
        if y < -629.0: y = -629.0
    if shelf == 3:
        if y > -30.0: y = -30.0
        if y < -172.0: y = -172.0
    if shelf == 4:
        if y > -265.0: y = -265.0
        if y < -629.0: y = -629.0
    print "y: ", y
    print "z: ", z

    demand_Grip = dict(uw.end_effector_home)
    demand_Grip["act"] = int(80-0.8*object_height)
    ser_ee.flush
    print "Sending actuator move"
    ser_ee.write("A" + chr(demand_Grip["act"]) + "\n")

    #demand_Grip["servo"] = 0
    msg = ic.safe_ur_move(c,Pose=dict(uw.shelf_grab_joints[shelf]),CMD=2)

    current_Pose = ic.get_ur_position(c,1)
    demand_Pose = {"x":current_Pose[0],"y":y,"z":current_Pose[2]+object_height-37,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = ic.safe_ur_move(c,Pose=dict(demand_Pose),CMD=4)

    while True:
        ipt = ser_ee.readline()
        print ipt
        if ipt == "done\r\n":
            break
    timeout = ser_ee.readline()
    print "timeout: ", timeout
    ser_ee.flush

    yoffset=20.0*(y+73.0)/480.0
    demand_Pose["x"]=current_Pose[0]+160.0-yoffset
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,Speed=0.25,CMD=8)

    demand_Pose["z"]=current_Pose[2]+object_height-52
    demand_Grip["servo"]=object_height+5
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,Speed=0.2,CMD=4)

    demand_Pose["x"]=current_Pose[0]+90-yoffset
    demand_Grip["servo"]=0
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,Speed=0.25,CMD=8)

    demand_Pose["x"]=current_Pose[0]-30-yoffset
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(demand_Pose),Grip=demand_Grip,Speed=0.25,CMD=8)

    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.shelf_joints_waypoint),Grip=demand_Grip,CMD=2)

    time.sleep(2)
    demand_Grip["servo"]=80
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.shelf_joints_waypoint),Grip=demand_Grip,CMD=2)

    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.grab_home_joints),Grip=demand_Grip,CMD=2)
    return "Completed pick from shelf "+ str(shelf)

def combined_stow(c,ser_ee,ser_vac,x,y,z=12,orientation=0,angle_of_attack=0,shelf=0,size=70):

    return "Shelf "+ str(shelf) + " completed"

def combined_pick(c,ser_ee,ser_vac,x,y,z=12,orientation=0,angle_of_attack=0,shelf=0,size=70):

    return "Completed pick from shelf "+ str(shelf)

'''
def banana(c,ser_ee,ser_vac,x,y,rz):
    # banana picking strategy
    demand_Pose = dict(uw.grab_home)
    demand_Grip = dict(uw.end_effector_home)
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.grab_home_joints),Grip=demand_Grip,CMD=2)
    demand_Grip["act"]=40
    current_Joints, current_Grip = ic.get_position(c,ser_ee,ser_vac,CMD=3)
    demand_Joints = {"x":current_Joints[0],"y":current_Joints[1],"z":current_Joints[2],"rx":current_Joints[3],"ry":current_Joints[4],"rz":current_Joints[5]+rz}
    ic.safe_move(c, ser_ee, ser_vac, Pose=demand_Joints, Grip=demand_Grip, CMD=2)
    time.sleep(1)
    current_Pose, current_Grip = ic.get_position(c,ser_ee,ser_vac,CMD=1)
    demand_Pose = {"x":x,"y":current_Pose[1],"z":current_Pose[2],"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    ic.safe_move(c, ser_ee, ser_vac, Pose=demand_Pose, Grip=demand_Grip)
    demand_Pose["y"]=y
    ic.safe_move(c, ser_ee, ser_vac, Pose=demand_Pose, Grip=demand_Grip)
    demand_Pose["z"]=8
    ic.safe_move(c, ser_ee, ser_vac, Pose=demand_Pose, Grip=demand_Grip)
    demand_Grip["servo"]=0
    ic.safe_move(c, ser_ee, ser_vac, Pose=demand_Pose, Grip=demand_Grip)
    time.sleep(0.5)
    demand_Grip["vac"]="g"
    ic.safe_move(c, ser_ee, ser_vac, Pose=demand_Pose, Grip=demand_Grip)
    time.sleep(2)
    demand_Pose["z"]=100
    ic.safe_move(c, ser_ee, ser_vac, Pose=demand_Pose, Grip=demand_Grip)
    ic.safe_move(c,ser_ee,ser_vac)
    demand_Grip["servo"]=80
    demand_Grip["vac"]="r"
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.grab_home_joints),Grip=demand_Grip,CMD=2)
    return
'''

'''
def fork(c, ser_ee, ser_vac,x,y,rz):
    # fork picking strategy
    demand_Pose = dict(uw.grab_home)
    demand_Grip = dict(uw.end_effector_home)
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.grab_home_joints),Grip=demand_Grip,CMD=2)

    current_Joints, current_Grip = ic.get_position(c,ser_ee,ser_vac,CMD=3)
    demand_Joints = {"x":current_Joints[0]-90,"y":current_Joints[1],"z":current_Joints[2],"rx":current_Joints[3],"ry":current_Joints[4]+60,"rz":current_Joints[5]-45}
    msg = ic.safe_move(c, ser_ee, ser_vac, Pose=demand_Joints, Grip=demand_Grip, CMD=2)

    current_Pose, current_Grip = ic.get_position(c,ser_ee,ser_vac,CMD=1)
    demand_Pose = {"x":x,"y":y,"z":100+rz,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = ic.safe_move(c, ser_ee, ser_vac, Pose=demand_Pose, Grip=demand_Grip, CMD=4)

    demand_Pose = {"x":x,"y":y,"z":25+rz,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    print "sending force_move................................................"
    msg = ic.safe_move(c, ser_ee, ser_vac, Pose=demand_Pose, Speed=0.05, Grip=demand_Grip, CMD=5)

    current_Pose, current_Grip = ic.get_position(c,ser_ee,ser_vac,CMD=1)
    demand_Pose = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    demand_Grip["vac"]="g"
    msg = ic.safe_move(c, ser_ee, ser_vac, Pose=demand_Pose, Grip=demand_Grip, CMD=4)

    time.sleep(2)
    demand_Pose = {"x":x,"y":y,"z":100+rz,"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
    msg = ic.safe_move(c, ser_ee, ser_vac, Pose=demand_Pose, Speed=0.25, Grip=demand_Grip, CMD=4)

    current_Joints, current_Grip = ic.get_position(c,ser_ee,ser_vac,CMD=3)
    demand_Joints = {"x":90,"y":current_Joints[1],"z":current_Joints[2],"rx":current_Joints[3],"ry":current_Joints[4],"rz":current_Joints[5]}
    msg = ic.safe_move(c, ser_ee, ser_vac, Pose=demand_Joints, Grip=demand_Grip, CMD=2)

    demand_Grip["vac"]="r"
    msg = ic.safe_move(c, ser_ee, ser_vac, Pose=demand_Joints, Grip=demand_Grip, CMD=2)

    time.sleep(4)
    msg = ic.safe_move(c,ser_ee,ser_vac,Pose=dict(uw.grab_home_joints),Grip=demand_Grip,CMD=2)

    return
'''

def get_grasping_coords(p_centre,p_edge,height):
    if height>80:
        return # sideways grasping coords
    else:
        aoa = 89.9
        ori = math.atan2(p_centre[1]-p_edge[1],p_centre[0]-p_edge[0])*180.0/math.pi
        size = math.sqrt(math.pow(p_centre[0]-p_edge[0],2)+math.pow(p_centre[1]-p_edge[1],2))
        print "ori: ",ori
        ori = ori-180
        if ori<-180:
            ori=360+ori
        x = p_edge[0]
        y = p_edge[1]
        z = height
    return float(x[0]), float(y[0]), z, ori, aoa, size
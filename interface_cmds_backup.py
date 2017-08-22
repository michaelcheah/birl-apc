#!/usr/bin/env python
# Grasping strategy for each classified object
import socket
import serial
import time
import math
import copy

from ur_waypoints import grab_home, grab_home_joints, end_effector_home, shelf_home_joints, shelf_joints_waypoint, shelf_joints, shelf_pos, cam1_joints, grabbing_joints_waypoint

# Socket CMDs
# Send (Pose, Command, Variable) to the UR5 using the socket connection c
# Returns string recieved from the UR
# List of CMDs to UR5:
# 0 - moves to Pose, Variable = max joint speed, UR returns string "complted_pose_move" upon completion 
# 1 - queries UR for current Pose, Variable = n.a., UR returns string "p[current_Pose]"
# 2 - moves to Joint positions, Variable = max joint speed, UR returns string "complted_joint_move" upon completion 
# 3 - queries UR for current Joint values, Variable = n.a., UR returns string "[current_Joints]"
# 4 - safe move to Pose-if any joints change by more than the threshold angle(2rad) Pose is rejected, if using safe_ur_move, move is subsampled until not rejected, Variable = max joint speed, UR returns string "no_safe_move_found" upon rejection, "complted_pose_move" upon completion 
# 5 - moves to new z position unless z force is exceeded, Variable = max joint speed, UR returns string final z position if force exceeded, else returns "0" upon completion
# 6 - queries UR for current Force, Variable = n.a., UR returns string "p[forces]"
# 7 - queries UR for current Joint torques, Variable = n.a., UR returns string "[torques]"
# 8 - moves linearly to Pose, Variable = max joint speed, UR returns string "complted_linear_move" upon completion
#
#
# 100 - set tool centre point to tcp_1 (only z offset from end), Variable = n.a., UR returns string "tool_1_selected"
# 101 - set tool centre point to tcp_2 (centre used for vector rotations), Variable = n.a., UR returns string "tool_2_selected"
def socket_send(c, sPose=copy.deepcopy(grab_home), sSpeed = 0.75, sCMD = 0):
    sendPose = copy.deepcopy(sPose)
    msg = "Failed"
    try:
        # Send formatted CMD
        c.send("("+str(sendPose["x"])+","+str(sendPose["y"])+","+str(sendPose["z"])+","+str(sendPose["rx"])+","+str(sendPose["ry"])+","+str(sendPose["rz"])+","+str(sCMD)+","+str(sSpeed)+")");
        print "Sent ur move"
        # Wait for reply
        msg=c.recv(1024)
        print msg
        print ""
    except socket.error as socketerror:
        print ".......................Some kind of error :(......................."
    # Return reply
    return msg

# Move CMDs
# Send socket and serial CMDs
# Returns nothing
# Now redundant to safe_ur_move
def ur_move(c, ser_ee, ser_vac,Pose = copy.deepcopy(grab_home),Speed = 0.75,Grip = copy.deepcopy(end_effector_home),CMD = 0):
    # Socket CMDs
    sendPose = copy.deepcopy(Pose)
    msg = socket_send(c, sPose=sendPose, sSpeed=Speed, sCMD=CMD)

    # Serial CMDs
    ser_ee.flush
    ser_vac.flush
    print "Sending end effector move"
    # Set Actuator position, min = 0, max = 80
    ser_ee.write("A" + chr(Grip["act"]) + "\n")
    # Wait for end effector arduino to finish
    while True:
        ipt = ser_ee.readline()
        print ipt
        if ipt == "done\r\n":
            break
    ser_ee.flush
    # Set Grabber Servo position, min = 22, max = 127
    ser_ee.write("G" + chr(Grip["servo"]) + "\n")
    # Wait for end effector arduino to finish
    while True:
        ipt = ser_ee.readline()
        print ipt
        if ipt == "done\r\n":
            break
    ser_ee.flush
    # Set Tilt Servo position, min = 0, max = 1
    ser_ee.write("T" + chr(Grip["tilt"]) + "\n")
    # Wait for end effector arduino to finish
    while True:
        ipt = ser_ee.readline()
        print ipt
        if ipt == "done\r\n":
            break
    ser_ee.flush
    # Set vacuum state, release = "r", grab = "g"
    ser_vac.write("V" + Grip["vac"] + "\n")
    # Wait for vacuum arduino to finish
    while True:
        ipt = ser_vac.readline()
        print ipt
        if ipt == "done\r\n":
            break

# Safe Move CMDs
# Send socket and serial CMDs
# Returns reply from UR
def safe_ur_move(c,ser_ee,ser_vac,Pose = copy.deepcopy(grab_home),Speed = 0.75,Grip = copy.deepcopy(end_effector_home),CMD = 4):
    # Socket CMDs
    sendPose = copy.deepcopy(Pose)
    if CMD == 4:
        # Safe move
        demand_Pose = copy.deepcopy(sendPose)
        print "demand_Pose: ", demand_Pose
        msg = "no_safe_move_found"
        n = 1           # Number of steps to divide the remaining move into
        alpha = 1.0     # Interpolation factor
        # Subsample pose until steps are small enough
        # e.g. demand_Pose rejected -> move split into 2 steps -> first step accepted -> second step accepted -> returns "completed_safe_move"
        #                                                                             -> second step rejected -> remaining n steps split into n+1 steps etc...
        #                                                      -> first step rejected -> move split into 3 steps etc...
        while msg == "no_safe_move_found":
            current_Pose, current_Grip = get_position(c,ser_ee,ser_vac,CMD=1)
            print "current_pose: ", current_Pose
            Pose2 = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
            alpha = 1.0/n
            for i in range(1,n+1):
                print n
                # Interpolate from current position to demand position in n steps
                i_Pose = interpolate_pose(demand_Pose, Pose2, alpha*i)
                print i_Pose
                # Send new demand_Pose
                msg = socket_send(c, sPose=i_Pose, sSpeed=Speed, sCMD=4)
                # If rejected, increment number of steps and restart for loop
                if msg == "no_safe_move_found":
                    n = n+1
                    time.sleep(0.5)
                    break
                # If accepted, decrement number of steps and continue for loop
                else:
                    n = n-1
    else:
        # Non-safe move
        msg = socket_send(c, sPose=sendPose, sSpeed=Speed, sCMD=CMD)

    # Serial CMDs
    ser_ee.flush
    ser_vac.flush
    print "Sending end effector move"
    # Set Actuator position, min = 0, max = 80
    ser_ee.write("A" + chr(Grip["act"]) + "\n")
    # Wait for end effector arduino to finish
    while True:
        ipt = ser_ee.readline()
        print ipt
        if ipt == "done\r\n":
            break
    ser_ee.flush
    # Set Grabber Servo position, min = 22, max = 127
    ser_ee.write("G" + chr(Grip["servo"]) + "\n")
    # Wait for end effector arduino to finish
    while True:
        ipt = ser_ee.readline()
        print ipt
        if ipt == "done\r\n":
            break
    ser_ee.flush
    # Set Tilt Servo position, min = 0, max = 1
    ser_ee.write("T" + chr(Grip["tilt"]) + "\n")
    # Wait for end effector arduino to finish
    while True:
        ipt = ser_ee.readline()
        print ipt
        if ipt == "done\r\n":
            break
    ser_ee.flush
    # Set vacuum state, release = "r", grab = "g"
    ser_vac.write("V" + Grip["vac"] + "\n")
    # Wait for vacuum arduino to finish
    while True:
        ipt = ser_vac.readline()
        print ipt
        if ipt == "done\r\n":
            break

    return msg

# Test function for moving UR while waiting for Serial
# Safe Move CMDs
# Send socket CMDs without Serial CMDs
# Returns reply from UR
def safe_ur_move_only(c,ser_ee,ser_vac,Pose = copy.deepcopy(grab_home),Speed = 0.75,CMD = 4):
    # Socket CMDs
    sendPose = copy.deepcopy(Pose)
    if CMD == 4:
        # Safe move
        demand_Pose = copy.deepcopy(sendPose)
        #print "demand_Pose: ", demand_Pose
        msg = "no_safe_move_found"
        n = 1           # Number of steps to divide the remaining move into
        alpha = 1.0     # Interpolation factor
        # Subsample pose until steps are small enough
        # e.g. demand_Pose rejected -> move split into 2 steps -> first step accepted -> second step accepted -> returns "completed_safe_move"
        #                                                                             -> second step rejected -> remaining n steps split into n+1 steps etc...
        #                                                      -> first step rejected -> move split into 3 steps etc...
        while msg == "no_safe_move_found":
            current_Pose, current_Grip = get_position(c,ser_ee,ser_vac,CMD=1)
            print "current_pose: ", current_Pose
            Pose2 = {"x":current_Pose[0],"y":current_Pose[1],"z":current_Pose[2],"rx":current_Pose[3],"ry":current_Pose[4],"rz":current_Pose[5]}
            alpha = 1.0/n
            for i in range(1,n+1):
                print n
                # Interpolate from current position to demand position in n steps
                i_Pose = interpolate_pose(demand_Pose, Pose2, alpha*i)
                print i_Pose
                # Send new demand_Pose
                msg = socket_send(c, sPose=i_Pose, sSpeed=0.75, sCMD=4)
                # If rejected, increment number of steps and restart for loop
                if msg == "no_safe_move_found":
                    n = n+1
                    time.sleep(0.5)
                    break
                # If accepted, decrement number of steps and continue for loop
                else:
                    n = n-1
    else:
        # Non-safe move
        msg = socket_send(c, sPose=sendPose, sSpeed=Speed, sCMD=CMD)

    return msg

# Query CMDs
# Send socket and serial CMDs
# Returns decoded replies: [current robot position], [actuator angle, servo pos, tilt pos, switch state, vac state]
def get_position(c, ser_ee, ser_vac,Pose = copy.deepcopy(grab_home),CMD = 1):
    # Initialize variables
    sendPose = copy.deepcopy(Pose)
    msg = "0"
    ipt1a = "0"
    ipt1b = "0"
    ipt1c = "0"
    ipt2 = "0"

    # Socket CMDs
    msg = socket_send(c, sPose=sendPose, sSpeed=0, sCMD=CMD)

    # Serial CMDs
    # Query end effector arduino state
    ser_ee.write("R" + "0" + "\n")

    ipt1a = ser_ee.readline()
    ipt1b = ser_ee.readline()
    ipt1c = ser_ee.readline()
    ipt1d = ser_ee.readline()
    print ipt1a
    print ""
    print ipt1b
    print ""
    print ipt1c
    print ""
    print ipt1d
    print ""

    # Query vacuum arduino state
    ser_vac.write("R" + "0" + "\n")
    ipt2 = ser_vac.readline()
    print ipt2
    print ""

    # Decode Pose or Joints from UR
    current_position = [0,0,0,0,0,0]
    data_start = 0
    data_end = 0
    n = 0
    x = 0
    while x < len(msg):
        if msg[x]=="," or msg[x]=="]" or msg[x]=="e":
            data_end = x-1
            current_position[n] = float(msg[data_start:data_end])
            if msg[x]=="e":
                current_position[n] = current_position[n]*math.pow(10,float(msg[x+1:x+4]))
                print "e", msg[x+1:x+4]
                print "e", int(msg[x+1:x+4])
                if n < 5:
                    x = x+5
                    data_start = x
                else:
                    break
            n=n+1
        if msg[x]=="[" or msg[x]==",":
            data_start = x+1
        x = x+1

    # Convert x,y,z values to mm, convert rotations to degrees
    for x in range(0,3):
        if CMD==1 or CMD==8: current_position[x] = current_position[x]*1000.0
        if CMD==3: current_position[x] = current_position[x]*180.0/math.pi
        current_position[x+3] = current_position[x+3]*180.0/math.pi
    
    # Decode Serial Data
    ipt1a = ipt1a[0:len(ipt1a)-2]
    ipt1b = ipt1b[0:len(ipt1b)-2]
    ipt1c = ipt1c[0:len(ipt1c)-2]
    ipt1d = ipt1d[0:len(ipt1d)-2]

    return current_position, [int(ipt1a)+300, int(ipt1b), int(ipt1c), int(ipt1d), ipt2[0]] 


# Query CMDs
# Send socket CMD=6
# Unused as force now sent as a vector and decoded by get_position()
def get_force(c):
    msg = socket_send(c, sPose=copy.deepcopy(grab_home), sCMD=6)
    print "force: ", msg
    return msg

# Query CMDs
# Send socket CMD=7
# Unused as torque now sent as a vector and decoded by get_position()
def get_torque(c):
    msg = socket_send(c, sPose=copy.deepcopy(grab_home), sCMD=7)
    print "torque: ", msg
    return msg

# Finds new pose linearly interpotlated from Pose2 to Pose1
# Returns new pose
def interpolate_pose(Pose1, Pose2, alpha):
    i_Pose = copy.deepcopy(grab_home)
    i_Pose["x"] = alpha*Pose1["x"] + (1.0-alpha)*Pose2["x"]
    i_Pose["y"] = alpha*Pose1["y"] + (1.0-alpha)*Pose2["y"]
    i_Pose["z"] = alpha*Pose1["z"] + (1.0-alpha)*Pose2["z"]
    i_Pose["rx"] = alpha*Pose1["rx"] + (1.0-alpha)*Pose2["rx"]
    i_Pose["ry"] = alpha*Pose1["ry"] + (1.0-alpha)*Pose2["ry"]
    i_Pose["rz"] = alpha*Pose1["rz"] + (1.0-alpha)*Pose2["rz"]
    return i_Pose

# Alternate end effector arduino CMD
# Returns switch state(0=open, 1=closed), data(timed out if !=0)
def ee_pinch(ser_ee, act):
    ser_ee.flush
    print "Sending end effector move"
    # Set Pinch position, min = 0, max = 80
    ser_ee.write("P" + chr(act) + "\n")
    # Wait for end effector arduino to finish 
    while True:
        ipt = ser_ee.readline()
        print ipt
        if ipt == "done\r\n":
            break
    # Red additional data
    msg = ser_ee.readline()
    msg = int(msg[0:len(msg)-2])
    timeout = ser_ee.readline()
    timeout = int(timeout[0:len(timeout)-2])
    return msg, timeout
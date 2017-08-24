#!/usr/bin/env python
import serial
import copy

import interface_cmds as ic
import ur_waypoints as uw

def illuminate_cluster(ser_led,strip,colour=copy.deepcopy(uw.empty_led),set=1):
    C = strip&0x01
    B = strip&0x02
    A = strip&0x04
    print A, B, C
    if C==1:
        for i in range(0,6):
            ic.led_serial_send(ser_led,"C",i,colour[i][0],colour[i][1],colour[i][2])
    if B==2:
        for i in range(0,6):
            print "B",i,colour[i][0],colour[i][1],colour[i][2]
            ic.led_serial_send(ser_led,"B",i,colour[i][0],colour[i][1],colour[i][2])
    if A==4:
        for i in range(0,6):
            ic.led_serial_send(ser_led,"A",i,colour[i][0],colour[i][1],colour[i][2])
    if set==1:
        print "strip: ", strip
        ic.led_serial_send(ser_led,"S",strip,0,0,0)

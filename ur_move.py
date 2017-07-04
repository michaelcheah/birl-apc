#!/usr/bin/env python
# Echo client program
import socket
import time

import numpy as np
import cv2
from matplotlib import pyplot as plt
import imutils
import cv2.cv as cv
from vision import capture_pic, crop_table, identify_glass
from vision import plot_212, plot_224
from vision import pix2world_cal, pix2world

# Names of Images to be stored. Change only TEST_NUMBER for different tests
TEST_NUMBER = str(1)
IMG_FORMAT = ".jpg"
CAL_IMG_1 = "cal_image_1_" + TEST_NUMBER + IMG_FORMAT
CAL_IMG_2 = "cal_image_2_" + TEST_NUMBER + IMG_FORMAT
POSITION_IMG = "position_image_" + TEST_NUMBER + IMG_FORMAT
CROPPED_IMG = "crop_image_" + TEST_NUMBER + IMG_FORMAT

# Calibration parameters
CAL_THRESH = [10, 200]
CAL_SIZES = [90, None]
CAL_WIDTH_HEIGHT_RATIO = 1.2
CAL_FILL = 0.7

CAL_PARAMS = {'thresh': CAL_THRESH,
             'size': CAL_SIZES,
             'ratio': CAL_WIDTH_HEIGHT_RATIO,
             'fill': CAL_FILL}

# Crop parameters
CROP_THRESH = [5, 220]
CROP_SIZES = [300, None]
CROP_WIDTH_HEIGHT_RATIO = 1.1
CROP_FILL = 0.6

CROP_PARAMS = {'thresh': CROP_THRESH,
             'size': CROP_SIZES,
             'ratio': CROP_WIDTH_HEIGHT_RATIO,
             'fill': CROP_FILL}

# Colors of Bounding Box Points
COLORS = ((0, 0, 255), (240, 0, 159), (255, 0, 0), (255, 255, 0))

# Adjustment to Crop
ADJUSTMENT = 10

# Wine Glass detection parameters
GLASS_PARAM = {'thresh': [19,20],
               'radius': [10,30]}

xc1 = -234.8
xc2 = -680.0
yc1 = -178.8
yc2 = -400
home_x = -400
home_y = 120
home_z = 65.0
home_rx = 90.0
home_ry = 0.0
home_rz = 0.0

def ur_move(c,x,y,z,rx,ry,rz,ee):
	try:
		c.send("("+str(x)+","+str(y)+","+str(z)+","+str(rx)+","+str(ry)+","+str(rz)+","+str(ee)+")");
		print "Sent move"
		msg=c.recv(1024)
		print msg
		print ""
	except socket.error as socketerror:
		print "Some kind of error :("

def calibration(c):
	ur_move(c,xc1,yc1,home_z,home_rx,home_ry,home_rz,0)
	time.sleep(2)
	capture_pic(CAL_IMG_1)
	inp = raw_input("Continue? y/n: ")[0]
	if (inp != 'y'):
		return
	ur_move(c,xc2,yc2,home_z,home_rx,home_ry,home_rz,0)
	time.sleep(2)
	capture_pic(CAL_IMG_2)


def pick_wine_glass(c):
	capture_pic(POSITION_IMG)
	crop_img, crop_points = crop_table(
							POSITION_IMG, CROP_PARAMS,
							adjustment = ADJUSTMENT, colors = COLORS)
	#plt.show()
	cimg, pixel_coords = identify_glass(
							crop_img, crop_points, glass_param = GLASS_PARAM,
							adjustment = ADJUSTMENT)
	world1 = [xc1, yc1]
	world2 = [xc2, yc2]

	p2w_calval = pix2world_cal(CAL_IMG_1, CAL_IMG_2,
					world1, world2, cal_params=CAL_PARAMS, colors=COLORS)
	worldx, worldy = pix2world(pixel_coords, p2w_calval)
	x = worldx #float(raw_input("Input x coord: "))
	y = worldy #float(raw_input("Input y coord: "))
	print "Moving to:"
	print x
	print y
	y = y + 180
	ur_move(c,x,home_y,home_z+50,home_rx,home_ry,home_rz,0)
	ur_move(c,x,y+20,home_z+50,home_rx,home_ry,home_rz,0)
	ur_move(c,x,y,home_z+50,home_rx,home_ry,home_rz,1)
	ur_move(c,x,y,200,home_rx,home_ry,home_rz,1)
	ur_move(c,x,y,home_z+50,home_rx,home_ry,home_rz,0)
	ur_move(c,x,y+20,home_z+50,home_rx,home_ry,home_rz,0)
	ur_move(c,x,home_y,home_z+50,home_rx,home_ry,home_rz,0)
	ur_move(c,home_x,home_y,home_z,home_rx,home_ry,home_rz,0)

def pick_mug(c):
	x = float(raw_input("Input x coord: "))
	y = float(raw_input("Input y coord: "))
	x2 =float(raw_input("Input x2 coord: "))
	y2 =float(raw_input("Input y2 coord: "))
	print "Moving to:"
	print x
	print y
	y = y + 165
	y2 = y2 + 165
	ur_move(c,x,home_y,home_z,home_rx,home_ry,home_rz,0)
	ur_move(c,x,y+80,home_z,home_rx,home_ry,home_rz,0)
	ur_move(c,x,y,home_z,home_rx,home_ry,home_rz,0)
	time.sleep(2)
	ur_move(c,x,y,200,home_rx,home_ry,home_rz,1)
	ur_move(c,x2,y2,200,home_rx,home_ry,home_rz,1)
	ur_move(c,x2,y2,80,home_rx,home_ry,home_rz,0)
	time.sleep(2)
	ur_move(c,x2,y2+80,80,home_rx,home_ry,home_rz,0)
	ur_move(c,x2,home_y,home_z,home_rx,home_ry,home_rz,0)
	ur_move(c,home_x,home_y,home_z,home_rx,home_ry,home_rz,0)

def test_move(c):
	x = float(raw_input("Input x coord: "))
	y = float(raw_input("Input y coord: "))
	home = raw_input("Default z? y/n: ")
	if (home == 'y'):
		z = home_z
		rx = home_rx
		ry = home_ry
		rz = home_rz
	else:
		z = float(raw_input("Input z coord: "))
		rx = float(raw_input("Input x rotation: "))
		ry = float(raw_input("Input y rotation: "))
		rz = float(raw_input("Input z rotation: "))
	g = raw_input("Grab? y/n: ")
	print "Moving to: %s" %(str(x)+","+str(y)+","+str(z)+","+str(rx)+","+str(ry)+","+str(rz))
	if (g == 'y'):
		ur_move(c,x,y,z,rx,ry,rz,1)
	else:
		ur_move(c,x,y,z,rx,ry,rz,0)

def main():
	HOST = "169.254.103.235" # The remote host
	PORT = 30000 # The same port as used by the server

	print "Starting Program"

	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	s.bind((HOST, PORT)) # Bind to the port
	s.listen(5) # Now wait for client connection.
	c, addr = s.accept() # Establish connection with client.

	print "Connected"
	inp = raw_input("Begin calibration? y/n: ")[0]
	if (inp == 'y'):
		calibration(c)

	inp = raw_input("Begin picking? w/c/m/n: ")[0]
	if (inp == 'n'):
		c.close()
		s.close()
		print "Program finish"
		return
	ur_move(c,home_x,home_y,home_z,home_rx,home_ry,home_rz,0)
	if (inp == 'm'):
		while True:
			test_move(c)
	if (inp == 'c'):
		while True:
			pick_mug(c)
	if (inp == 'w'):
		while True:
			inp = raw_input("Ready?")[0]
			pick_wine_glass(c)

if __name__ == '__main__': main()


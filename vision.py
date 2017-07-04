from scipy.spatial import distance as dist
import numpy as np
import cv2
from matplotlib import pyplot as plt
#import argparse
import imutils
import cv2.cv as cv



# Crop parameters
CROP_THRESH = [5, 220]
CROP_SIZES = [300, None]
CROP_WIDTH_HEIGHT_RATIO = 1.1
CROP_FILL = 0.6

CROP_PARAMS = {'thresh': CROP_THRESH,
             'size': CROP_SIZES,
             'ratio': CROP_WIDTH_HEIGHT_RATIO,
             'fill': CROP_FILL}

# Calibration parameters
CAL_THRESH = [10, 230]
CAL_SIZES = [90, None]
CAL_WIDTH_HEIGHT_RATIO = 1.1
CAL_FILL = 0.9

CAL_PARAMS = {'thresh': CAL_THRESH,
             'size': CAL_SIZES,
             'ratio': CAL_WIDTH_HEIGHT_RATIO,
             'fill': CAL_FILL}

# Colors of Bounding Box Points
COLORS = ((0, 0, 255), (240, 0, 159), (255, 0, 0), (255, 255, 0))

# Adjustment to Crop
ADJUSTMENT = 10

# Wine Glass detection parameters
GLASS_PARAM = {'thresh': [29,30],
               'radius': [10,50]}


def capture_pic(name):
    cap = cv2.VideoCapture(0)
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Display the resulting frame
    cv2.imwrite(name,frame)
    plt.imshow(frame)

def order_points(pts):
    # sort the points based on their x-coordinates
    xSorted = pts[np.argsort(pts[:, 0]), :]

    # grab the left-most and right-most points from the sorted
    # x-roodinate points
    leftMost = xSorted[:2, :]
    rightMost = xSorted[2:, :]

    # now, sort the left-most coordinates according to their
    # y-coordinates so we can grab the top-left and bottom-left
    # points, respectively
    leftMost = leftMost[np.argsort(leftMost[:, 1]), :]
    (tl, bl) = leftMost

    # now that we have the top-left coordinate, use it as an
    # anchor to calculate the Euclidean distance between the
    # top-left and right-most points; by the Pythagorean
    # theorem, the point with the largest distance will be
    # our bottom-right point
    D = dist.cdist(tl[np.newaxis], rightMost, "euclidean")[0]
    (br, tr) = rightMost[np.argsort(D)[::-1], :]

    # return the coordinates in top-left, top-right,
    # bottom-right, and bottom-left order
    return np.array([tl, tr, br, bl], dtype="float32")

def extract_contours(image, min_thresh=10, max_thresh=50):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (7, 7), 0)

    # perform edge detection, then perform a dilation + erosion to
    # close gaps in between object edges
    edged1 = cv2.Canny(gray, min_thresh, max_thresh)
    edged2 = cv2.dilate(edged1, None, iterations=1)
    edged = cv2.erode(edged2, None, iterations=1)

    cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]

    return edged, cnts


def find_box(cnts, minsize = 50, maxsize = None, squareness = None, fill = None):
    boxes = []
    for (i, c) in enumerate(cnts):
    # if the contour is not sufficiently large, ignore it
        if cv2.contourArea(c) < minsize:
            print("Object #{} REJECTED because not big enough: ".format(i + 1), cv2.contourArea(c))
            continue

        if maxsize is not None:
            if cv2.contourArea(c) > maxsize:
                continue

        box = cv2.minAreaRect(c)
        box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)

        Xs = [k[0] for k in box]
        Ys = [k[1] for k in box]
        x1=(min(Xs))
        x2=(max(Xs))
        y1=(min(Ys))
        y2=(max(Ys))

        if squareness is not None:
            if ((x2-x1)/(y2-y1)) > squareness or ((x2-x1)/(y2-y1)) < 1/squareness:
                print("Object #{} REJECTED because not square: ".format(i + 1), (x2-x1)/(y2-y1))
                continue


        if fill is not None:
            if cv2.contourArea(c)/((x2-x1)*(y2-y1))<fill:
                print("Object #{} REJECTED because not filled: ".format(i + 1), cv2.contourArea(c)/((x2-x1)*(y2-y1)))
                continue

        box = np.array(box, dtype="int")
        #print (np.shape(box))
        boxes.append(box)
        #print boxes

    return boxes

def draw_boxes(image, boxes, colors):
    for (i,box) in enumerate(boxes):
        cv2.drawContours(image, [box], -1, (0, 255, 0), 2)

        # order the points in the contour such that they appear
        # in top-left, top-right, bottom-right, and bottom-left
        # order, then draw the outline of the rotated bounding
        # box
        rect = order_points(box)

        # loop over the original points and draw them
        for ((x, y), color) in zip(rect, colors):
            cv2.circle(image, (int(x), int(y)), 5, color, -1)

        # draw the object num at the top-left corner
        cv2.putText(image, "Object #{}".format(i + 1),
            (int(rect[0][0] - 15), int(rect[0][1] - 15)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    return image

def crop_table(position_img, crop_params=CROP_PARAMS, adjustment=ADJUSTMENT, colors=COLORS):
    image = cv2.imread(position_img)
    edged, cnts = extract_contours(image, crop_params['thresh'][0], crop_params['thresh'][1])
    boxes = find_box(cnts, crop_params['size'][0], crop_params['size'][1],
                      squareness = crop_params['ratio'], fill = crop_params['fill'])

    max_area = 0
    counter = 0
    for box in boxes:
        Xs = [k[0] for k in box]
        Ys = [k[1] for k in box]
        x1=(min(Xs))
        x2=(max(Xs))
        y1=(min(Ys))
        y2=(max(Ys))

        counter = counter + 1
        if (x2-x1)*(y2-y1)>max_area:
            max_area = (x2-x1)*(y2-y1)
            crop_points = ([int(x1),int(x2),int(y1),int(y2)])
            for (i,cp) in enumerate(crop_points):
                if cp<0:
                    crop_points[i]=0

    crop_img = image[crop_points[2]+ADJUSTMENT:crop_points[3]-ADJUSTMENT,
                     crop_points[0]+ADJUSTMENT:crop_points[1]-ADJUSTMENT]

    crop_input = np.array([[[crop_points[0], crop_points[2]],
                           [crop_points[0], crop_points[3]],
                           [crop_points[1], crop_points[3]],
                           [crop_points[1], crop_points[2]]]])

    draw_boxes(image, crop_input, colors)

    orig_image = cv2.imread(position_img)
    plt.figure()
    plot_224([orig_image, edged, image, crop_img], ["Original", "Edged", "Bounded Image", "Cropped Image"])

    return crop_img, crop_points

def identify_glass(crop_img, crop_points, glass_param = GLASS_PARAM, adjustment = ADJUSTMENT):
    blurred_img = cv2.medianBlur(crop_img, 3)
    gimg = cv2.cvtColor(blurred_img, cv2.COLOR_BGR2GRAY)

    circles = None
    counter = 0

    print glass_param

    while counter < 15:
        print counter
        circles = cv2.HoughCircles(gimg, cv.CV_HOUGH_GRADIENT, 1, 20,
                                   param1=glass_param['thresh'][1], param2=glass_param['thresh'][0]-counter,
                                  minRadius = glass_param['radius'][0], maxRadius = glass_param['radius'][1])
        if circles is not None:
            print 'Glass found'
            break

        counter = counter + 1

    if circles is None:
        print 'No circles Detected, try changing glass_param values'

    cimg = cv2.cvtColor(gimg,cv2.COLOR_GRAY2BGR)

    for i in circles[0,:]:
        # draw the outer circle
        cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center of the circle
        cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

    pixel_coords = [circles[0][0][0] + crop_points[0] + adjustment,
                    circles[0][0][1] + crop_points[2] + ADJUSTMENT]

    return cimg, pixel_coords


def pix2world_cal(cal_img_1, cal_img_2, world_1, world_2, cal_params=CAL_PARAMS, colors=COLORS):
    image_1 = cv2.imread(cal_img_1)
    image_2 = cv2.imread(cal_img_2)

    edged_1, cnts_1 = extract_contours(image_1, cal_params['thresh'][0], cal_params['thresh'][1])
    boxes_1 = find_box(cnts_1, cal_params['size'][0], cal_params['size'][1],
                      squareness = cal_params['ratio'], fill = cal_params['fill'])

    edged_2, cnts_2 = extract_contours(image_2, cal_params['thresh'][0], cal_params['thresh'][1])
    boxes_2 = find_box(cnts_2, cal_params['size'][0], cal_params['size'][1],
                      squareness = cal_params['ratio'], fill = cal_params['fill'])
                      
    print 'box1: ', len(boxes_1)
    print 'box2: ', len(boxes_2)
    boxed_image_1 = draw_boxes(image_1, boxes_1, colors)
    boxed_image_2 = draw_boxes(image_2, boxes_2, colors)
    
    plot_212([boxed_image_1, boxed_image_2], ['1','2'])
    plt.show()

    px_centre = {0: [0,0],
                 1: [0,0]}

    if len(boxes_1)==1:
        Xs = [i[0] for i in boxes_1[0]]
        Ys = [i[1] for i in boxes_1[0]]
        # Switch X and Y coordinates
        px_centre[0][1] = int(np.mean(Xs))
        px_centre[0][0] = int(np.mean(Ys))
    if len(boxes_2)==1:
        Xs = [i[0] for i in boxes_2[0]]
        Ys = [i[1] for i in boxes_2[0]]
        # Switch X and Y coordinates
        px_centre[1][1] = int(np.mean(Xs))
        px_centre[1][0] = int(np.mean(Ys))

    grad_x = (world_1[0]-world_2[0])/(px_centre[0][0] - px_centre[1][0])
    grad_y = (world_1[1]-world_2[1])/(px_centre[0][1] - px_centre[1][1])

    world_px0_x = world_1[0] - px_centre[0][0]*grad_x
    world_px0_y = world_1[1] - px_centre[0][1]*grad_y

    p2w_calval = [grad_x, grad_y, world_px0_x, world_px0_y]

    return p2w_calval

def pix2world(pixels, p2w_calval):
    worldx =  p2w_calval[2]+pixels[1]* p2w_calval[0]
    worldy =  p2w_calval[3]+pixels[0]* p2w_calval[1]
    return worldx, worldy

def plot_212(image, names):
    plt.subplot(211),plt.imshow(image[0])
    plt.title(names[0]), plt.xticks([]), plt.yticks([])
    plt.subplot(212),plt.imshow(image[1])
    plt.title(names[1]), plt.xticks([]), plt.yticks([])

def plot_224(image, names):
    plt.subplot(221),plt.imshow(image[0])
    plt.title(names[0]), plt.xticks([]), plt.yticks([])
    plt.subplot(222),plt.imshow(image[1])
    plt.title(names[1]), plt.xticks([]), plt.yticks([])
    plt.subplot(223),plt.imshow(image[2])
    plt.title(names[2]), plt.xticks([]), plt.yticks([])
    plt.subplot(224),plt.imshow(image[3])
    plt.title(names[3]), plt.xticks([]), plt.yticks([])

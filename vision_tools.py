from scipy.spatial import distance as dist
import numpy as np
import cv2
from matplotlib import pyplot as plt
#import imutils
import copy

CAMERA=0
PORTS=3
CAL_PARAM = {'thresh': [65,100],
            'radius': [4, 9]}

def capture_pic(name, camera = CAMERA, rotation = 0):
    '''Capture a picture from camera.
    
    Args:
        name (str):     The name of picture
        camera (int):   Camera number (Use check_camera() to find correct camera
        rotation(int):  Number of anti-clockwise rotations for image
    
    Returns:
        bool: True if successful, False otherwise
    '''
    try:
        cap = cv2.VideoCapture(camera)
        # Capture frame-by-frame
        ret, frame = cap.read()
    
        frame = np.rot90(frame, rotation)
    # Display the resulting frame
        cv2.imwrite(name,frame)
        plt.imshow(frame)
        return True
    except:
        return False

def check_camera(ports = PORTS):
    
    '''Checks all attached ports for connected camera
    
    Args:
        ports (int): Number of ports to test
    
    Returns:
        Displays matplotlib figure with connected cameras and corresponding port numbers
    '''
    frame = {}
    for i in range(ports):
        vc = cv2.VideoCapture(i)
        if vc.isOpened():
            rval, capture = vc.read()
            frame[i] = capture
        else:
            print ('Webcam ' + str(i) + ' is not connected')

    dim1 = int(np.ceil(np.sqrt(len(frame))))
    dim2 = int(np.ceil(float(len(frame))/dim1))
    dim = (str(dim1) + str(dim2))
    print (len(frame), dim)
    if len(frame)>0:
        plt.figure()
        num = 1
        for (key) in frame:
            #print key, np.shape(frame[key])
            plt.subplot(int(str(dim)+str(num))), plt.imshow(frame[key])
            plt.title(str(key)), plt.xticks([]), plt.yticks([])
            num = num + 1
    else:
        print ("No webcams detected at all")
    plt.show()

def extract_contours(image, min_thresh=10, max_thresh=50, blur = 5, dilate=1, erode=1, cnt_mode = cv2.RETR_TREE):
    '''Extracts contours from an image
    
    Args:
        image(numpy.array): Image array, either RGB/BGR or GRAY, dtype=Any
        min_thresh(int):    Minimum threshold for Canny edge detection
        max_thresh(int):    Maximum threshold for Canny edge detection
        dilate(int):        Number of iterations for dilation of edges
        erode(int):         Number of iterations for erosion of edges
        
    Returns:
        edged(numpy.array):     Image array with final Canny edge detection lines
        img(numpy.array):       Image array of modified input image
        cnts(list):             List of 2-D numpy arrays representing contour points
        hierarchy(numpy.array): Numpy array with np.shape()=(1,m,4), where m is the number of contours. Each entry m has 
                                has 4 entries: [Next Fellow, Previous Fellow, First Child, Parent]
    '''
    gray = copy.copy(image)
    if len(np.shape(gray))>2:
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    
    gray[gray<0] = 0
    gray=gray.astype('uint8')
    gray[gray==255]=0
    
    blurred = cv2.GaussianBlur(gray, (blur, blur), 0)

    # perform edge detection, then perform a dilation + erosion to
    # close gaps in between object edges
    edged = cv2.Canny(blurred, min_thresh, max_thresh)
    edged = cv2.dilate(edged, None, iterations=dilate)
    edged = cv2.erode(edged, None, iterations=erode)

    img, cnts, hierarchy = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    return edged, img, cnts, hierarchy


    ### FIND THE CIRCLES TO CROP FROM THE IR IMAGE
def find_circles(img, num_circles, param=CAL_PARAM, blur=3, show=True):
    gray = copy.copy(img)
    if len(np.shape(gray))>2:
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    
    blurred_img = cv2.medianBlur(gray, blur)

    circles = None
    counter = 0
    #plt.imshow(blurred_img)
    print param
    

    while counter < param['thresh'][0]-1 :
        #print (counter)
        circles = cv2.HoughCircles(blurred_img.astype("uint8"), cv2.HOUGH_GRADIENT, 1, 20,
                                       param1=param['thresh'][1],
                                       param2=param['thresh'][0]-counter,
                                       minRadius = param['radius'][0],
                                       maxRadius = param['radius'][1])

        if circles is not None and len(circles[0])>num_circles:
            print param['thresh'][0]-counter
            print ('All Calibration points found')
            break

        counter = counter + 1

    if circles is None:
        print ('No circles Detected, try changing param values')
        return None
    else:
        cimg = cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR)
        #cimg = copy.copy(ir_img)
        for i in circles[0,:]:
                # draw the outer circle
            cv2.circle(cimg,(int(i[0]),int(i[1])),int(i[2]),(0,0,255),1)
                # draw the center of the ci~rcle
            cv2.circle(cimg,(int(i[0]),int(i[1])),2,(0,0,255),1)

        if show==True:
            cv2.imshow("Calibration Points Identified", cimg)
            cv2.imwrite("calibrated_image.jpg", cimg)
            if cv2.waitKey():# & 0xFF == ord('q'):
                print ("Quit")
            cv2.destroyAllWindows()
        return circles, cimg

def sort_circles3(circles):
        circles_sorted = np.zeros(shape=np.shape(circles))
        # Sort so that circle points ordered clockwise from top left
        circles_sorted[0] = circles[0][np.argsort(circles[0][:,1])]
        #print (circles_sorted[0])
        circles_sorted[0][1:] = circles_sorted[0][1:][np.argsort(circles_sorted[0][1:][:,0])]
        #circles_sorted[0][2:] = circles_sorted[0][2:][np.argsort(circles_sorted[0][2:][:,1])]
        #print (circles_sorted[0])

        crop_points = [circles_sorted[0][0][1], circles_sorted[0][1][1], 
                        circles_sorted[0][0][0], circles_sorted[0][2][0]]
        
        return circles_sorted, crop_points


def find_mean_in_contour(img, mask=None, show=False):
    image = copy.copy(img)
    if len(np.shape(image))>2:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    if mask is None:
        mask = np.ones(image.shape, np.uint8)*255
    
    out = np.zeros_like(img)
    out[mask==255] = image[mask==255]
    
    if show==True:
        plt.figure()
        plt.imshow(mask)
        plt.figure()
        plt.imshow(out)
    
    contour_median = np.median(image[mask==255])
    contour_mean = np.mean(image[mask==255])
    contour_max = image[mask==255].max()
    contour_min = image[mask==255].min()
    return contour_median, contour_mean, contour_max, contour_min

# Remove zero pixels from depth images:
def clean_image(img, show=False):
    default_value = np.median(img)
    for i in range(np.shape(img)[0]):
        for j in range(np.shape(img)[1]):
            if img[i][j]==0:
                if show==True: print 0,
                if img[i][j-1]:
                    img[i][j] = img[i][j-1]
                else:
                    img[i][j] = default_value
    return img

def convert2rgb(image, datatype = "uint8"):
    img = copy.copy(image)
    if len(np.shape(img))>2:
        if np.shape(img)==4:
            r,g,b,x = cv2.split(img)
            img = cv2.merge([r,g,b])
    else:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    img = img.astype(datatype)
    return img

def convert2gray(image, datatype = "uint8"):
    img = copy.copy(image)
    if len(np.shape(img))>2:
        if np.shape(img)==4:
            r,g,b,x = cv2.split(img)
            img = cv2.merge([r,g,b])
    
        img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    img = img.astype(datatype)
    return img

def black_out(image, crop_points):
    img = copy.copy(image)
    img[0:crop_points[0],:]=0
    img[:,0:crop_points[2]]=0
    img[crop_points[1]:,:]=0
    img[:,crop_points[3]:]=0
    return img

def crop_out(image, crop_points):
    img = copy.copy(image)
    crop_points = [int(i) for i in crop_points]

    img = img[crop_points[0]:crop_points[1], crop_points[2]:crop_points[3]]
    
    return img

def create_normalised_image(image, emp_image, cal_image=None):
    img = copy.copy(image)
    emp_img = copy.copy(emp_image)
    
    img = convert2gray(img, 'float32')
    emp_img = convert2gray(emp_img, 'float32')

    clean_img = emp_img - img

    table_rows = np.array([i for i in range(3)])
    table_rows = np.append(table_rows, -(table_rows+1))

    black,white = min([np.mean(clean_img[i]) for i in table_rows]), clean_img.max()
    
    
    
    #Only calls this for Depth data
    if cal_image is not None:
        cal_img = copy.copy(cal_image)
        
        cal_depth = emp_img - cal_img
        
        cal_edged, cal_img_, cal_cnts, cal_hierarchy  = extract_contours(cal_depth, 
                                                                           min_thresh=10, 
                                                                           max_thresh=240)
        
        highest = len(cal_cnts)-1
        mask=create_contour_mask(cal_cnts[highest], cal_depth)

        max_median, max_mean, max_max, max_min = find_mean_in_contour(cal_depth, mask, show=False)

        white = max_median
    
    normclean = copy.copy(clean_img)
    
    normclean[normclean>white] = white
    normclean[normclean<black] = black
    
    normclean = (normclean-black)/(white-black)
    
    return normclean

def create_contour_mask(cnt, image):
    mask = np.zeros(image.shape, np.uint8)
    if len(np.shape(image))>2:
        mask = cv2.drawContours(mask, cnt, -1, (255,255,255), -1)
    else:
        mask = cv2.drawContours(mask, cnt, -1, (255,255,255), -1)
    return mask

def resize_image(image, scale=1, shape=None):
    img = copy.copy(image)
    img = cv2.resize(img, (0,0), fx=scale, fy=scale)
    if shape is not None:
        #print "hello"
        print shape
        img = cv2.resize(img, shape)
    return img

def normclean2cv2(image):
    img = copy.copy(image)
    img = img*255
    img = img.astype("uint8")
    return img
import numpy as np
from matplotlib import pyplot as plt
import cv2
import copy
import os
import errno

from numpysocket_class import numpysocket, HOST, PORT
import vision_tools as vt
PATH_TO_KINECT_IMAGES_DIR = "kinect_images"

from freenect2 import Device, FrameType


CAL_PARAM = {'thresh': [65,100],
            'radius': [4, 9]}

def capture_frames(register=False):
    device = Device()
    frames = {}
    with device.running():
        for type_, frame in device:
            frames[type_] = frame
            if FrameType.Color in frames and FrameType.Depth in frames and FrameType.Ir in frames:
                break
        
    rgb, depth, ir = frames[FrameType.Color], frames[FrameType.Depth], frames[FrameType.Ir]
    image = {}
    image['rgb'] = rgb.to_array()
    image['depth'] = depth.to_array()
    image['ir'] = ir.to_array()
    
    b,g,r,x = cv2.split(image['rgb'])
    image['rgb'] = cv2.merge([r,g,b])
    
    for type_ in image.keys():
        image[type_] = np.rot90(image[type_], 1)
        image[type_] = np.fliplr(image[type_])
    
    if register == False:
        return image
    if register == True:
        depth_rect, color_rect = device.registration.apply(rgb, depth, enable_filter=False)
        return depth_rect, color_rect

def make_path_exist(path):
    try:
        os.makedirs(path)
    except OSError as exception:
        if exception.errno != errno.EEXIST:
            raise

def run_kinect_server(path=None):
    if 'PATH_TO_KINECT_IMAGES_DIR' in locals():
        storage_check = raw_input("Storage location is currently: "+ PATH_TO_KINECT_IMAGES_DIR
                              +"\n If correct, press Enter, else press any key: ")
        if storage_check != "":
            PATH_TO_KINECT_IMAGES_DIR = raw_input("Where to store?: ")
    elif path is not None:
        PATH_TO_KINECT_IMAGES_DIR=path
    else:
        PATH_TO_KINECT_IMAGES_DIR = raw_input("Where to store?: ")
    kinectsocket = numpysocket()
    client_connection, client_address = numpysocket.startServer(kinectsocket)
    while (1):
        action = raw_input("What image to receive?: ")
        if action == "rgb":
            client_connection.send(action)
            image = numpysocket.collectImage(kinectsocket, client_connection, client_address)
            print ("SUCCESSFULLY GOT THE IMAGE!")
            cv2.imshow("Received Image", image)
            cv2.waitKey()
            
        if action == "all":
            img_type = raw_input("test/cal/empty?: ")
            client_connection.send(action)
            rgb = numpysocket.collectImage(kinectsocket, client_connection, client_address)
            depth = numpysocket.collectImage(kinectsocket, client_connection, client_address)
            ir = numpysocket.collectImage(kinectsocket, client_connection, client_address)
            
            np.savez(os.path.join(PATH_TO_KINECT_IMAGES_DIR, 'im_array_{}'.format(img_type)), rgb=rgb, depth=depth, ir=ir)
        if action == "end":
            client_connection.send("stop")
            break
        print "Finished"
    return PATH_TO_KINECT_IMAGES_DIR

def load_npz_as_array(npz_filename, directory=PATH_TO_KINECT_IMAGES_DIR):
    if len(npz_filename)<=4:
        npz_filename = npz_filename + ".npz"
    elif npz_filename[-4:]!=".npz":
        npz_filename = npz_filename + ".npz"
    try:
        im_array = np.load(os.path.join(PATH_TO_KINECT_IMAGES_DIR, npz_filename))
    except ValueError:
        print "Not a valid filename"
    except IOError:
        print ("Cannot convert "+npz_filename+" to array")
    return im_array

def prepare_im_array(dictionary):
    npzfile = copy.copy(dictionary)

    rgb = npzfile['rgb']
    depth = npzfile['depth']
    ir = npzfile['ir']
    
    ir = ir - ir.min()
    ir /= ir.max()
    ir= np.sqrt(ir)

    return rgb, depth, ir

def extract_depth_contours(cnts, hierarchy, image, minsize=80, fineness=0.01, show=True):
    # Prepare Values:
    new_cnts = []
    retain_list = []
    hier = copy.copy(hierarchy)
    used_img = copy.copy(image)
    if hier is None:
        new_cnts = []
        new_h = []
    else:
        for (i,c) in enumerate(cnts):
            if cv2.contourArea(c) < minsize:
                continue

            epsilon = (fineness*cv2.arcLength(c,True))
            approx = cv2.approxPolyDP(c, epsilon, True)

            retain_list.append(i)

            box = np.array(approx, dtype="int")
            new_cnts.append([box])

            if show==True:
                cimg = vt.convert2rgb(used_img, 'uint8')
                cv2.drawContours(cimg, [box], -1, (5,205,5), 1)
                extRight = box[np.argmax(box[:,:,0])][0]
                cv2.putText(cimg, "contour {}".format(i),
                    (extRight[0],extRight[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.25, (25, 0, 255), 1)
                plt.figure()
                plt.imshow(cimg)

        for i in range(np.shape(hier)[1]):
            if i not in retain_list:
                hier[hier==i]=-1

        new_h = np.zeros((1,len(retain_list),4))

        for i,j in enumerate(retain_list):
            new_h[0][i] = hier[0][j]

        for i,j in enumerate(retain_list):
            new_h[new_h==j]=i

        new_h = new_h.astype('int8')


        for j,k in enumerate(new_h[0]):
            if k[3]!=-1:
                if new_h[0][k[3]][2] == -1:
                    new_h[0][k[3]][2] = j
                if k[3]==new_h[0][j-1][3]:
                    new_h[0][j][1]=j-1
                    new_h[0][j-1][0]=j

    return new_cnts, new_h
def create_family(test_cnts, test_h):
    generation = [0 for i in range(len(test_h[0]))]
    family = []
    try:
        test_h = test_h[0]
    except:
        test_h = test_h
    for n,m in enumerate(test_h):
        heir = m
        cont = {}
        cont['id'] = n
        fellow_child = []
        if heir[2] != -1:
            first_child = test_h[heir[2]]
            fellow_child.append(heir[2])        
            while first_child[0] != -1:
                fellow_child.append(first_child[0])
                first_child = test_h[first_child[0]]
            print fellow_child

        if heir[3] != -1:
            generation[n] = generation[heir[3]] + 1

        cont['generation'] = generation[n]
        cont['children'] = fellow_child
        cont['contour'] = test_cnts[n]
        family.append(cont)

    return family

def print_family(family):
    print_family=[]
    for member in family:
        new_member = copy.copy(member)
        del new_member['contour']
        print_family.append(new_member)
        print new_member

        
def sort_family(family, depth_image, height_resolution=0.04, debug=False, show=False):
    depth_img = copy.copy(depth_image)
    max_gen = max([i['generation'] for i in family])
    child_to_pop = []
    if max_gen == 0:
        for member_id, member in enumerate(family):
            parent_mask = np.zeros(depth_img.shape, np.uint8)
            parent_mask = cv2.drawContours(parent_mask, member['contour'], -1, (255), -1)
            family[member_id]['height']=vt.find_mean_in_contour(depth_img, parent_mask, show=show)
    
    for i in range(max_gen):
        for member_id, member in enumerate(family):
            if member['generation']==i:
                parent_mask = np.zeros(depth_img.shape, np.uint8)
                parent_mask = cv2.drawContours(parent_mask, member['contour'], -1, (255), -1)
                children_mask = np.zeros(depth_img.shape, np.uint8)
                child_mask = {}
                child_mean = {}
                if debug: print member['id'], "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
                for num, child in enumerate(member['children']):
                    if debug: print num, child
                    child_mask[num] = np.zeros(depth_img.shape, np.uint8)
                    child_mask[num] = cv2.drawContours(child_mask[num], family[child]['contour'], -1, (255), -1)
                    child_mean[num] = vt.find_mean_in_contour(depth_img, child_mask[num], show=show)
                    if debug: print('Current Child being drawn: ', child)
                    children_mask = child_mask[num] + children_mask
                parent_child_separation = vt.find_mean_in_contour(depth_img, parent_mask-children_mask, show=show)
                if debug: print ("Parent Height: ",member_id, parent_child_separation)
                if parent_child_separation[1] < 0.04:
                        if debug: print (member_id, "Object is a bit short")

                for num, child in enumerate(member['children']):
                    if debug: print ("Child Height: ",child,child_mean[num])
                    family[child]['height'] = child_mean[num]
                    if abs(parent_child_separation[0]-child_mean[num][0]) < height_resolution :

                        if debug: print num, "@@@@@@@@@@@::::::::::::@@@@@@@@@@@@@@@@"
                        if debug: print child

                        removed_child_mask = cv2.drawContours(np.zeros(depth_img.shape, np.uint8), 
                                                              family[child]['contour'], -1, (255), -1)
                        children_mask = children_mask - removed_child_mask
                        child_to_pop.append(child)
                  
                    elif parent_child_separation[0]-child_mean[num][0] > height_resolution:
                        family[child]['generation']=family[member_id]['generation']-1

                    else:
                        family[child]['generation']=family[member_id]['generation']+1


                parent_child_separation = vt.find_mean_in_contour(depth_img, parent_mask - children_mask, show=show)
                family[member_id]['height'] = parent_child_separation
                if debug: print "child_to_pop", child_to_pop
        for child_ in list(reversed(child_to_pop)):
            for member_ in family:
                if child_ in member_['children']:
                    member_['children'].remove(child_)
            family.pop(child_)
        for new_id, new_member in enumerate(family):
            if new_member['id']!=new_id:
                if debug: print "test", new_member['id']
                for kkk in range(new_id):
                    if new_member['id'] in family[kkk]['children']:
                        if debug: print kkk
                        family[kkk]['children'] = [new_id if mmm == new_member['id'] else new_member['id']
                                                   for mmm in family[kkk]['children'] ]
                family[new_id]['id'] = new_id
    return family
#IMPORT CONSTANTS

import numpy as np
from matplotlib import pyplot as plt
import cv2
#matplotlib inline
import copy


import vision_tools as vt

import kinect_vision as kv
from kinect_vision import PATH_TO_KINECT_IMAGES_DIR

class TableObject(object):
    
    def __init__(self, member=None, family=None, rgb_member=None, name="Unknown"):
        
        
        
        if member is not None:
            self.contour = member['contour']
            self.height = member['height']
            self.children = member['children']
            self.generation = member['generation']
            self.id = member['id']
            self.name = name

            #Call all methods
            self.nuclear = TableObject.return_nuclear(self, member, family)
            TableObject.find_children(self, family)
            self.box = self.extract_minBox(self.contour)
            self.centre, self.radius = TableObject.extract_minCircle(self,self.contour)
            self.area = TableObject.find_objectArea(self, self.contour)
        
        elif rgb_member is not None:
            self.height = (0, 0, 0, 0)
            self.children = []
            self.generation = 0
            self.name = name
        
        else:
            print "This object does not exist"
        
        if rgb_member is not None:
            self.item_contour = [rgb_member['contour']]
            
            self.rgb_box = TableObject.extract_minBox(self, self.item_contour)
            self.rgb_centre, self.rgb_radius = TableObject.extract_minCircle(self, self.item_contour)
            self.rgb_area = TableObject.find_objectArea(self, self.item_contour)
        
        TableObject.prepare_data(self)
        
    def find_children(self,family):
        children_list = []
        for child in self.children:
            children_list.append(family[child])
        self.children_info = children_list
    
    def return_nuclear(self, member, family):
        nuclear = [member]
        for i in member['children']:
            nuclear.append(family[i])
            print family[i]['id']
        return nuclear
    
    def extract_minBox(self, contour):
        box = cv2.minAreaRect(contour[0])
        box = cv2.boxPoints(box)
        box = np.array(box, dtype="int")
        return box
    
    def extract_minCircle(self, contour):
        centre, radius = cv2.minEnclosingCircle(contour[0])
        return centre, radius
        
    def draw_minBox(self, image, box):
        img = copy.copy(image)
        cv2.drawContours(img, [box], -1, (255,0,0), 1)
        plt.figure()
        plt.imshow(img)
        
    def draw_minCircle(self, image, centre, radius):
        img = copy.copy(image)
        cent = (int(centre[0]), int(centre[1]))
        radi = int(radius)
        cv2.circle(img, cent, radi, (0,255,0), 1)
        plt.figure()
        plt.imshow(img)
    
    def find_objectArea(self, contour):
        area = cv2.contourArea(contour[0])
        return area
    
    def find_objectPerimeter(self, contour):
        perimeter = cv2.arcLength(contour[0],True)
        return perimeter
    
    def print_children_attributes(self):
        print ("Children Info:")
        for i,child in enumerate(self.children_info):
            print (i, child['generation'], child['children'],child['height'])
            
    def print_object_attributes(self, children_info=False):
        object_attr = [attr for attr in dir(self) 
                       if not callable(getattr(self, attr)) 
                       and not attr.startswith("__")]
        for attr in object_attr:
            if attr != 'children_info':
                print attr, ": ", getattr(self, attr)
                
    def find_rgb_average(self, image):
        out = TableObject.filter_object(self, image)
        try:
            contour = self.item_contour
        except:
            contour = self.contour
        img = copy.copy(image)
        
        if len(np.shape(out))>2:
            mask = vt.create_contour_mask(contour, img[:,:,0])
            r_avg = np.mean(img[:,:,0][mask==255])
            g_avg = np.mean(img[:,:,1][mask==255])
            b_avg = np.mean(img[:,:,2][mask==255])
            self.rgb_average = [r_avg, b_avg, g_avg]
        else:
            mask = vt.create_contour_mask(contour, img)
            np.mean(img[mask==255])
    
    def return_object_attributes(self, children_info=False):
        object_attr = [attr for attr in dir(self) 
                       if not callable(getattr(self, attr)) 
                       and not attr.startswith("__")]
        object_dict = {}
        for attr in object_attr:
            if attr != 'children_info' or attr != "nuclear":
                object_dict[attr]=getattr(self, attr)
        return object_dict
        
    def filter_object(self, image, show_contour = False):
        try:
            contour = self.item_contour
            print "item"
        except:
            contour = self.contour
            print "depth"
        #print contour
        img = copy.copy(image)
        obj_mask = vt.create_contour_mask(contour, img)
        out = np.zeros_like(image)
        
        if show_contour:
            cv2.drawContours(img, [self.contour], -1, (255,255,255), 1)
        out[obj_mask==255] = img[obj_mask==255]
        return out
    
    def label_object(self, image):
        out = TableObject.filter_object(self, image)
        plt.figure(str(self.name))
        plt.imshow(out)
        plt.show()
        name = raw_input("Name of object?: ")
        self.name = name
    
    def add_depth(self, depth_object):
        try:
            
            self.contour = depth_object.contour
            self.height = depth_object.height
            self.children = depth_object.children
            self.generation = depth_object.generation
            self.nuclear = depth_object.nuclear
            self.id = depth_object.id

            #Call all methods
            self.children_info = depth_object.children_info
            self.box = depth_object.box
            self.centre = depth_object.centre
            self.radius = depth_object.radius
            self.area = depth_object.area

            # Extra
            self.aspect = depth_object.aspect
            self.circularness = depth_object.circularness
            self.fill = depth_object.fill
            
        except Exception as ex:
            template = "An exception of type {0} occurred. Arguments:\n{1!r}"
            message = template.format(type(ex).__name__, ex.args)
            print "There is no depth data to be added"
            print message
    
    
    def prepare_data(self):
        try:
            # prepare aspect ratio
            box = self.rgb_box
            first_line = np.sqrt(sum((box[1]-box[0])**2))
            second_line = np.sqrt(sum((box[2]-box[1])**2))
            rgb_aspect = first_line/second_line
            if rgb_aspect > 1:
                rgb_aspect = second_line/first_line
            self.rgb_aspect = rgb_aspect

            # prepare rgb fill ratio
            rgb_boundBoxArea = cv2.contourArea(self.rgb_box)
            rgb_fill = self.rgb_area/rgb_boundBoxArea
            self.rgb_fill = rgb_fill

            # prepare rgb circularness data
            rgb_boundCircleArea = np.pi*(self.rgb_radius)**2
            rgb_circularness = self.rgb_area/rgb_boundCircleArea
            self.rgb_circularness = rgb_circularness
            
        except Exception as ex:
            template = "An exception of type {0} occurred. Arguments:\n{1!r}"
            message = template.format(type(ex).__name__, ex.args)
            print "RGB data not considered"
            print message
    
        try:
            # FILL
            boundBoxArea = cv2.contourArea(self.box)
            fill = self.area/boundBoxArea
            self.fill = fill

            # ASPECT
            box = self.box
            first_line = np.sqrt(sum((box[1]-box[0])**2))
            second_line = np.sqrt(sum((box[2]-box[1])**2))
            aspect = first_line/second_line
            if aspect > 1:
                aspect = second_line/first_line
            self.aspect = fill

            # CIRCULARNESS
            boundCircleArea = np.pi*(self.radius)**2
            circularness = self.area/boundCircleArea
            self.circularness = circularness
            
        except Exception as ex:
            template = "An exception of type {0} occurred. Arguments:\n{1!r}"
            message = template.format(type(ex).__name__, ex.args)
            print "Depth data not implemented"
            print message
            
    def label_data_auto(self):
        if self.height[0] == 0:
            self.name = 'cd'
        elif self.circularness > 0.8:
            print "no"

def find_object(image, family=None, rgb_family=None, label=True):
    Object_List = {}
    if label is False:
        object_id = 1
    img = copy.copy(image)
    if rgb_family is not None:
        curr_family = rgb_family  
    elif family is not None:
        curr_family = family
    else:
        curr_family = [{'generation': 0, 'id': -1}]
    for obj in curr_family:
        if obj['generation']==0:
            if obj['id']>=0:
                box = cv2.minAreaRect(np.array(obj['contour'])[0])
                box = cv2.boxPoints(box)
                box = np.array(box, dtype="int")
                cv2.drawContours(img, [box], -1, (255,0,0), 1)
            
            
                if rgb_family is None:
                    tobj = TableObject(member = obj, family = family)
                else:
                    tobj = TableObject(rgb_member = obj)
                    TableObject.find_rgb_average(tobj, img)
                if label:
                    TableObject.label_object(tobj, img)
                    Object_List[tobj.name] = tobj
                else:
                    Object_List[str(object_id)] = tobj
                    object_id = object_id+1

    return Object_List
        
def match_rgb_with_depth(family, rgb_family, image, rgb_image, label=True):
    img = copy.copy(image)
    rgb_img = copy.copy(rgb_image)
    
    if label:
        depth_obj_list = find_object(family=family, image=img, label=label)
        obj_list = find_object(rgb_family=rgb_family, image=rgb_img, label=label)
        for item in obj_list.keys():
            if item in depth_obj_list.keys():
                obj_list[item].add_depth(depth_obj_list[item])

    else:
        depth_obj_list = find_object(family=family, image=img, label=label)
        obj_list = find_object(rgb_family=rgb_family, image=rgb_img, label=label)
        print "unmatched"

    return obj_list



def match_rgb_with_depth_v2(family, rgb_family, image, rgb_image, label=False):
    img = copy.copy(image)
    rgb_img = copy.copy(rgb_image)
    
    if label:
        depth_obj_list = find_object(family=family, image=img, label=label)
        obj_list = find_object(rgb_family=rgb_family, image=rgb_img, label=label)
        for item in obj_list.keys():
            if item in depth_obj_list.keys():
                obj_list[item].add_depth(depth_obj_list[item])

    else:
        print "Making depth List ..."
        depth_list = find_object(family=family, image=img, label=label)
        print "Making rgb object list..."
        obj_list = find_object(rgb_family=rgb_family, image=rgb_img, label=label)
        print "Length of depth and obj lists:", len(depth_list), len(obj_list)
        for item in obj_list.keys():
            contour = obj_list[item].item_contour[0]
            mask = vt.create_contour_mask([contour], img)
            final = 0.5
            most_likely_depth = '0'

            for depth_item in depth_list.keys():

                depth_contour = depth_list[depth_item].contour[0]
                total = len(depth_contour)
                inside = 0

                for point in depth_contour:
                    check = cv2.pointPolygonTest((contour), 
                                                           (point[0][0],point[0][1]), 
                                                           measureDist=True)
                    if check>0:
                        inside = inside+1

                frac = float(inside)/total

                if frac>final:
                    final=frac
                    most_likely_depth = depth_item

            if most_likely_depth == '0':
                print item, ": No depth profile found"

            else:
                mask2 = vt.create_contour_mask([depth_list[most_likely_depth].contour[0]], img)
                mask = mask-mask2
                obj_list[item].add_depth(depth_list[most_likely_depth])
        
        print "Matched based on Contour Overlap"

    return obj_list

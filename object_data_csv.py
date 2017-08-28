import os
import pandas as pd
import copy
import numpy as np
import cv2

def create_object_df():
    obj_df = pd.DataFrame(columns = ['name', 
                                     'area', 
                                     'aspect',######
                                     'centre',
                                     'circularness',##### 
                                     'fill', ######
                                     'number of children',#####
                                     'rgb_area',
                                     'rgb_aspect',#####
                                     'rgb_centre',
                                     'rgb_circularness',#####
                                     'rgb_fill',#####
                                     'rgb_average',#####
                                     'median_height',#####
                                     'mean_height' #####
                                    ])
    return obj_df

def prepare_object_dict(tableObject, obj_df):
    copy.copy(tableObject)
    object_dict = tableObject.return_object_attributes()
    
    box = tableObject.rgb_box
    first_line = np.sqrt(sum((box[1]-box[0])**2))
    second_line = np.sqrt(sum((box[2]-box[1])**2))
    rgb_aspect = first_line/second_line
    if rgb_aspect > 1:
        rgb_aspect = second_line/first_line
    object_dict['rgb_aspect'] = rgb_aspect
    
    rgb_boundBoxArea = cv2.contourArea(tableObject.rgb_box)
    rgb_fill = tableObject.rgb_area/rgb_boundBoxArea
    object_dict['rgb_fill'] = rgb_fill
    
    rgb_boundCircleArea = np.pi*(tableObject.rgb_radius)**2
    rgb_circularness = tableObject.rgb_area/rgb_boundCircleArea
    object_dict['rgb_circularness'] = rgb_circularness
    
    try:
        # FILL
        boundBoxArea = cv2.contourArea(tableObject.box)
        fill = tableObject.area/boundBoxArea
        
        # ASPECT
        box = tableObject.box
        first_line = np.sqrt(sum((box[1]-box[0])**2))
        second_line = np.sqrt(sum((box[2]-box[1])**2))
        aspect = first_line/second_line
        if aspect > 1:
            aspect = second_line/first_line
            
        # CIRCULARNESS
        boundCircleArea = np.pi*(tableObject.radius)**2
        circularness = tableObject.area/boundCircleArea
        
        # NUMBER OF CHILDREN
        object_dict['number of children'] = len(tableObject.children)
        
        # HEIGHTS
        median_height = tableObject.height[0]
        mean_height = tableObject.height[1]
        
    except:
        aspect = rgb_aspect
        circularness = rgb_circularness
        fill = rgb_fill
        median_height = 0
        mean_height = 0
        object_dict['number of children'] = 0
        
    object_dict['aspect'] = aspect
    object_dict['circularness'] = circularness
    object_dict['fill'] = fill
    object_dict['median_height'] = median_height
    object_dict['mean_height'] = mean_height
    
    for key in object_dict.keys():
        if key not in obj_df.head():
            del object_dict[key]
    return object_dict

def create_csv(filename, df):
    if len(filename)>4:
        if filename[-4:]!=".csv":
            filename = filename + ".csv"
    header = df.keys()
    if os.path.isfile(filename) is False:
        df.to_csv(filename, columns = header)
    print "done"

def add_to_csv(filename, df):
    with open(filename, 'a') as f:
        df.to_csv(f, header=False)
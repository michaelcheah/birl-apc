import copy

import cv2
from matplotlib import pyplot as plt
from object_data_csv import create_object_df, prepare_object_dict, add_to_csv, create_csv

##################################### Vision Imports ###########################################
import numpy as np
import os

from tableObject_class import TableObject, match_rgb_with_depth, match_rgb_with_depth_v2

import pandas as pd
import ast

def extract_centres(centre):
    if type(centre) is not type(1.0):
        return np.array(ast.literal_eval(centre[1:-2]))
    else:
        return np.array([np.nan, np.nan])
    
excluded_val = ['centre', 'rgb_centre', 'number of children']
extras = ['R','G','B', 'centre_offset']

def prepare_obj_features_param(obj_feat_csv = 'object_features.csv', 
                               excluded_val = excluded_val, 
                               extras=extras):
    
    obj_feat_df = pd.read_csv(obj_feat_csv)
    object_features = copy.copy(obj_feat_df)

    object_features = object_features.drop("Unnamed: 0", 1)
    
    included_val = [i for i in list(object_features.columns.values) if i not in excluded_val]
    for i in extras:
        included_val.append(i)

    xxx = object_features.centre.apply(lambda x: extract_centres(x))
    yyy = object_features.rgb_centre.apply(lambda x: extract_centres(x))
    
    if 'R' in extras and 'B' in extras and 'G' in extras:
        object_features.rgb_average = object_features.rgb_average.apply(lambda x: 
                                                                        np.array(ast.literal_eval(x)))
        object_features[['R','G', 'B']] = pd.DataFrame([x for x in object_features.rgb_average])
        object_features.drop("rgb_average",1)
    
    if 'centre_offset' in extras:
        object_features['centre_offset'] = xxx.subtract(yyy).apply(lambda x: np.sqrt(x[0]**2+x[1]**2))
        object_features['centre_offset'].fillna(0, inplace=True)
    
    obj_features_mean = object_features.groupby("name")[included_val].mean()
    obj_features_std = object_features.groupby("name")[included_val].std()
    
    return obj_features_mean, obj_features_std

def prepare_pick_obj_features_param(original_obj_list, excluded_val, extras):
    object_list = copy.copy(original_obj_list)
    rec_df = create_object_df()
    rec_df = rec_df.drop(excluded_val,1)
    rgb_list = ['R','G','B']
    for i in rgb_list:
        rec_df[i]=np.nan
    rec_df['centre_offset']=np.nan

    for item in sorted(object_list.keys()):
        print item #object_list[item].return_object_attributes().keys()
        rec_dict = prepare_object_dict(object_list[item], rec_df)

        if 'centre' in object_list[item].return_object_attributes().keys():
            xxx = np.array(object_list[item].centre)
            yyy = np.array(object_list[item].rgb_centre)
            magn = lambda x: np.sqrt(x[0]**2+x[1]**2)
            rec_dict['centre_offset']=magn(xxx-yyy)

        else:
            rec_dict['centre_offset'] = 0

        for key in rec_dict.keys():
            if key == "rgb_average":
                for k,i in enumerate(rgb_list):
                    rec_dict[i]=rec_dict[key][k]
            if key in excluded_val:
                del rec_dict[key]

        rec_df = rec_df.append(rec_dict, ignore_index=True)
        
    if 'R' not in extras or 'B' not in extras or 'G' not in extras:
        for i in rgb_list:
            rec_df = rec_df.drop(i, 1)
            
    if 'centre_offset' not in extras:
        rec_df.drop('center_offset',1)
        
    return rec_df.drop("rgb_average",1)

def create_cost_list(obj_features_mean, obj_features_std, rec_df):
    cost_list = []
    for item in range(len(rec_df)):
        obj_series = rec_df.iloc[item]
        obj_features_diff = obj_features_mean.apply(lambda row: abs(row - obj_series), axis=1)
        obj_features_cost = obj_features_diff/obj_features_std
        obj_features_cost.sum(axis=1)
        sum_obj_features_cost=obj_features_cost.sum(axis=1)
        sorted_costs = sum_obj_features_cost.sort_values()
        cost_list.append(sorted_costs)
    return cost_list


def label_object_list(original_obj_list, cost_list, image, show=False):
    object_list = copy.copy(original_obj_list)
    #print object_list
    for i,item in enumerate(sorted(object_list.keys())):
        print cost_list[i].keys()[0:2]
        print "NAME: ", cost_list[i].keys()[0]
        object_list[item].label_object(image, name=cost_list[i].keys()[0], show=show)
    return object_list


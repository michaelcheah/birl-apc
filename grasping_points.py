import numpy as np
from matplotlib import pyplot as plt
import cv2
import copy

def pnt2line(pnt, start, end):
    line_vec = end-start
    pnt_vec = pnt-start
    line_len = np.sqrt(line_vec[0]**2+line_vec[1]**2)
    #print line_vec, line_len
    line_unitvec = (line_vec)/line_len
    pnt_vec_scaled = pnt_vec*1.0/line_len
    t = np.dot(line_unitvec, pnt_vec_scaled)    
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    nearest = line_vec*t
    dist = np.sqrt(sum((pnt_vec-nearest)**2))
    nearest = start + nearest
    return (dist, nearest)

def find_closest_contour(point, family):
    closest_contour=family[0]
    closest_contour['distance'] = cv2.pointPolygonTest((family[0]['contour'][0]), 
                                                   (point[0],point[1]), 
                                                   measureDist=True)
    print len(family)
    for i in range(len(family)):
        distance = cv2.pointPolygonTest((family[i]['contour'][0]), 
                                        (point[0],point[1]), 
                                        measureDist=True)
        print distance, closest_contour['distance']
        if abs(distance)<abs(closest_contour['distance']):
            closest_contour = family[i]
            closest_contour['distance']=distance
    return closest_contour

def find_point_generation(point, family):
    closest_contour = find_closest_contour(point, family)
    print "CONTOUR_ID: ", closest_contour['id']
    distance = cv2.pointPolygonTest(closest_cnt['contour'][0], 
                                    (point[0],point[1]), measureDist=True)
    if distance < 0:
        print "OUTSIDE NEAREST CONTOUR"
        child_count = 0
        for member in family:
            if closest_contour['id'] in member['children']:
                print "INSIDE PARENT"
                parent_dist = cv2.pointPolygonTest(member['contour'][0],
                                                  (point[0], point[1]), measureDist=True)
                if parent_dist<0:
                    print "SOMETHING WRONG HAS HAPPENED"
                else:
                    point_generation = member['generation']
                    child_count = child_count + 1
        if child_count == 0:
            point_generation = closest_contour['generation']-1
    else:
        point_generation = closest_contour['generation']
    
    return point_generation


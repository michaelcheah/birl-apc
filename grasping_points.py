import numpy as np
from matplotlib import pyplot as plt
import cv2
import copy
from scipy.spatial.distance import cdist

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
    #print len(family)
    for i in range(len(family)):
        distance = cv2.pointPolygonTest((family[i]['contour'][0]), 
                                        (point[0],point[1]), 
                                        measureDist=True)
        #print distance, closest_contour['distance']
        if abs(distance)<abs(closest_contour['distance']):
            closest_contour = family[i]
            closest_contour['distance']=distance
    return closest_contour

def find_point_generation(point, family):
    closest_contour = find_closest_contour(point, family)
    #print "CONTOUR_ID: ", closest_contour['id']
    distance = cv2.pointPolygonTest(closest_contour['contour'][0], 
                                    (point[0],point[1]), measureDist=True)
    if distance < 0:
        #print "OUTSIDE NEAREST CONTOUR"
        child_count = 0
        for member in family:
            if closest_contour['id'] in member['children']:
                #print "INSIDE PARENT"
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

def closest_node(node, nodes):
    return nodes[cdist([node], nodes).argmin()], cdist([node], nodes).argmin()

def first_grasping_point(table_object):
    closest_contour = find_closest_contour(table_object.centre, table_object.nuclear)
    current_cnt = []
    for points in closest_contour['contour'][0]:
        current_cnt.append(points[0])
        
    first_node1, node1_id = closest_node(table_object.centre, current_cnt)
    first_neighbours = closest_contour['contour'][0][node1_id-1:node1_id+2:2]
    
    dist1, first_neighbour1 = pnt2line(table_object.centre, first_node1, first_neighbours[0][0])
    
    if len(first_neighbours)>1:
        dist2, first_neighbour2 = pnt2line(table_object.centre, first_node1, first_neighbours[1][0])
    
        if dist1<dist2:
            first_node = first_neighbour1
            first_node2 = first_neighbours[0]
        else:
            first_node = first_neighbour2
            first_node2 = first_neighbours[1]
    
    else:
        first_node = first_neighbour1
        first_node2 = first_neighbours[0]
    
    return first_node, first_node1, first_node2

def find_perpendicular_line(node1, node2):
    vect = node2-node1
    if vect[0] == 0:
        current_line = [1.0,0.0]
    else:
        gradient = float(vect[1])/vect[0]
        if gradient == 0:
            current_line = [0.0,1.0]
        else:
            perp_grad = -1/gradient
            current_line = [1.0, perp_grad]
    return current_line

def find_possible_cross_pairs(table_object, first_node, current_line):
    family = table_object.nuclear
    possible_pairs = []
    
    for x in family:
        sec_cnt = []
        for points in x['contour'][0]:
            sec_cnt.append(points[0])
        for i,j in enumerate(sec_cnt):
            check1 = sec_cnt[i-1] - first_node
            check2 = sec_cnt[i] - first_node
            sign1 = check1[0]*current_line[1] - check1[1]*current_line[0]
            sign2 = check2[0]*current_line[1] - check2[1]*current_line[0]
            
            if sign1*sign2 <0:
                possible_pairs.append([sec_cnt[i-1], sec_cnt[i]])
                
    return possible_pairs

def remove_duplicates(possible_pairs, node1, node2):
    for pair_num,pair in enumerate(list(reversed(possible_pairs))):
        if np.array_equal(node1, pair[0]) or np.array_equal(node1, pair[1]):
            if np.array_equal(node2, pair[0]) or np.array_equal(node2, pair[1]):
                print pair
                possible_pairs.pop(-(pair_num+1))
    return possible_pairs

def find_second_grasping_point(possible_pairs, first_node, table_object):
    possible_second_node = []
    for pair in possible_pairs:
        dist, nearest = pnt2line(first_node, pair[0], pair[1])
        possible_second_node.append([p for p in nearest])
    actual_grasp_centre = []
    
    remove_snode = []
    for num_snode, snode in enumerate(possible_second_node):
        node_separation = snode-first_node
        grasp_centre = first_node + node_separation/2
        
        grasp_node1 = first_node - node_separation*0.02
        grasp_node2 = snode + node_separation* 0.02
        grasp_gen = {}
        for gnum, grasp_point in enumerate([grasp_node1, grasp_centre, grasp_node2]):
            grasp_gen[gnum] = find_point_generation(grasp_point, table_object.nuclear)
        
        if grasp_gen[0]!=grasp_gen[2]:
            remove_snode.append(num_snode)
        elif grasp_gen[1]<grasp_gen[0]:
            remove_snode.append(num_snode)
        else:
            actual_grasp_centre.append(grasp_centre)
    
    for i in list(reversed(remove_snode)):
        possible_second_node.pop(i)
    
    return possible_second_node, actual_grasp_centre

def determine_best_grasping_point(possible_second_node, possible_grasp_centre, first_node, grasping_limit=100):
    second_node = possible_second_node[0]
    grasp_centre = possible_grasp_centre[0]

    for snode in possible_second_node:
        node_dist = np.sqrt(sum((snode-first_node)**2))
        if node_dist < grasping_limit:
            grasping_limit = node_dist
            second_node = snode
            grasp_centre = (snode+first_node)/2
    return second_node, grasp_centre

def display_grasping_points(image, first_node, second_node, grasp_centre, table_object, show=False):
    img = copy.copy(image)
    color = [(255,0,0), (0,255,0), (0,0,255), (0,255,255)]
    for i,points in enumerate([first_node, second_node, grasp_centre, table_object.centre]):
        img = cv2.circle(img, (int(points[0]), int(points[1])),2,color[i],3)
    
    show_img = cv2.resize(img, (0,0), fx=3, fy=3)
    
    cv2.imwrite("grasping_points_img.jpg", show_img)
    
    if show==True:
        plt.figure("Grasping Points")
        plt.imshow(show_img)
        plt.show()
import numpy as np
from matplotlib import pyplot as plt
import cv2

import copy


import vision_tools as vt
import kinect_vision as kv
from kinect_vision import PATH_TO_KINECT_IMAGES_DIR

# Provide Image array:



def run_calibration(empt_all, cali_all, adjust=True, show=True):
    empt, empt_depth, empt_ir = empt_all
    cali, cali_depth, cali_ir = cali_all
    
    empt_ir = cv2.resize(empt_ir, (0,0), fx=2, fy=2)
    
    CAL_PARAM = {'thresh': [85,100],
                 'radius': [8, 16]}
    BLACK = [250, 720, 250, 650]
    
    #for i in range(len(empt_ir)):
    #    if empt_ir[i].max() <0.4:
    #        print i,
    #    if empt_ir[i].max() == 0:
    #        empt_ir = empt_ir[i:-1, 0:-1]
    #        print "CROP NEEDED"
    
    copy_empt_ir = copy.copy(empt_ir)
    
    while True:
        empt_ir2 = vt.convert2gray(copy_empt_ir*255, "uint8")
        empt_ir3 = vt.black_out(empt_ir2, BLACK)
        empt_circles, empt_cimg = vt.find_circles(empt_ir3, 2, param=CAL_PARAM, blur=3, show=False)
        empt_sort,crop = vt.sort_circles3(empt_circles)
        
        empt_ir_img = vt.crop_out(empt_cimg, crop)
        
        if adjust: 
            plt.figure("CIRCLE IMAGE")
            plt.imshow(empt_cimg)
            plt.figure("CROPPED EMPTY IMAGE")
            plt.imshow(empt_ir_img)
            plt.show()
        
            check = raw_input("Continue?: ")
            if check == "yes":
                break
            else:
                cal_check = raw_input("Change calibration?: ")
                if cal_check == "yes":
                    print CAL_PARAM
                    r1 = int(raw_input("Radius 1: "))
                    r2 = int(raw_input("Radius 2: "))
                    t1 = int(raw_input("Thresh 1: "))
                    t2 = int(raw_input("Thresh 2: "))
                    
                    CAL_PARAM = {'thresh': [t1,t2],
                                'radius': [r1, r2]}
                    print "New CAL_PARAM: ", CAL_PARAM
                
                black_check = raw_input("Change black crop?: ")
                if black_check == "yes":
                    print BLACK
                    y1 = int(raw_input("y1: "))
                    y2 = int(raw_input("y2: "))
                    x1 = int(raw_input("x1: "))
                    x2 = int(raw_input("x2: "))
                    BLACK = [y1, y2, x1, x2]
                    print "New BLACK: ", BLACK
        else:
            break
    #get normclean
    new_crop = [i/2 for i in crop]
    empt_img = vt.crop_out(empt_depth, new_crop)
    cali_img = vt.crop_out(cali_depth, new_crop)

    empt_img = vt.clean_image(empt_img)
    cali_img = vt.clean_image(cali_img)
    
    depth_shape = (np.shape(empt_img)[1], np.shape(empt_img)[0])
    
    depth_cal = [empt_img, cali_img, new_crop, depth_shape, empt_sort]
    
    return depth_cal


def run_calibration_rgb(empt_all, cali_all, depth_cal, adjust=True, show=True):
    empt_img, cali_img, crop, depth_shape, empt_sort = depth_cal
    
    empt, empt_depth, empt_ir = empt_all
    cali, cali_depth, cali_ir = cali_all
    
    empt = cv2.resize(empt, (0,0), fx=0.5, fy=0.5)    
        
    RGB_CAL_PARAM = {'thresh': [85,220],
                    'radius': [5, 8]}
    
    RGB_BLACK = [250,630,100,480]
    
    while True:
        empt_2 = vt.convert2gray(empt*255, "uint8")
        empt_3 = vt.black_out(empt_2, RGB_BLACK)

        empt_rgb_circles, empt_rgb_cimg=vt.find_circles(empt_3, 2, param=RGB_CAL_PARAM, blur=3, show=False)
        empt_rgb_sort,rgb_crop = vt.sort_circles3(empt_rgb_circles)
        
        empt_rgb_large_img = vt.crop_out(empt,rgb_crop)

        if adjust:
            plt.figure("CROPPED EMPTY RGB LARGE IMAGE")
            plt.imshow(empt_rgb_large_img)
            plt.show()
        
        
            rgb_check = raw_input("Continue?: ")
            if rgb_check == "yes":
                break
            else:
                rgb_cal_check = raw_input("Change calibration?: ")
                if rgb_cal_check == "yes":
                    print RGB_CAL_PARAM
                    t1 = raw_input("Thresh 1: ")
                    t2 = raw_input("Thresh 2: ")
                    r1 = raw_input("Radius 1: ")
                    r2 = raw_input("Radius 2: ")
                    RGB_CAL_PARAM = {'thresh': [t1,t2],
                                'radius': [r1, r2]}

                rgb_black_check = raw_input("Change black crop?: ")
                if rgb_black_check == "yes":
                    print RGB_BLACK
                    y1 = raw_input("y1: ")
                    y2 = raw_input("y2: ")
                    x1 = raw_input("x1: ")
                    x2 = raw_input("x2: ")
                    RGB_BLACK = [y1, y2, x1, x2]
        else:
            break

    #depth_shape = (np.shape(empt_img)[1], np.shape(empt_img)[0])
    
    empt_rgb_img = vt.resize_image(empt_rgb_large_img, shape=depth_shape)
    rgb_cal = [empt_rgb_img, rgb_crop]
    
    return rgb_cal

def run_image_processing_v2_depth(test_all, depth_cal, show=True):
    # Prepare Calibration Values
    empt_img, cali_img, crop, depth_shape, empt_sort = depth_cal
    
    #Prepare Test Images
    test,test_depth, test_ir = test_all
    test_img2 = vt.crop_out(test_depth, crop)
    test_img = vt.clean_image(test_img2)
    
    if show:
        plt.figure()
        plt.imshow(test_depth)
        plt.show()

    normclean = vt.create_normalised_image(test_img, empt_img, cali_img)
    if show:
        plt.figure("Normclean")
        plt.imshow((normclean*255).astype('uint8'))
        plt.show()
    
    normclean_uint8 = (normclean*255).astype('uint8')

    edged, img, cnts, hierarchy  = vt.extract_contours(normclean_uint8, 
                                                       min_thresh=20, 
                                                       max_thresh=90, 
                                                       dilate=2, 
                                                       erode=1)
    
    
    new_cnts, new_h = kv.extract_depth_contours(cnts, hierarchy, 
                                                normclean*255, 
                                                minsize=200,
                                                show=show)
    if len(new_cnts)>0:
        family = kv.create_family(new_cnts, new_h)
        sorted_family = kv.sort_family(family, normclean, show=False, debug=False)

        if show:
            plt.figure("OBJECT SEGMENTATION")
            mask = np.zeros_like(normclean)
            for member in sorted_family:
                if member['generation']==0:
                    obj_mask = vt.create_contour_mask([member['contour'][0]], normclean)
                    mask = mask + obj_mask

            plt.imshow(mask)
            plt.show()
        print "Depth Done"
    else:
        sorted_family = None
    
    return normclean, sorted_family

def run_image_processing_v2_rgb(test_all, rgb_cal, depth_cal, show=True):
    # Prepare the images
    test, test_depth, test_ir = test_all
    # Prepare rgb values
    empt_rgb_img, rgb_crop = rgb_cal
    test = cv2.resize(test, (0,0), fx=0.5, fy=0.5)
    test_rgb_large_img = vt.crop_out(test, rgb_crop)

    depth_shape = depth_cal[3]
    
    test_rgbx_img = vt.resize_image(test_rgb_large_img, shape=depth_shape)
    
    rgbnormclean = vt.create_normalised_image(test_rgbx_img, empt_rgb_img)
    
    if show:
        plt.figure("RGBNORMCLEAN")
        plt.imshow((rgbnormclean*255).astype('uint8'))
        plt.show()
    
    rgbnormclean_uint8 = (rgbnormclean*255).astype('uint8')

    rgb_edged, rgb_img, rgb_cnts, rgb_hierarchy  = vt.extract_contours(rgbnormclean_uint8, 
                                                       min_thresh=10, 
                                                       max_thresh=100, 
                                                       dilate=3, 
                                                       erode=1,
                                                       cnt_mode = cv2.RETR_TREE)
    
    rgb_family = kv.create_family(rgb_cnts, rgb_hierarchy)
    
    if show:
        plt.figure("OBJECT SEGMENTATION")
        mask = np.zeros_like(rgbnormclean)
        for member in rgb_family:
            if member['generation']==0:
                obj_mask = vt.create_contour_mask([member['contour']], rgbnormclean)
                mask = mask + obj_mask

        plt.imshow(mask)
        plt.show()

    return rgbnormclean, rgb_family, test_rgbx_img

def run_image_processing(test, empt, cali, adjust=True, show=True):
    test,test_depth, test_ir = kv.prepare_im_array(test)
    empt, empt_depth, empt_ir = kv.prepare_im_array(empt)
    cali, cali_depth, cali_ir = kv.prepare_im_array(cali)
    
    empt_ir = cv2.resize(empt_ir, (0,0), fx=2, fy=2)
    
    CAL_PARAM = {'thresh': [85,220],
                 'radius': [10, 16]}
    BLACK = [270,760,230,670]
    
    copy_empt_ir = copy.copy(empt_ir)
    #Find crop
    while True:
        empt_ir2 = vt.convert2gray(copy_empt_ir*255, "uint8")
        empt_ir3 = vt.black_out(empt_ir2, BLACK)
        empt_circles, empt_cimg=vt.find_circles(empt_ir3, 2, param=CAL_PARAM, blur=1, show=False)
        empt_sort,crop = vt.sort_circles3(empt_circles)
        
        empt_ir_img = vt.crop_out(empt_ir, crop)
        
        if adjust: 
            plt.figure("CIRCLES")
            plt.imshow(empt_cimg)
            plt.figure("CROPPED EMPTY IMAGE")
            plt.imshow(empt_ir_img)
            plt.show()
        
            check = raw_input("Continue?: ")
            if check == "yes":
                break
            else:
                cal_check = raw_input("Change calibration?: ")
                if cal_check == "yes":
                    print CAL_PARAM
                    t1 = int(raw_input("Thresh 1: "))
                    t2 = int(raw_input("Thresh 2: "))
                    r1 = int(raw_input("Radius 1: "))
                    r2 = int(raw_input("Radius 2: "))
                    CAL_PARAM = {'thresh': [t1,t2],
                                'radius': [r1, r2]}
                    print "New CAL_PARAM: ", CAL_PARAM
                
                black_check = raw_input("Change black crop?: ")
                if black_check == "yes":
                    print BLACK
                    y1 = int(raw_input("y1: "))
                    y2 = int(raw_input("y2: "))
                    x1 = int(raw_input("x1: "))
                    x2 = int(raw_input("x2: "))
                    BLACK = [y1, y2, x1, x2]
                    print "New BLACK: ", BLACK
        else:
            break
    #get normclean
    crop = [i/2 for i in crop]
    empt_img = vt.crop_out(empt_depth, crop)
    test_img = vt.crop_out(test_depth, crop)
    cali_img = vt.crop_out(cali_depth, crop)

    test_img = vt.clean_image(test_img)
    empt_img = vt.clean_image(empt_img)
    cali_img = vt.clean_image(cali_img)
    
    #plt.figure()
    #plt.imshow(test_img)
    #plt.show()
    #plt.figure()
    #plt.imshow(empt_img)
    #plt.show()
    #plt.figure()
    #plt.imshow(cali_img)
    #plt.show()
    normclean = vt.create_normalised_image(test_img, empt_img, cali_img)
    if show:
        plt.figure("Normclean")
        plt.imshow((normclean*255).astype('uint8'))
        plt.show()
    
    normclean_uint8 = (normclean*255).astype('uint8')

    edged, img, cnts, hierarchy  = vt.extract_contours(normclean_uint8, 
                                                       min_thresh=20, 
                                                       max_thresh=90, 
                                                       dilate=2, 
                                                       erode=1)
    
    
    new_cnts, new_h = kv.extract_depth_contours(cnts, hierarchy, 
                                                normclean*255, 
                                                minsize=200,
                                                show=False)
    family = kv.create_family(new_cnts, new_h)
    sorted_family = kv.sort_family(family, normclean, show=False, debug=False)
    
    
    if show:
        plt.figure("OBJECT SEGMENTATION")
        mask = np.zeros_like(normclean)
        for member in sorted_family:
            if member['generation']==0:
                obj_mask = vt.create_contour_mask([member['contour'][0]], normclean)
                mask = mask + obj_mask

        plt.imshow(mask)
        plt.show()
    print "Depth Done"
    
    test = cv2.resize(test, (0,0), fx=0.5, fy=0.5)
    
    
    ####################################### RGB ##########################################
        #Find crop
    empt = cv2.resize(empt, (0,0), fx=0.5, fy=0.5)    
        
    RGB_CAL_PARAM = {'thresh': [85,220],
                    'radius': [5, 8]}
    
    RGB_BLACK = [300,630,100,430]
    
    while True:
        empt_2 = vt.convert2gray(empt*255, "uint8")
        empt_3 = vt.black_out(empt_2, RGB_BLACK)

        empt_rgb_circles, empt_rgb_cimg=vt.find_circles(empt_3, 2, param=RGB_CAL_PARAM, blur=3, show=False)
        empt_rgb_sort,rgb_crop = vt.sort_circles3(empt_rgb_circles)
        
        empt_rgb_large_img = vt.crop_out(empt,rgb_crop)

        if adjust:
            plt.figure("CROPPED EMPTY RGB LARGE IMAGE")
            plt.imshow(empt_rgb_large_img)
            plt.show()
        
        
            rgb_check = raw_input("Continue?: ")
            if rgb_check == "yes":
                break
            else:
                rgb_cal_check = raw_input("Change calibration?: ")
                if rgb_cal_check == "yes":
                    print CAL_PARAM
                    t1 = raw_input("Thresh 1: ")
                    t2 = raw_input("Thresh 2: ")
                    r1 = raw_input("Radius 1: ")
                    r2 = raw_input("Radius 2: ")
                    RGB_CAL_PARAM = {'thresh': [t1,t2],
                                'radius': [r1, r2]}

                rgb_black_check = raw_input("Change black crop?: ")
                if rgb_black_check == "yes":
                    print RGB_BLACK
                    y1 = raw_input("y1: ")
                    y2 = raw_input("y2: ")
                    x1 = raw_input("x1: ")
                    x2 = raw_input("x2: ")
                    RGB_BLACK = [y1, y2, x1, x2]
        else:
            break
    #get normclean
    test_rgb_large_img = vt.crop_out(test, rgb_crop)

    depth_shape = (np.shape(test_img)[1], np.shape(test_img)[0])
    
    empt_rgb_img = vt.resize_image(empt_rgb_large_img, shape=depth_shape)
    test_rgb_img = vt.resize_image(test_rgb_large_img, shape=depth_shape)
    if show:
        plt.figure()
        plt.imshow(empt_rgb_img)#
        plt.show()
    #ergb_img = vt.clean_image(ergb_img)

    rgbnormclean = vt.create_normalised_image(test_rgb_img, empt_rgb_img)
    
    if show:
        plt.figure("RGBNORMCLEAN")
        plt.imshow((rgbnormclean*255).astype('uint8'))
        plt.show()
    
    rgbnormclean_uint8 = (rgbnormclean*255).astype('uint8')

    rgb_edged, rgb_img, rgb_cnts, rgb_hierarchy  = vt.extract_contours(rgbnormclean_uint8, 
                                                       min_thresh=10, 
                                                       max_thresh=100, 
                                                       dilate=3, 
                                                       erode=1,
                                                       cnt_mode = cv2.RETR_TREE)
    
    rgb_family = kv.create_family(rgb_cnts, rgb_hierarchy)
    
    if show:
        plt.figure("OBJECT SEGMENTATION")
        mask = np.zeros_like(rgbnormclean)
        for member in rgb_family:
            if member['generation']==0:
                obj_mask = vt.create_contour_mask([member['contour']], rgbnormclean)
                mask = mask + obj_mask

        plt.imshow(mask)
        plt.show()

    return normclean, sorted_family, rgbnormclean, rgb_family, test_rgb_img
    
    
    
def main():
    directory = PATH_TO_KINECT_IMAGES_DIR
    test_num = str(1)
    test = kv.load_npz_as_array("im_array_"+test_num, directory)
    cali = kv.load_npz_as_array("im_array_cal"+test_num, directory)
    empt = kv.load_npz_as_array("im_array_empty"+test_num, directory)
    
    run_image_processing(test, empt, cali)

if __name__ == '__main__': main()
    

#!/usr/bin/env python3

# Images are stored in paparazzi/datasets
#
#

import os
from cv2 import cv2
import numpy as np 
import matplotlib.pyplot as plt 
import time

def isFloor(pixel):
    if ((pixel[0]>70 and pixel[0]<100) and 
    (pixel[1]>80 and pixel[1]<100) and 
    (pixel[2]>70 and pixel[2]<100)):
        return True
    else:
        return False


def floorDetect(img):
    img_size = img.shape[0:2]
    floor_mask = np.zeros(img_size)

    for i in range(img_size[0]):
        for j in range(img_size[1]):
            if (isFloor(img[i][j][:])):

                floor_mask[i][j] = 1

    return floor_mask

def snakeHorizon(img):
    img_size = img.shape
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # edge detection
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    cl = clahe.apply(gray)
    edges = cv2.Canny(cl,20,80)
    cv2.imshow('edges',edges)
    
    # track the snake
    track = np.zeros(img_size[0:2])
    
    # find a "floor" pixel at the bottom of the image
    # with the image turned, x is 'up' and y is 'right' in reality
    x = 0
    y = 0
    onFloor = isFloor(img[y][x][:])
    track[y][x] = True
    while not onFloor:
        y += 1
        onFloor = isFloor(img[y][x][:])
        track[y][x] = True

    # move up to closest edge
    onEdge = (edges[y][x] > 0)
    while not onEdge and y>=0:
        x += 1
        onEdge = (edges[y][x] > 0)
        track[y][x] = True

    initial_point_on_edge = [x,y]
    start_edge = initial_point_on_edge
    # follow edge to the left of the image (-y direction)
    while (y>0 and x>0 and x<img_size[1]-1):
        if (edges[y-1][x] > 0):
            y -= 1                  # edge continues left
        elif (edges[y-1][x-1] > 0):   
            y -= 1
            x -= 1                  # edge continues bottom left
        elif (edges[y-1][x+1] > 0):
            y -= 1
            x += 1                  # edge continues top left
        else:
            start_edge = [x,y]      # edge doesn't continue

        track[y][x] = True

    # follow edge to the right of the image (+y direction)
    x = initial_point_on_edge[0]
    y = initial_point_on_edge[1]
    steps_dir_x = 0
    steps_dir_y = 0

    while (y<img_size[0]-1 and x>0 and x<img_size[1]-1):
        y += 1
        if (edges[y][x] > 0):     # edge continues right
            pass
        elif (edges[y][x-1] > 0): # edge continues bottom right
            x -= 1
        elif (edges[y][x+1] > 0): # edge continues top right
            x += 1                  
        else:                     # increase step
            y += 1
            if (edges[y][x] > 0):
                pass
            elif (edges[y][x-1] > 0): 
                x -= 1
            elif (edges[y][x+1] > 0): 
                x += 1
            elif (edges[y][x-2] > 0):
                x -= 2
            elif (edges[y][x+2] > 0):
                x += 2
            else:
                #edge_dir[0] = x - start_edge[0]
                #edge_dir[1] = x - start_edge[1]

                break


        
        track[y][x] = True
    

    # paint track of snake algorithm
    img_w_track = img
    for i in range(img_size[0]):
        for j in range(img_size[1]):
            if track[i][j]:
                img_w_track[i][j][0] = 0
                img_w_track[i][j][1] = 0
                img_w_track[i][j][2] = 255
    cv2.imshow('track',img_w_track)



# imagepath
img_path = 'datasets/cyberzoo_poles/20190121-135009/'
img_nmb = 80211420
#img_path = 'datasets/cyberzoo_poles_panels/20190121-140205/'
#img_nmb = 93549223#80211420
img_name = img_path + str(img_nmb) + ".jpg"
#img_name = './80211420.jpg'
if os.path.isfile(img_name):
    img = cv2.imread(img_name)
    cv2.imshow('image',img)

    #gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    #cv2.imshow('grayscale',gray)

    #eq = cv2.equalizeHist(gray)
    #cv2.imshow('equalized gray', eq)
    
    #clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    #cl1 = clahe.apply(gray)
    #cv2.imshow('CLAHE',cl1)   

    #edges = cv2.Canny(cl1,30,80) 
    #cv2.imshow('edges',edges)
    
    floor = floorDetect(img)
    cv2.imshow('floor',floor)
    snakeHorizon(img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    #plt.title('Raw image')
    #time.sleep(5)
else:
    print("not a valid image")
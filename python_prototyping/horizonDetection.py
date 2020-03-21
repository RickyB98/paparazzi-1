#!/usr/bin/env python3

# Images are stored in paparazzi/datasets
#
#

import os
import sys
import cv2
import numpy as np 

import matplotlib.pyplot as plt 
import random

# Parameters
ransac_iter = 20            # ransac iterations
ransac_max_error = 5        # maximum error a point can have in ransac
obstacle_threshold = 5     # distance from horizon to be considered an obstacle
simulation = True

# floor color thresholds
if simulation:
    B_low = 0
    B_high = 30
    R_low = 70
    R_high = 110
    G_low = 70
    G_high = 100
else:
    B_low = 70
    B_high = 100
    R_low = 70
    R_high = 100
    G_low = 70
    G_high = 100

# open image
#img_path = 'datasets/cyberzoo_poles/20190121-135009/'
#img_nmb = 80211420
img_path = '/home/lunajuliao/paparazzi-tudelft/datasets/cyberzoo_poles_panels/20190121-140205/'
img_nmb = 93349216 #96849201  
#img_path = 'datasets/cyberzoo_canvas_approach/20190121-151448/'
#img_nmb = 54248124
#img_path = 'datasets/sim_poles_panels_mats/20190121-161931/'
##img_nmb = 40800000

img_name = img_path + str(img_nmb) + ".jpg"


if not os.path.isfile(img_name):
    sys.exit()

img = cv2.imread(img_name)
track = np.zeros(img.shape[0:2])
horizon = np.zeros(img.shape[0])

def isFloor(pixel):
    if ((pixel[0]>B_low and pixel[0]<B_high) and 
    (pixel[1]>R_low and pixel[1]<R_high) and 
    (pixel[2]>G_low and pixel[2]<G_high)):
        return True    
    else:
        return False

def findHorizonCandidate(img,edges,p0):
    global track
    x = p0[0]
    y = p0[1]
    onFloor = isFloor(img[y][x][:])
    track[y][x] = True
    while not onFloor and x<img.shape[1]-1:
        x += 1
        onFloor = isFloor(img[y][x][:])
        track[y][x] = True

    # move up to closest edge
    onEdge = (edges[y][x] > 0)
    while not onEdge and x<img.shape[1]-1:
        x += 1
        onEdge = (edges[y][x] > 0)
        track[y][x] = True
    return [x,y]

def followHorizonLeft(edges,p0,y_lim):
    global track, horizon
    x = p0[0]
    y = p0[1]

    while (y>y_lim and x>0 and x<edges.shape[1]-1):
        y -= 1
        if (edges[y][x] > 0):     # edge continues right
            pass
        elif (edges[y][x-1] > 0): # edge continues bottom right
            x -= 1
        elif (edges[y][x+1] > 0): # edge continues top right
            x += 1                  
        else:                     # increase step
            y -= 1
            if (edges[y][x] > 0):
                horizon[y+1] = x
            elif (edges[y][x-1] > 0):
                horizon[y+1] = x 
                x -= 1
            elif (edges[y][x+1] > 0): 
                horizon[y+1] = x
                x += 1
            else:
                y = y+2     # if the edge does not continue, revert to last know edge position
                break
        horizon[y] = x

    return y
    
def followHorizonRight(edges,p0):
    global track, horizon
    x = p0[0]
    y = p0[1]
    while (y<edges.shape[0]-1 and x>0 and x<edges.shape[1]-1):
        y += 1
        if (edges[y][x] > 0):     # edge continues right
            pass
        elif (edges[y][x-1] > 0): # edge continues bottom right
            x -= 1
        elif (edges[y][x+1] > 0): # edge continues top right
            x += 1                  
        else:                     # increase step
            y += 1
            if (y<edges.shape[0]-1 and edges[y][x] > 0):
                horizon[y-1] = x
            elif (y<edges.shape[0]-1 and edges[y][x-1] > 0):
                horizon[y-1] = x 
                x -= 1
            elif (y<edges.shape[0]-1 and edges[y][x+1] > 0): 
                horizon[y-1] = x
                x += 1
            else:
                y = y-2     # if the edge does not continue, revert to last know edge position
                break
        horizon[y] = x

    return y
    

def floorDetect(img):
    img_size = img.shape[0:2]
    floor_mask = np.zeros(img_size)

    for i in range(img_size[0]):
        for j in range(img_size[1]):
            if (isFloor(img[i][j][:])):

                floor_mask[i][j] = 1

    return floor_mask

def snakeHorizon(img):
    global track, horizon
    img_size = img.shape
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # edge detection
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    cl = clahe.apply(gray)
    if simulation:
        edges = cv2.Canny(cl,100,150)
    else:
        edges = cv2.Canny(cl,15,80)
    cv2.imshow('edges',edges)
    
    x = 0
    y = 0
    y_max = 0
    while (y<img_size[0]):
        [x,y] = findHorizonCandidate(img,edges,[x,y])
        if (x==img_size[1]-1):
            x = 0
            y += 1
            continue
        else:
            horizon[y] = x
            # can limit y_lim to y_max to avoid overwriting past edges, however, it would be helpful to know which one is better
            # other idea: do snake horizon > ransacHorizon > second snake horizon only keeping lines close to the ransac Horizon
            y_min = followHorizonLeft(edges, [x,y], 0)  
            y_max = followHorizonRight(edges, [x,y])
            # could go right first and use distance to decide char *imgif we want to overwrite when moving left
            y = y_max + 1
            x = 0
            # if the segment is too short, scrap it
            if (y_max-y_min < 5):
                for i in range(y_min,y_max+1):
                    horizon[i] = 0    
        #break
    #print("exit while")

def ransacHorizonLine(iterations, threshold, img = None):
    global horizon
    elements = horizon.shape[0]             # pylint: disable=E1136     # pylint/issues/3139
    best_error = threshold * (elements+1)
    error = np.zeros(iterations)
    m = np.zeros(iterations)
    b = np.zeros(iterations)
    for i in range(iterations):
        [s1,s2] = random.sample(range(elements),2)
        if (horizon[s2] == 0 and horizon[s1] == 0):
            continue

        m[i] = (horizon[s2]-horizon[s1])/(s2-s1)
        b[i] = horizon[s1] -  m[i]*s1

        for j in range(elements):
            # draw on image
            x = int(np.round(m[i]*j + b[i]))
            if ((img is not None) and (x>=0) and (x<img.shape[1])):
                img[j][x][0] = 255
                img[j][x][1] = 155
                img[j][x][2] = 0

            delta = np.abs(horizon[j] - m[i]*j - b[i])
            if delta < threshold:
                error[i] = error[i] + delta
            else:
                error[i] = error[i] + threshold
        
        if error[i] < best_error:
            best_error = error[i]
            best_m = m[i]
            best_b = b[i]
    
    best_horizon = np.zeros(elements)
    for j in range(elements):
        x = int(np.round(best_m*j + best_b))
        best_horizon[j] = x
        if img is not None:
            img[j][x][0] = 0
            img[j][x][1] = 255
            img[j][x][2] = 0

    if img is not None:
        cv2.imshow('horizon_lines',img)
    
    return best_horizon



#cv2.imshow('image',img)

gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
#cv2.imshow('grayscale',gray)

eq = cv2.equalizeHist(gray)
#cv2.imshow('equalized gray', eq)

#clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
#cl1 = clahe.apply(gray)
#cv2.imshow('CLAHE',cl1)   

#edges = cv2.Canny(cl1,30,80) 
#cv2.imshow('edges',edges)

#floor = floorDetect(img)
#cv2.imshow('floor',floor)
snakeHorizon(img)
img_ransac = cv2.imread(img_name)
best_horizon = ransacHorizonLine(ransac_iter, ransac_max_error)

obstacle = np.zeros(horizon.shape)
for i in range(horizon.shape[0]):       # pylint: disable=E1136     # pylint/issues/3139
    obstacle_threshold = 10
    if (abs(best_horizon[i]-horizon[i]) > obstacle_threshold):
        obstacle[i] = horizon[i]
    else:
        obstacle[i] = -1


img_w_track = cv2.imread(img_name)
img_w_horizon = cv2.imread(img_name)

for i in range(img.shape[0]):
    for j in range(img.shape[1]):
        if track[i][j]:
            img_w_track[i][j][0] = 0
            img_w_track[i][j][1] = 0
            img_w_track[i][j][2] = 255

    value = int(obstacle[i])
    if value >= 0:
        img_w_horizon[i][value][0] = 0
        img_w_horizon[i][value][1] = 0
        img_w_horizon[i][value][2] = 255
    if (best_horizon[i]>=0 and best_horizon[i]<img_w_horizon.shape[1]):
        img_w_horizon[i][int(best_horizon[i])][0] = 0
        img_w_horizon[i][int(best_horizon[i])][1] = 255
        img_w_horizon[i][int(best_horizon[i])][2] = 0
    img_w_track[i][int(horizon[i])][0] = 0
    img_w_track[i][int(horizon[i])][1] = 0
    img_w_track[i][int(horizon[i])][2] = 255


#cv2.imshow('track',img_w_track)
cv2.imshow('horizon',img_w_horizon)

cv2.waitKey(0)
cv2.destroyAllWindows()


# %%

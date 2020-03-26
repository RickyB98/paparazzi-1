import sys
import math
import cv2 as cv
import numpy as np

simulation = False
# open image
img_path = 'datasets/cyberzoo_poles/20190121-135009/'
img_nmb = 80211420
#img_path = '/home/lunajuliao/paparazzi-tudelft/datasets/cyberzoo_poles_panels/20190121-140205/'
#img_nmb = 96849201  #93349216 # 
#img_path = 'datasets/cyberzoo_canvas_approach/20190121-151448/'
#img_nmb = 54248124
#img_path = 'datasets/sim_poles_panels_mats/20190121-161931/'
##img_nmb = 40800000

img_name = img_path + str(img_nmb) + ".jpg"

# Loads an image
src = cv.imread(img_name, cv.IMREAD_GRAYSCALE)

# Check if image is loaded fine
if src is None:
    print ('Error opening image!')
    print ('Usage: hough_lines.py [image_name -- default ' + default_file + '] \n')
    sys.exit()

   
#dst = cv.Canny(src, 50, 200, None, 3)
# edge detection
clahe = cv.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
cl = clahe.apply(src)
if simulation:
    edges = cv.Canny(cl,100,150)
else:
    edges = cv.Canny(cl,15,80)
cv.imshow('edges',edges)
# Copy edges to the images that will display the results in BGR
cdst = cv.cvtColor(edges, cv.COLOR_GRAY2BGR)
cdstP = np.copy(cdst)

lines = cv.HoughLines(edges, 1, 5* np.pi / 180, 70, None, 0, 0)

if lines is not None:
    for i in range(0, len(lines)):
        rho = lines[i][0][0]
        theta = lines[i][0][1]
        a = math.cos(theta)
        b = math.sin(theta)
        x0 = a * rho
        y0 = b * rho
        pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
        pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
        cv.line(cdst, pt1, pt2, (0,0,255), 3, cv.LINE_AA)


linesP = cv.HoughLinesP(edges, 2, np.pi / 180, 50, None, 50, 10)

if linesP is not None:
    for i in range(0, len(linesP)):
        l = linesP[i][0]
        cv.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv.LINE_AA)

cv.imshow("Source", src)
cv.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)

cv.waitKey()
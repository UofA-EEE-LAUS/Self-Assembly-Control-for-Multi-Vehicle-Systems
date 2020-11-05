# -*- coding: utf-8 -*-
"""
Created on Mon Oct 19 19:16:02 2020

@author: SARAVANA KUMAR
"""

import sim                  #V-rep library
import sys
import os
import six.moves.urllib as urllib
import time                #used to keep track of time
import numpy as np         #array library
import math
import matplotlib.pyplot as mlp   #used for image plotting
import cv2 
import zipfile
import scipy.io
from collections import defaultdict
from io import StringIO
from pylab import *

def ROI (img): # to crop the image for the region of our interest
    x1,y1 = [187, 50]
    x2,y2 = [322, 50]
    x3,y3 = [5, 190]
    x4,y4 = [494, 190]
    top_left_x = min([x1,x2,x3,x4])
    top_left_y = min([y1,y2,y3,y4])
    bot_right_x = max([x1,x2,x3,x4])
    bot_right_y = max([y1,y2,y3,y4])
    img = img[top_left_y:bot_right_y+1, top_left_x:bot_right_x+1] #  Cropping the igae here from the decided coordinates
    return img

def Homography (CX, CY,img): # to convert the selected pixel from planar view to top-down view
    width,height = 512,512
    matrix = [[ 3.57954734e+00,  1.35403401e+00, -7.34289760e+02],
       [-4.96728553e-02,  3.55160915e+00, -5.21564981e-01],
       [-5.65459330e-05,  4.93340418e-03,  1.00000000e+00]] # to reduce the computational complexity directly used the homography matrix
    p = (CX,CY)
    px = (matrix[0][0]*p[0] + matrix[0][1]*p[1] + matrix[0][2]) / ((matrix[2][0]*p[0] + matrix[2][1]*p[1] + matrix[2][2])) # formula to convert the pixel(x,y) in planar to top-down view
    py = (matrix[1][0]*p[0] + matrix[1][1]*p[1] + matrix[1][2]) / ((matrix[2][0]*p[0] + matrix[2][1]*p[1] + matrix[2][2]))
    p_after = (int(px), int(py)) # putting them together as a single entity
    return p_after

def Modification (cam_pos, cX, cY): # modifying the calculated position of the objects with respect to theri sides to the rover
    pairs = []
    if (cam_pos == 2 ): # front facing of the camera
        if (cX > 256): # placing the position as per the cartesian coordination
            cX_p = cX - 256
            x_real = cX_p/500 # dividing the pixel by scaling factor for real time conversion
            cY = 512 - cY
            y_real = cY/264 # dividing by scaling factor
            y_real = y_real + 0.55
            pairs.append(x_real)
            pairs.append(y_real)
        if (cX < 256):
            cX_p = 256 - cX
            x_real = (cX_p/500) * (-1)
            cY = 512 - cY
            y_real = cY/264
            y_real = y_real + 0.55
            pairs.append(x_real)
            pairs.append(y_real)
    if (cam_pos == 3): # right facing of the camera
        if (cX > 256):
            cX_p = cX - 256
            x_real = cX_p/500
            cY = 512 - cY
            y_real = cY/264
            y_real = (y_real + 0.55) * (-1)
            dummy_x = x_real
            dummy_y = y_real
            x_real = dummy_y * (-1)
            y_real = dummy_x * (-1)
            pairs.append(x_real)
            pairs.append(y_real)
        if (cX < 256):
            cX_p = 256 - cX
            x_real = (cX_p/500)
            cY = 512 - cY
            y_real = cY/264
            y_real = y_real + 0.55
            dummy_x = x_real
            dummy_y = y_real
            x_real = dummy_y 
            y_real = dummy_x 
            pairs.append(x_real)
            pairs.append(y_real)
    if (cam_pos == 4): # back facing of the camera
        if (cX > 256):
            cX_p = cX - 256
            x_real = (cX_p/500) * (-1)
            cY = 512 - cY
            y_real = cY/264
            y_real = (y_real + 0.55) * (-1)
            pairs.append(x_real)
            pairs.append(y_real)
        if (cX < 256):
            cX_p = 256 - cX
            x_real = (cX_p/500)
            cY = 512 - cY
            y_real = cY/264
            y_real = (y_real + 0.55) * (-1)
            pairs.append(x_real)
            pairs.append(y_real)
    if (cam_pos == 5): # left facing of the camera 
        if (cX > 256):
            cX_p = cX - 256
            x_real = (cX_p/500) * (-1)
            cY = 512 - cY
            y_real = cY/264
            y_real = (y_real + 0.5) 
            dummy_x = x_real
            dummy_y = y_real
            x_real = dummy_y * (-1)
            y_real = dummy_x * (-1)
            pairs.append(x_real)
            pairs.append(y_real)
        if (cX < 256):
            cX_p = 256 - cX
            x_real = (cX_p/500) * (-1)
            cY = 512 - cY
            y_real = cY/264
            y_real = (y_real + 0.5) * (-1)
            dummy_x = x_real
            dummy_y = y_real
            x_real = dummy_y 
            y_real = dummy_x 
            pairs.append(x_real) # storing the calucated results in the the array 
            pairs.append(y_real)
    return pairs 

def Bounding (img, cam_pos): # To draw the contour bounding box around each object of interest
    img = cv2.resize(img,(512,512)) # image resizing
    image = (img)
     # Grayscale 
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 
    gray = cv2.medianBlur(gray, 15)

    # Find Canny edges 
    edged = cv2.Canny(gray, 30, 200) 
    
    pairs_1 = []
    contours, hierarchy = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    count = 0
    for c in contours:
        # get the bounding rect
        area = cv2.contourArea(c)
        if (area > 1):
            x, y, w, h = cv2.boundingRect(c)
            # draw a green rectangle to visualize the bounding rect
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cX = round(int(x + int(w/2))) #x-coordinate of rectangle's center
            # Up sampling the centre value by multiplying with 2 samples
            # finding the value from the camera origin (256,512)
       
            # dividing it with scaling factor to get the original size in meters
            cY = round(int(y + int(h) )) #y-coordinate of rectangle's center + int(h/2)
            # performing similar in Y valuesa s well
            x_BEV = Homography(cX,cY,img)
            #print (x_BEV)
            cX = x_BEV[0]
            cY = x_BEV[1]
            pairs =  Modification (cam_pos, cX, cY)
            pairs_1 .append (pairs)
             # offset from the camera to the image point
            count = count +1
            w = w / 500 
            #distance = math.sqrt((x_real* x_real) + (y_real*y_real))
            # get the min area rect
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            # convert all coordinates floating point values to int
            box = np.int0(box)

        # Draw all contours 
        cv2.drawContours(image, contours, -1, (0, 255, 0), 1) 
                #Pre-Allocation
    return pairs_1


PI=math.pi  #pi=3.14..., constant

sim.simxFinish(-1) # just in case, close all opened connections

clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5)

if clientID!=-1:  #check if client connection successful
    print ('Connected to remote API server')
    
else:
    print ('Connection not successful')
    sys.exit('Could not connect')

# retrive vision sensor handle
errorCode, cam_Handle = sim.simxGetObjectHandle(clientID,'cam_main',sim.simx_opmode_oneshot_wait)
errorCode,resolution,image=sim.simxGetVisionSensorImage(clientID, cam_Handle,0,sim.simx_opmode_streaming)
errorCode,resolution,image=sim.simxGetVisionSensorImage(clientID, cam_Handle,0,sim.simx_opmode_buffer)

# retrive yaw and pitch handle
errorCode, yaw_Handle = sim.simxGetObjectHandle(clientID,'yaw',sim.simx_opmode_oneshot_wait)
errorCode, pitch_Handle = sim.simxGetObjectHandle(clientID,'pitch',sim.simx_opmode_oneshot_wait)

# rotating yaw to capture the images on each angle

cam_start0, cam_start1 = sim.simxGetJointPosition(clientID, yaw_Handle, sim.simx_opmode_oneshot_wait)

angle = 0
errorCode = sim.simxSetJointPosition(clientID, yaw_Handle, angle*math.pi/180, sim.simx_opmode_oneshot_wait)
real_coordinate = [] # storing the values from the arrays
for cam_start1 in range(1,6) : # to rotate camera to cover the 360 degree
   errorCode = sim.simxSetJointPosition(clientID, yaw_Handle, angle*math.pi/180, sim.simx_opmode_oneshot_wait)
   errorCode,resolution,image=sim.simxGetVisionSensorImage(clientID, cam_Handle,0,sim.simx_opmode_buffer)
   im = np.array (image, dtype=np.uint8) # taking the array from vision sensor and plotting it as image
   im.resize([resolution[0],resolution[1],3]) # pre processing of the image to make it suitable for processing
   im = cv2.rotate(im, cv2.ROTATE_180)
   im = cv2.flip(im, 1)
   
   if cam_start1 != 1: # repeating the collowing functions till it reaches backit original position
       cropped = ROI (im) # ROI function
       pairs = Bounding(cropped, cam_start1) # bounding box of the objects
       real_coordinate.append(pairs) # to calculate the real world values
   angle = angle-90
   
# Plotting the cartesian coordinate system havinf the rover as origin
x_plot = []
y_plot = []
pos_BEV = []
my_array = np.asarray(real_coordinate)

for idx, i in enumerate (my_array): # processing to separate the x and y values
    length = len(my_array[idx])
    j = 0
    while j < length:
        pos_BEV.append(my_array[idx][j][0])
        x_plot.append(math.ceil((my_array[idx][j][0]) * 100)/ 100)
        y_plot.append(math.ceil((my_array[idx][j][1]) * 100)/ 100)
        j += 1
    
x = np.asarray(x_plot)
y = np.asarray(y_plot)
fig = plt.figure() # plotting as the graph
ax = fig.add_subplot(111)

scatter(x,y)

[ plot( [dot_x,dot_x] ,[0,dot_y], '-', linewidth = 0.5 ) for dot_x,dot_y in zip(x,y) ] 
[ plot( [0,dot_x] ,[dot_y,dot_y], '-', linewidth = 0.5 ) for dot_x,dot_y in zip(x,y) ]

left,right = ax.get_xlim()
low,high = ax.get_ylim()
arrow( left, 0, right -left, 0, length_includes_head = True, head_width = 0.02 )
arrow( 0, low, 0, high-low, length_includes_head = True, head_width = 0.02 ) 

grid()

show()

# Transferring the data to matlab by storing it as mat file and then loading the file in matlab
scipy.io.savemat('test.mat', dict(x=x, y=y))
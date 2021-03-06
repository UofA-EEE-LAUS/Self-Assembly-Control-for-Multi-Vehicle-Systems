
import cv2
import numpy as np
import os 

directory = r'C:/Users/SARAVANA KUMAR/Desktop/Birds_View'
os.chdir(directory)
filename = 'Test_Real_Warp.jpg'

circles = np.zeros((4,2),np.int)
counter = 0

def mousepoints(event,x,y,flags,params):
    global counter
    if event == cv2. EVENT_LBUTTONDOWN:
        circles[counter] = x,y
        counter=counter+1
        print(circles)
        

img = cv2.imread('Test_3.jpg')
#(height, width) = img.shape[:2]

while True:
    if counter == 4:
        width,height = 640,480
        src = np.float32([circles[0], circles[1], circles[2], circles[3]])
        des = np.float32([[0,0],[width,0],[0,height],[width,height] ])
        matrix = cv2.getPerspectiveTransform(src, des)
        #matrix = cv2.findHomography(src, des)
        #print(matrix)
        imgOutput = cv2.warpPerspective(img, matrix, (width,height))
        #imgOutput = cv2.flip(imgOutput, 1)
        cv2.imshow("Output Image", imgOutput)   
        cv2.imwrite(filename, imgOutput)
        
    for x in range (0,4):
        cv2.circle(img,(circles[x][0], circles[x][1]),3,(0,255,0),cv2.FILLED)
        
        
        cv2.imshow("Original Image", img)
        cv2.setMouseCallback("Original Image", mousepoints)
        
        cv2.waitKey(1)
    
    
 
print("After saving image:")   
print(os.listdir(directory)) 
  
print('Successfully saved') 
        

    
    
        


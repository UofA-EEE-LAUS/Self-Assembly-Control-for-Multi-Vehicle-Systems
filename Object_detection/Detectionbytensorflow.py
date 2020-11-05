import sim                  #V-rep library
import sys
import os
import time                #used to keep track of time
import numpy as np         #array library
import math
import matplotlib
import matplotlib.pyplot as mlp   #used for image plotting
import cv2 
import tensorflow as tf
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

def Homography (CX, CY): # to convert the selected pixel from planar view to top-down view
    width,height = 512,512
    matrix = [[ 3.57954734e+00,  1.35403401e+00, -7.34289760e+02],# to reduce the computational complexity directly used the homography matrix
       [-4.96728553e-02,  3.55160915e+00, -5.21564981e-01],
       [-5.65459330e-05,  4.93340418e-03,  1.00000000e+00]]
    p = (CX,CY)
    px = (matrix[0][0]*p[0] + matrix[0][1]*p[1] + matrix[0][2]) / ((matrix[2][0]*p[0] + matrix[2][1]*p[1] + matrix[2][2])) # formula to convert the pixel(x,y) in planar to top-down view
    py = (matrix[1][0]*p[0] + matrix[1][1]*p[1] + matrix[1][2]) / ((matrix[2][0]*p[0] + matrix[2][1]*p[1] + matrix[2][2]))
    p_after = (int(px), int(py))
    return p_after

def Modification (cam_pos, cX, cY): # modifying the calculated position of the objects with respect to theri sides to the rover
    pairs = []
    if (cam_pos == 2 ): # front facing camera
        if (cX > 256): # placing the position as per the cartesian coordination
            cX_p = cX - 256
            x_real = cX_p/500
            cY = 512 - cY
            y_real = cY/264
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
    if (cam_pos == 3): # Right Facing camera
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
    if (cam_pos == 4): # back facing camera
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
    if (cam_pos == 5): # Left facing camera 
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
            pairs.append(x_real) # store the found x and y poisitions in the array list
            pairs.append(y_real)
    return pairs
def obj_image_test(Image, cam_pos): # Tensorflow object detection for calssification and bounding box
    cap = cv2.rotate(Image, cv2.ROTATE_180) # pre processing the image for the need
    pairs_1 = []
    from object_detection.utils import label_map_util

    from object_detection.utils import visualization_utils as vis_util
    # What model to download.
    MODEL_NAME = 'classification_graph' # model folder name where the following files are being stored
# Path to frozen detection graph. This is the actual model that is used for the object detection.
    PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

# List of the strings that is used to add correct label for each box.
    PATH_TO_LABELS = os.path.join('training', 'labelmap.pbtxt')

    NUM_CLASSES = 2 # two classes for classification 

    # ## Load a (frozen) Tensorflow model into memory.
    detection_graph = tf.Graph()
    with detection_graph.as_default():
      od_graph_def = tf.GraphDef()
      with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')
        
        # PATH_TO_LABELS 

    label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
    categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
    category_index = label_map_util.create_category_index(categories)

    categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
    category_index = label_map_util.create_category_index(categories)
    
    # ## Main fucntion code


    def load_image_into_numpy_array(image):
      (im_width, im_height) = image.size
      return np.array(image.getdata()).reshape(
          (im_height, im_width, 3)).astype(np.uint8)
  
    
    
    with detection_graph.as_default():
        with tf.Session(graph=detection_graph) as sess:
            
            #cap = cv2.rotate(Image, cv2.ROTATE_180)
            image_np = Image;
            # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
            image_np_expanded = np.expand_dims(image_np, axis=0)
            image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
            # Each box represents a part of the image where a particular object was detected.
            boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            scores = detection_graph.get_tensor_by_name('detection_scores:0')
            classes = detection_graph.get_tensor_by_name('detection_classes:0')
            num_detections = detection_graph.get_tensor_by_name('num_detections:0')
            # Actual detection.
            (boxes, scores, classes, num_detections) = sess.run(
                [boxes, scores, classes, num_detections],
                feed_dict={image_tensor: image_np_expanded})
            # Visualization of the results of a detection.
            vis_util.visualize_boxes_and_labels_on_image_array(
                image_np,
                np.squeeze(boxes),
                np.squeeze(classes).astype(np.int32),
                np.squeeze(scores),
                category_index,
                use_normalized_coordinates=True,
                line_thickness=8)
            width_pixels = 0;
            height_pixels = 0;
            (frame_height, frame_width) = cap.shape[:2]
              # Generating mid x and  ground y positions
            for i,b in enumerate(boxes[0]):
                if( int(classes[0][i] == 1)):
                    mid_x = int(((boxes[0][i][1]+boxes[0][i][3])/2) * 512)
                    mid_y =  ((boxes[0][i][0]+boxes[0][i][2])/2) 
                    ground_y = int((mid_y+((boxes[0][i][2]-boxes[0][i][0])/2)) * 512)
                    BEV = Homography(mid_x, ground_y) # Converting the planar pixel to BEV
                    pairs =  Modification (cam_pos, cX, cY) # post processing for positioning the objects based on the camera view
                    pairs_1 .append (pairs)
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

# roating yaw to capture the images on each angle

cam_start0, cam_start1 = sim.simxGetJointPosition(clientID, yaw_Handle, sim.simx_opmode_oneshot_wait)

angle = 0
errorCode = sim.simxSetJointPosition(clientID, yaw_Handle, angle*math.pi/180, sim.simx_opmode_oneshot_wait)
real_coordinate = []
for cam_start1 in range(1,6) :
   errorCode = sim.simxSetJointPosition(clientID, yaw_Handle, angle*math.pi/180, sim.simx_opmode_oneshot_wait)
   errorCode,resolution,image=sim.simxGetVisionSensorImage(clientID, cam_Handle,0,sim.simx_opmode_buffer)
   im = np.array (image, dtype=np.uint8) # pre processing the image for deploying the fucntions
   im.resize([resolution[0],resolution[1],3])
   im = cv2.rotate(im, cv2.ROTATE_180)
   im = cv2.flip(im, 1)
   
   if cam_start1 != 1:
       cropped = ROI (im) # to crop the image to our regon of interest
       pairs = obj_image_test(cropped, cam_start1) # deploying object detection
       real_coordinate.append(pairs) # storingall the positions in the list
   angle = angle-90
   
# Plotting the cartesian coordinate system having the rover as origin
x_plot = [] # to seprate and store the values of x and y 
y_plot = []
pos_BEV = []
my_array = np.asarray(real_coordinate)

for idx, i in enumerate (my_array):
    length = len(my_array[idx])
    j = 0
    while j < length:
        pos_BEV.append(my_array[idx][j][0])
        x_plot.append(math.ceil((my_array[idx][j][0]) * 100)/ 100) # rounding off the values 
        y_plot.append(math.ceil((my_array[idx][j][1]) * 100)/ 100)
        j += 1
    
x = np.asarray(x_plot)
y = np.asarray(y_plot)
fig = plt.figure() # plotting the figure as the cartesian graph
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
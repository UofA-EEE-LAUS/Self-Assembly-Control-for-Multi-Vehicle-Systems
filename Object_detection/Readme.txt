The object detection function is achieved by means of both Opencv Contour detection and by deep learning method. The countour detecton method is made to ease the processing power 
usage so that others can use it along with their functions.
the over all pipeline of the program Detectionbycontour is as follows:
Main function-->
    Reads vision sensor input image.
    Crop the region of interest.
    Deploy Bounding box function.
      perform homography transfer function.
      perform modification of the positions as per the camera facing the scene.
    plot the results in the graph.
 
the over all pipeline of the program Detectionbytensorflow is as follows:
Main function-->
    Reads vision sensor input image.
    Crop the region of interest.
    Deploy object detection using tensorflow function.
      perform homography transfer function.
      perform modification of the positions as per the camera facing the scene.
    plot the results in the graph.

The BEV calibration file is used to find the transfer matrix for the detection algorithm and it takes input in clockwise direction. 

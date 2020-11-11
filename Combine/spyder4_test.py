# -*- coding: utf-8 -*-
"""
Created on Sun Apr 26 14:58:49 2020

@author: SARAVANA KUMAR
"""

import vrep
import sys
import numpy as np
import matplotlib.pyplot as mlp

vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')
else:
        print ('Connection not Sucessfull')
        sys.exit ('Could not be able to connect')
errorCode, left_Motor_Handle = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_oneshot_wait)
errorCode, right_Motor_Handle = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_oneshot_wait)
errorCode = vrep.simxSetJointTargetVelocity(clientID,left_Motor_Handle,0.2,vrep.simx_opmode_streaming)
errorCode = vrep.simxSetJointTargetVelocity(clientID,right_Motor_Handle,0.2,vrep.simx_opmode_streaming)
errorCode, sensor1 = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor1',vrep.simx_opmode_oneshot_wait)
errorCode, detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor1,vrep.simx_opmode_streaming)
errorCode, cam_Handle = vrep.simxGetObjectHandle(clientID,'cam1',vrep.simx_opmode_oneshot_wait)
errorCode,resolution,image=vrep.simxGetVisionSensorImage(clientID, cam_Handle,0,vrep.simx_opmode_streaming)
errorCode,resolution,image=vrep.simxGetVisionSensorImage(clientID, cam_Handle,0,vrep.simx_opmode_buffer)
im = np.array (image, dtype=np.uint8)
im.resize([resolution[0],resolution[1],3])
mlp.imshow(im, origin='lower')



clear all;

vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

 [returnCode,Gripper1]=vrep.simxGetObjectHandle(clientID,'P_Grip_right_angle_motor0',vrep.simx_opmode_oneshot_wait);
 
 if (clientID>-1)
    disp('connected');
    
    open_button=input('Whether to open the gripper input 1 or 0');
    
    if open_button==1
        [returnCode]=vrep.simxSetJointForce(clientID,Gripper1,200,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Gripper1,-0.01,vrep.simx_opmode_blocking);
    end
        
    vrep.simxFinish(-1);
     
end
        
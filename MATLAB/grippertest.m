% load api library
vrep=remApi('remoteApi');
% close all the potential link
vrep.simxFinish(-1);
%set up connection to v-rep simulation on port 19999
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
% open the synchronous mode to control the objects in vrep
vrep.simxSynchronous(clientID,true);
% Simulation Initialization
vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
%clientID is -1 if the connection to the server was NOT possible
if (clientID>-1)
    disp('connected to v-rep');
    
    [returnCode,Gripper1]=vrep.simxGetObjectHandle(clientID,'P_Grip_right_angle_motor0',vrep.simx_opmode_oneshot_wait);
    [returnCode,Gripper2]=vrep.simxGetObjectHandle(clientID,'P_Grip_right_angle_motor1',vrep.simx_opmode_oneshot_wait);
    [returnCode]=vrep.simxSetJointForce(clientID,Gripper1,200,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Gripper1,-0.01,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointForce(clientID,Gripper2,200,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Gripper2,-0.01,vrep.simx_opmode_blocking);
     pause(100);
    [returnCode]=vrep.simxSetJointForce(clientID,Gripper1,200,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Gripper1,0.01,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointForce(clientID,Gripper2,200,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Gripper2,0.01,vrep.simx_opmode_blocking);
    
    else
    disp('Failed connecting to remote API server');
end
vrep.delete()
           
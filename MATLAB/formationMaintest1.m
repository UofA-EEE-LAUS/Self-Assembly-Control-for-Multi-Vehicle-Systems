%---------------------------------SETUP----------------------------------%
clear
clc
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
%------------------------------CODE HERE------------------------------%
%Get 3 rovers' position
    [returnCode,rover0]=vrep.simxGetObjectHandle(clientID,strcat('rover0'),vrep.simx_opmode_blocking);
    [returnCode,rover1]=vrep.simxGetObjectHandle(clientID,strcat('rover1'),vrep.simx_opmode_blocking);
    [returnCode,rover2]=vrep.simxGetObjectHandle(clientID,strcat('rover2'),vrep.simx_opmode_blocking);
    [returnCode,position0]=vrep.simxGetObjectPosition(clientID,rover0,-1,vrep.simx_opmode_blocking);
    [returnCode,position1]=vrep.simxGetObjectPosition(clientID,rover1,-1,vrep.simx_opmode_blocking);
    [returnCode,position2]=vrep.simxGetObjectPosition(clientID,rover2,-1,vrep.simx_opmode_blocking);
  %Line Formation
    x0=position0(:,1);
    y0=position0(:,2);
    x1=position1(:,1);
    y1=position1(:,2);
    x2=position2(:,1);
    y2=position2(:,2);
    [xr0,yr0,xr1,yr1,xr2,yr2]=lineformation(x0,y0,x1,y1,x2,y2);
 %Get 3 rovers' new position
        Movetest1(xr0,yr0,pi,int2str(0),clientID,vrep);
        Movetest1(xr1,yr1,pi,int2str(1),clientID,vrep);
        Movetest1(xr2,yr2,pi,int2str(2),clientID,vrep);   
        % pause(1)
        elapsedTime = toc;   
  %Get 3 rovers' euler angle     
    [returnCode,gripper0]=vrep.simxGetObjectHandle(clientID,strcat('P_Grip_right_angle0'),vrep.simx_opmode_blocking);
    [returnCode,gripper1]=vrep.simxGetObjectHandle(clientID,strcat('P_Grip_right_angle1'),vrep.simx_opmode_blocking);
    [returnCode,gripper2]=vrep.simxGetObjectHandle(clientID,strcat('P_Grip_right_angle2'),vrep.simx_opmode_blocking);
    [returnCode,grippereuler0]=vrep.simxGetObjectOrientation(clientID,gripper0,-1,vrep.simx_opmode_blocking);
    [returnCode,grippereuler1]=vrep.simxGetObjectOrientation(clientID,gripper1,-1,vrep.simx_opmode_blocking);
    [returnCode,grippereuler2]=vrep.simxGetObjectOrientation(clientID,gripper2,-1,vrep.simx_opmode_blocking);
     yaw0=grippereuler0(2)*(180/pi);
     yaw1=grippereuler1(2)*(180/pi);
     yaw2=grippereuler2(2)*(180/pi);
%------------------------------CODE HERE------------------------------%
%destroy connection to v-rep simulation
    vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot_wait);
else
    disp('Failed connecting to remote API server');
end
vrep.delete()

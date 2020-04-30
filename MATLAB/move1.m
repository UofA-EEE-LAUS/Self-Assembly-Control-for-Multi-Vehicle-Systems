function []=move1(x,y)
%---------------------------------SETUP----------------------------------%
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

%get position
[returnCode,rover1]=vrep.simxGetObjectHandle(clientID,strcat('rover1'),vrep.simx_opmode_blocking);
[returnCode,position]=vrep.simxGetObjectPosition(clientID,rover1,-1,vrep.simx_opmode_blocking);
positionx=position(:,1);
positiony=position(:,2);
%------------------------------CODE HERE------------------------------%
for threshold=0.1
while(abs(positionx-x) >= threshold || abs(positiony-y) >= threshold)
    rover_radius = 15;
wheel_radius = 5.22;
dphi = 0/ 180 * pi;
phi = 0.5/180*pi;
dist_x = zeros(500);
dist_y = zeros(500);
dist_x(1:500) = x-positionx;
dist_y(1:500) = y-positiony;
i = 1;
elapsedTime = 1;
tic
    %get object position for derivative
    [returnCode,rover1]=vrep.simxGetObjectHandle(clientID,strcat('rover1'),vrep.simx_opmode_blocking);
    [returnCode,positiond]=vrep.simxGetObjectPosition(clientID,rover1,-1,vrep.simx_opmode_blocking);
    positiondx=positiond(:,1);
    positiondy=positiond(:,2);
    const_speed = 2;
    Kp = 0.75;
    Kd = 2.25;
    dx=(x-positionx);
    dy=(y-positiony);
    dphi = 2 / 180 * pi;
    theta = abs(atan(dy/dx));
    
    v_xs = (positiondx - positionx) / elapsedTime;
    v_ys = (positiondy - positiony) / elapsedTime;
    
    v_x = Kp * (const_speed * (dx/abs(dx)) * abs(cos(theta)) - v_xs) + Kd * (const_speed * (dx/abs(dx)) * abs(cos(theta)) - v_xs) / elapsedTime;
    v_y = Kp * (const_speed * (dy/abs(dy)) * abs(sin(theta)) - v_ys) + Kd * (const_speed * (dy/abs(dy)) * abs(sin(theta)) - v_ys) / elapsedTime;
    w = dphi;
    
    v   = ( v_x * cos(phi) + v_y * sin(phi) ) / 7.5;
    v_n = (-v_x * sin(phi) + v_y * cos(phi) ) / 7.5;
    
    v0 = -v * sin(pi/3) + v_n * cos(pi/3) + w * rover_radius;
    v1 =                - v_n             + w * rover_radius;
    v2 =  v * sin(pi/3) + v_n * cos(pi/3) + w * rover_radius;
    %defining motor handles
        %return code functions as a debug tool/error message
        [returnCode,motor01]=vrep.simxGetObjectHandle(clientID,'motor01',vrep.simx_opmode_blocking);
        [returnCode,motor11]=vrep.simxGetObjectHandle(clientID,'motor11',vrep.simx_opmode_blocking);
        [returnCode,motor21]=vrep.simxGetObjectHandle(clientID,'motor21',vrep.simx_opmode_blocking);
        %setting motor speeds for straight line
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor01,v0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor11,v1,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor21,v2,vrep.simx_opmode_blocking);
        %get position
[returnCode,rover1]=vrep.simxGetObjectHandle(clientID,strcat('rover1'),vrep.simx_opmode_blocking);
[returnCode,position]=vrep.simxGetObjectPosition(clientID,rover1,-1,vrep.simx_opmode_blocking);
positionx=position(:,1);
positiony=position(:,2);
 %record position
    dist_x(i) = dist_x(i) - (x - positionx);
    dist_y(i) = dist_y(i) - (y - positiony);
    i = i + 1;
    elapsedTime = toc;
end
end
end

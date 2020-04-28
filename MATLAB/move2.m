function [position]=move2(x,y)
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
[returnCode,rover2]=vrep.simxGetObjectHandle(clientID,strcat,'rover2',vrep.simx_opmode_blocking);
[returnCode,position]=vrep.simxGetObjectPosition(clientID,rover2,-1,vrep.simx_opmode_blocking);
positionx=position(:,1);
positiony=position(:,2);
%------------------------------CODE HERE------------------------------%
for threshold=0.2
while (abs(positionx-x) >= threshold || abs(positiony-y) >= threshold)
    rover_radius=15;
    wheel_radius=5.22;
    const_linearVelocity=(0.5*pi*wheel_radius);
    phi=0/180*pi;
    dphi=0.5/180*pi;
    dx=(x-positionx);
    dy=(y-positiony);
    timelimit = sqrt(dx*dx + dy*dy)/const_linearVelocity;
    vx=dx/timelimit;
    vy=dy/timelimit;
    w=dphi/timelimit;
    v=vx*cos(phi)+vy*sin(phi);
    vn=(-vx)* sin(phi)+vy*cos(phi);
    v0=(-v)*sin(pi/3)+vn*cos(pi/3)+w*rover_radius;
    v1=(-vn)+w*rover_radius;
    v2=v*sin(pi/3)+vn*cos(pi/3)+w*rover_radius;
    %defining motor handles
        %return code functions as a debug tool/error message
        [returnCode,motor0]=vrep.simxGetObjectHandle(clientID,'motor02',vrep.simx_opmode_blocking);
        [returnCode,motor1]=vrep.simxGetObjectHandle(clientID,'motor12',vrep.simx_opmode_blocking);
        [returnCode,motor2]=vrep.simxGetObjectHandle(clientID,'motor22',vrep.simx_opmode_blocking);
        %setting motor speeds for straight line
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor0,v0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor1,v1,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor2,v2,vrep.simx_opmode_blocking);

end
end
end

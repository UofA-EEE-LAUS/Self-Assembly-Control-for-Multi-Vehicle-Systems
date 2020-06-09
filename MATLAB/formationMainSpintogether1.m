%MATLAB script for the rover formation
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
    %------------------------------CODE HERE------------------------------
 
        %Get 3 rovers' position
        [returnCode,rover0]=vrep.simxGetObjectHandle(clientID,strcat('rover0'),vrep.simx_opmode_blocking);
        [returnCode,rover1]=vrep.simxGetObjectHandle(clientID,strcat('rover1'),vrep.simx_opmode_blocking);
        [returnCode,rover2]=vrep.simxGetObjectHandle(clientID,strcat('rover2'),vrep.simx_opmode_blocking);
        [returnCode,position0]=vrep.simxGetObjectPosition(clientID,rover0,-1,vrep.simx_opmode_blocking);
        [returnCode,position1]=vrep.simxGetObjectPosition(clientID,rover1,-1,vrep.simx_opmode_blocking);
        [returnCode,position2]=vrep.simxGetObjectPosition(clientID,rover2,-1,vrep.simx_opmode_blocking);
        position=[position0; position1; position2];
        positionl0=position0;
        positionl1=position1;
        positionl2=position2;
        %Line Formation
        x0=position0(:,1);
        y0=position0(:,2);
        x1=position1(:,1);
        y1=position1(:,2);
        x2=position2(:,1);
        y2=position2(:,2);
        dl01=sqrt((abs(x0-x1)^2)+(abs(y0-y1)^2));
        dl02=sqrt((abs(x0-x2)^2)+(abs(y0-y2)^2));
        thetal01=atan(abs(y0-y1)/abs(x0-x1))*(180/pi);
        thetal02=atan(abs(y0-y2)/abs(x0-x1))*(180/pi);
        [xl0,yl0,xl1,yl1,xl2,yl2]=linedistanceformation(x0,y0,dl01,dl02,thetal01,thetal02);
        %Get 3 rovers' position
        
        Movetest4(xl1,yl1,pi,positionl1,int2str(1),clientID,vrep);
        Movetest4(xl2,yl2,pi,positionl2,int2str(2),clientID,vrep);
        
        elapsedTime = toc;

    %------------------------------CODE HERE------------------------------%
    %destroy connection to v-rep simulation
    vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot_wait);
else
    disp('Failed connecting to remote API server');
end
vrep.delete()

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
    %------------------------------CODE HERE------------------------------%
    
    %Get 3 rovers' position
    [returnCode,rover0]=vrep.simxGetObjectHandle(clientID,strcat('rover0'),vrep.simx_opmode_blocking);
    [returnCode,rover1]=vrep.simxGetObjectHandle(clientID,strcat('rover1'),vrep.simx_opmode_blocking);
    [returnCode,rover2]=vrep.simxGetObjectHandle(clientID,strcat('rover2'),vrep.simx_opmode_blocking);
    [returnCode,position0]=vrep.simxGetObjectPosition(clientID,rover0,-1,vrep.simx_opmode_blocking);
    [returnCode,position1]=vrep.simxGetObjectPosition(clientID,rover1,-1,vrep.simx_opmode_blocking);
    [returnCode,position2]=vrep.simxGetObjectPosition(clientID,rover2,-1,vrep.simx_opmode_blocking);
    position=[position0; position1; position2];
    positionr0 = position0;
    positionr1 = position1;
    positionr2 = position2;
    
    for loop = 1:22
        tic
        %Get 3 rovers' position
        [returnCode,rover0]=vrep.simxGetObjectHandle(clientID,strcat('rover0'),vrep.simx_opmode_blocking);
        [returnCode,rover1]=vrep.simxGetObjectHandle(clientID,strcat('rover1'),vrep.simx_opmode_blocking);
        [returnCode,rover2]=vrep.simxGetObjectHandle(clientID,strcat('rover2'),vrep.simx_opmode_blocking);
        [returnCode,position0]=vrep.simxGetObjectPosition(clientID,rover0,-1,vrep.simx_opmode_blocking);
        [returnCode,position1]=vrep.simxGetObjectPosition(clientID,rover1,-1,vrep.simx_opmode_blocking);
        [returnCode,position2]=vrep.simxGetObjectPosition(clientID,rover2,-1,vrep.simx_opmode_blocking);
        position=[position0; position1; position2];
        positionr0=position0;
        positionr1=position1;
        positionr2=position2;
        %Line Formation
        x0=position0(:,1);
        y0=position0(:,2);
        x1=position1(:,1);
        y1=position1(:,2);
        x2=position2(:,1);
        y2=position2(:,2);
        d01=sqrt((abs(x0-x1)^2)+(abs(y0-y1)^2));
        d02=sqrt((abs(x0-x2)^2)+(abs(y0-y2)^2));
        theta01=atan(abs(y0-y1)/abs(x0-x1))*(180/pi);
        theta02=atan(abs(y0-y2)/abs(x0-x1))*(180/pi);
        [xr0,yr0,xr1,yr1,xr2,yr2]=linedistanceformation(x0,y0,d01,d02,theta01,theta02);
        %Get 3 rovers' position
        Move2(xr2,yr2,positionr2,int2str(2),clientID,vrep);
        Move2(xr1,yr1,positionr1,int2str(1),clientID,vrep);
        % pause(1)
        elapsedTime = toc;
    end
    
    for loop = 1:2
        tic
        %Get 3 rovers' position
        [returnCode,rover0]=vrep.simxGetObjectHandle(clientID,strcat('rover0'),vrep.simx_opmode_blocking);
        [returnCode,rover1]=vrep.simxGetObjectHandle(clientID,strcat('rover1'),vrep.simx_opmode_blocking);
        [returnCode,rover2]=vrep.simxGetObjectHandle(clientID,strcat('rover2'),vrep.simx_opmode_blocking);
        [returnCode,position0]=vrep.simxGetObjectPosition(clientID,rover0,-1,vrep.simx_opmode_blocking);
        [returnCode,position1]=vrep.simxGetObjectPosition(clientID,rover1,-1,vrep.simx_opmode_blocking);
        [returnCode,position2]=vrep.simxGetObjectPosition(clientID,rover2,-1,vrep.simx_opmode_blocking);
        position=[position0; position1; position2];
        positionr0=position0;
        positionr1=position1;
        positionr2=position2;
        %Split rovers
        x0=position0(:,1);
        y0=position0(:,2);
        x1=position1(:,1);
        y1=position1(:,2);
        x2=position2(:,1);
        y2=position2(:,2);
        [xr0,yr0,xr1,yr1,xr2,yr2]=split(x0,y0,x1,y1,x2,y2);
        %Get 3 rovers' position
        Move2(xr2,yr2,positionr2,int2str(2),clientID,vrep);
        Move2(xr1,yr1,positionr1,int2str(1),clientID,vrep);
        % pause(1)
        elapsedTime = toc;
    end
    
      for loop = 1:3
        tic
        %Get 3 rovers' position
        [returnCode,rover0]=vrep.simxGetObjectHandle(clientID,strcat('rover0'),vrep.simx_opmode_blocking);
        [returnCode,rover1]=vrep.simxGetObjectHandle(clientID,strcat('rover1'),vrep.simx_opmode_blocking);
        [returnCode,rover2]=vrep.simxGetObjectHandle(clientID,strcat('rover2'),vrep.simx_opmode_blocking);
        [returnCode,position0]=vrep.simxGetObjectPosition(clientID,rover0,-1,vrep.simx_opmode_blocking);
        [returnCode,position1]=vrep.simxGetObjectPosition(clientID,rover1,-1,vrep.simx_opmode_blocking);
        [returnCode,position2]=vrep.simxGetObjectPosition(clientID,rover2,-1,vrep.simx_opmode_blocking);
        position=[position0; position1; position2];
        positionr0=position0;
        positionr1=position1;
        positionr2=position2;
        %Triangle Formation
        x0=position0(:,1);
        y0=position0(:,2);
        x1=position1(:,1);
        y1=position1(:,2);
        x2=position2(:,1);
        y2=position2(:,2);
        d01=sqrt((abs(x0-x1)^2)+(abs(y0-y1)^2));
        d02=sqrt((abs(x0-x2)^2)+(abs(y0-y2)^2));
        theta01=atan(abs(y0-y1)/abs(x0-x1))*(180/pi);
        theta02=atan(abs(y0-y2)/abs(x0-x1))*(180/pi);
        [xr0,yr0,xr1,yr1,xr2,yr2]=tridistanceformation(x0,y0,d01,d02,theta01,theta02);
          %Get 3 rovers' position
        Move2(xr2,yr2,positionr2,int2str(2),clientID,vrep);
        Move2(xr1,yr1,positionr1,int2str(1),clientID,vrep);
        % pause(1)
        elapsedTime = toc;
    end
    
    %------------------------------CODE HERE------------------------------%
    %destroy connection to v-rep simulation
    vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot_wait);
else
    disp('Failed connecting to remote API server');
end
vrep.delete()

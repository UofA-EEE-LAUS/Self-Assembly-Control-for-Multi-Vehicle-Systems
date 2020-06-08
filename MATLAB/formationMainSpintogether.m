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
    for loop = 1:21
        tic
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
        
        MoveThate2(xl1,yl1,3*pi/2,int2str(1),clientID,vrep);
        MoveThate2(xl2,yl2,3*pi/2,int2str(2),clientID,vrep);
        
        
        elapsedTime = toc;
    end
    pause(1);
  for loop=1:5
      tic;
    [returnCode,rover0]=vrep.simxGetObjectHandle(clientID,strcat('rover0'),vrep.simx_opmode_blocking);
    [returnCode,rover1]=vrep.simxGetObjectHandle(clientID,strcat('rover1'),vrep.simx_opmode_blocking);
    [returnCode,rover2]=vrep.simxGetObjectHandle(clientID,strcat('rover2'),vrep.simx_opmode_blocking);
    [returnCode,positions0]=vrep.simxGetObjectPosition(clientID,rover0,-1,vrep.simx_opmode_blocking);
    [returnCode,positions1]=vrep.simxGetObjectPosition(clientID,rover1,-1,vrep.simx_opmode_blocking);
    [returnCode,positions2]=vrep.simxGetObjectPosition(clientID,rover2,-1,vrep.simx_opmode_blocking);
    position=[positions0; positions1; positions2];
    positiont0=positions0;
    positiont1=positions1;
    positiont2=positions2;
    
    xs0=positions0(:,1);
    ys0=positions0(:,2);
    xs1=positions1(:,1);
    ys1=positions1(:,2);
    xs2=positions2(:,1);
    ys2=positions2(:,2);
    dt01=sqrt((abs(xs0-xs1)^2)+(abs(ys0-ys1)^2));
    dt02=sqrt((abs(xs0-xs2)^2)+(abs(ys0-ys2)^2));
    
    gripper(dt01,int2str(1),clientID,vrep);
    gripper(dt02,int2str(2),clientID,vrep);
    
  end
    pause(3);
    
    
    for loop = 1:2
        tic
        %Get 3 rovers' position
        [returnCode,rover0]=vrep.simxGetObjectHandle(clientID,strcat('rover0'),vrep.simx_opmode_blocking);
        [returnCode,rover1]=vrep.simxGetObjectHandle(clientID,strcat('rover1'),vrep.simx_opmode_blocking);
        [returnCode,rover2]=vrep.simxGetObjectHandle(clientID,strcat('rover2'),vrep.simx_opmode_blocking);
        [returnCode,positionl0]=vrep.simxGetObjectPosition(clientID,rover0,-1,vrep.simx_opmode_blocking);
        [returnCode,positionl1]=vrep.simxGetObjectPosition(clientID,rover1,-1,vrep.simx_opmode_blocking);
        [returnCode,positionl2]=vrep.simxGetObjectPosition(clientID,rover2,-1,vrep.simx_opmode_blocking);
        positionl=[positionl0; positionl1; positionl2];
        positions0=positionl0;
        positions1=positionl1;
        positions2=positionl2;
        %Split rovers
        xl0=positionl0(:,1);
        yl0=positionl0(:,2);
        xl1=positionl1(:,1);
        yl1=positionl1(:,2);
        xl2=positionl2(:,1);
        yl2=positionl2(:,2);
        [xs0,ys0,xs1,ys1,xs2,ys2]=split(xl0,yl0,xl1,yl1,xl2,yl2);
        %Get 3 rovers' position
        MoveThate2(xl1,yl1,pi/2,int2str(1),clientID,vrep);
        MoveThate2(xs2,ys2,pi/2,int2str(2),clientID,vrep);
        
        
        elapsedTime = toc;
    end
    pause(1);
    for loop = 1:3
        tic
        %Get 3 rovers' position
        [returnCode,rover0]=vrep.simxGetObjectHandle(clientID,strcat('rover0'),vrep.simx_opmode_blocking);
        [returnCode,rover1]=vrep.simxGetObjectHandle(clientID,strcat('rover1'),vrep.simx_opmode_blocking);
        [returnCode,rover2]=vrep.simxGetObjectHandle(clientID,strcat('rover2'),vrep.simx_opmode_blocking);
        [returnCode,positions0]=vrep.simxGetObjectPosition(clientID,rover0,-1,vrep.simx_opmode_blocking);
        [returnCode,positions1]=vrep.simxGetObjectPosition(clientID,rover1,-1,vrep.simx_opmode_blocking);
        [returnCode,positions2]=vrep.simxGetObjectPosition(clientID,rover2,-1,vrep.simx_opmode_blocking);
        position=[positions0; positions1; positions2];
        positiont0=positions0;
        positiont1=positions1;
        positiont2=positions2;
        %Triangle Formation
        xs0=positions0(:,1);
        ys0=positions0(:,2);
        xs1=positions1(:,1);
        ys1=positions1(:,2);
        xs2=positions2(:,1);
        ys2=positions2(:,2);
        dt01=sqrt((abs(xs0-xs1)^2)+(abs(ys0-ys1)^2));
        dt02=sqrt((abs(xs0-xs2)^2)+(abs(ys0-ys2)^2));
        thetat01=atan(abs(ys0-ys1)/abs(xs0-xs1))*(180/pi);
        thetat02=atan(abs(ys0-ys2)/abs(xs0-xs1))*(180/pi);
        [xt0,yt0,xt1,yt1,xt2,yt2]=tridistanceformation(xs0,ys0,dt01,dt02,thetat01,thetat02);
        %Get 3 rovers' position
        MoveThate2(xt0,yt0,-pi/2,int2str(0),clientID,vrep);
        MoveThate2(xt1,yt1,pi/3,int2str(1),clientID,vrep);
        MoveThate2(xt2,yt2,2*pi/3,int2str(2),clientID,vrep);
        
        
        elapsedTime = toc;
    end
    
    %------------------------------CODE HERE------------------------------%
    %destroy connection to v-rep simulation
    vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot_wait);
else
    disp('Failed connecting to remote API server');
end
vrep.delete()
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
    xr0=x0;
    yr0=y0;
  for y01=abs(y0-y1)
  if y01~=0
      yr1=y0;
  else
      yr1=y1;
  end
  end 
  for y02=abs(y0-y2)
      if y02~=0
          yr2=y0;
      else
          yr2=y2;
      end
  end
 for x01=abs(x0-x1)
     if x01~=1
         xr1=x0+1;
     else
         xr1=x1;
     end
 end
 for x02=abs(x0-x2)
     if x02~=2
         xr2=x0+2;
     else
         xr2=x2;
     end
 end
 %Get 3 rovers' position
        [positionr0]=move0(xr0,yr0);
        [positionr1]=move1(xr1,yr1);
        [positionr2]=move2(xr2,yr2);
        % pause(1)
        elapsedTime = toc;     
%------------------------------CODE HERE------------------------------%
%destroy connection to v-rep simulation
    vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot_wait);
else
    disp('Failed connecting to remote API server');
end
vrep.delete()

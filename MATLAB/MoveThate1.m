function [] = Move (x,y,phid,Num,clientID,vrep)  % only yaw control


%get position
[returnCode,rover]=vrep.simxGetObjectHandle(clientID,strcat('rover',Num),vrep.simx_opmode_blocking);
[returnCode,laser_sensor]=vrep.simxGetObjectHandle(clientID,strcat('laser_sensor',Num),vrep.simx_opmode_oneshot_wait);
[returnCode,position]=vrep.simxGetObjectPosition(clientID,rover,-1,vrep.simx_opmode_blocking);
[returnCode] = vrep.simxGetObjectOrientation(clientID,laser_sensor,-1,vrep.simx_opmode_streaming);
[returnCode, orientations] = vrep.simxGetObjectOrientation(clientID,laser_sensor,-1,vrep.simx_opmode_buffer);
position_x=position(:,1);
position_y=position(:,2);

if (orientations(1) >= 0)
    theta_sample = orientations(2);
else
    if(orientations(1) < 0)
    	theta_sample = (pi/2-orientations(2))+pi/2;
    elseif(orientations(2) < 0)
    	theta_sample = (pi/2-orientations(2))-pi/2;
    end
end

orien_0 = theta_sample;

rover_radius = 15;
wheel_radius = 5.22;
dphi = 0/ 180 * pi;
phi = orien_0;
% dphi = phi;
dist_x = zeros(500);
dist_y = zeros(500);
dist_x(1:500) = x-position_x;
dist_y(1:500) = y-position_y;
i = 1;
elapsedTime = 1;
threshold = 0.1;
thresholdphi=5/57.3;

[returnCode, position_d]=vrep.simxGetObjectPosition(clientID,rover,-1,vrep.simx_opmode_blocking);
[returnCode, orientations] = vrep.simxGetObjectOrientation(clientID,laser_sensor,-1,vrep.simx_opmode_buffer);
[returnCode,motor_0]=vrep.simxGetObjectHandle(clientID,strcat('motor0',Num),vrep.simx_opmode_blocking);
[returnCode,motor_1]=vrep.simxGetObjectHandle(clientID,strcat('motor1',Num),vrep.simx_opmode_blocking);
[returnCode,motor_2]=vrep.simxGetObjectHandle(clientID,strcat('motor2',Num),vrep.simx_opmode_blocking);
%%%% rover 
[returnCode, orientationsR]=vrep.simxGetObjectOrientation(clientID,rover,-1,vrep.simx_opmode_blocking);

psi_grip=pi-orientationsR(3); % the gripper yaw ralative to y axis in Vrep, range: [0,2*pi) 

%control
while  abs(phid-psi_grip) >= thresholdphi
    
    tic;
    %get object position for derivative
    [returnCode, position_d]=vrep.simxGetObjectPosition(clientID,rover,-1,vrep.simx_opmode_blocking);
    
  
    [returnCode, orientations] = vrep.simxGetObjectOrientation(clientID,laser_sensor,-1,vrep.simx_opmode_buffer);
    
    
    %% yaw control
  [returnCode, orientationsR]=vrep.simxGetObjectOrientation(clientID,rover,-1,vrep.simx_opmode_blocking); % the rover yaw ralative to -y axis in Vrep  range: [-pi,pi) 

  psi_grip=pi-orientationsR(3);% transfor to the gripper yaw ralative to y axis in Vrep  range: [0,2*pi) 

   error=phid-psi_grip; % the angle error range (-2*pi,2*pi) due to the range of phid and psi_grip are all [0, 2*pi)
   if error>-pi && error<=0
       direction=1;
          psi_error=direction*error; % turn left direction with error
   end
      if error>-2*pi && error<=-pi
          direction=-1;
          psi_error=direction*(error+pi);% turn right direction with error+pi
      end
    if error>0 && error<=pi
          direction=1;
          psi_error=direction*error;% turn right direction with error
    end
         if error>pi && error<2*pi
          direction=-1;
          psi_error=direction*(2*pi-error);% turn left direction with 2*pi-error
         end
   
   kp_psi=0.05; % control law only P control; you can add PID control law 
    dpsi_grip = kp_psi*psi_error;
%%
    if (orientations(1) >= 0)
        theta_sample = orientations(2);
    else
        if(orientations(2) > 0)
            theta_sample = pi-(pi/2-orientations(2));
        elseif(orientations(2) < 0)
            theta_sample = (pi/2+orientations(2));
        end
    end
    
    phi = theta_sample + (3/2*pi);
    
 
%% only yaw control
   v_x =0;
    v_y =0;
    w = dpsi_grip;
%%    
    v   = ( v_x * cos(phi) + v_y * sin(phi) ) / 7.5;
    v_n = (-v_x * sin(phi) + v_y * cos(phi) ) / 7.5;
    
    v0 = -v * sin(pi/3) + v_n * cos(pi/3) + w * rover_radius;
    v1 =                - v_n             + w * rover_radius;
    v2 =  v * sin(pi/3) + v_n * cos(pi/3) + w * rover_radius;
    
 
    
    %setting motor speeds for straight line
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_0,v0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_1,v1,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_2,v2,vrep.simx_opmode_blocking);
    
    %get object position
    [returnCode,position]=vrep.simxGetObjectPosition(clientID,rover,-1,vrep.simx_opmode_blocking);
    position_x=position(:,1);
    position_y=position(:,2);
    
    %record position
    dist_x(i) = dist_x(i) - (x - position_x);
    dist_y(i) = dist_y(i) - (y - position_y);
    i = i + 1;
    
    elapsedTime = toc;

end


if  abs(phid-psi_grip)<= thresholdphi
    
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_0,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_1,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_2,0,vrep.simx_opmode_blocking);
end
end

function [] = Move (x,y,Num,clientID,vrep)


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


[returnCode, position_d]=vrep.simxGetObjectPosition(clientID,rover,-1,vrep.simx_opmode_blocking);
[returnCode, orientations] = vrep.simxGetObjectOrientation(clientID,laser_sensor,-1,vrep.simx_opmode_buffer);
[returnCode,motor_0]=vrep.simxGetObjectHandle(clientID,strcat('motor0',Num),vrep.simx_opmode_blocking);
[returnCode,motor_1]=vrep.simxGetObjectHandle(clientID,strcat('motor1',Num),vrep.simx_opmode_blocking);
[returnCode,motor_2]=vrep.simxGetObjectHandle(clientID,strcat('motor2',Num),vrep.simx_opmode_blocking);
    
    
%control
while abs(position_x-x) >= threshold || abs(position_y-y) >= threshold
    
    tic;
    %get object position for derivative
    [returnCode, position_d]=vrep.simxGetObjectPosition(clientID,rover,-1,vrep.simx_opmode_blocking);
    [returnCode, orientations] = vrep.simxGetObjectOrientation(clientID,laser_sensor,-1,vrep.simx_opmode_buffer);
    position_dx=position_d(:,1);
    position_dy=position_d(:,2);
    
    const_speed = 15;
    Kp = 0.75;
    Kd = 2.25;
    
    dx = (x - position_x);
    dy = (y - position_y);

    if (orientations(1) >= 0)
        theta_sample = orientations(2);
    else
        if(orientations(1) < 0)
            theta_sample = (pi/2-orientations(2))+pi/2;
        elseif(orientations(2) < 0)
            theta_sample = (pi/2-orientations(2))-pi/2;
        end
    end
    
    phi = theta_sample + (pi + 1.5);
    disp(phi);
    dphi = 2 / 180 * pi;
    theta = abs(atan(dy/dx));
    
    v_xs = (position_dx - position_x) / elapsedTime;
    v_ys = (position_dy - position_y) / elapsedTime;
    
    v_x = Kp * (const_speed * (dx/abs(dx)) * abs(cos(theta)) - v_xs) + Kd * (const_speed * (dx/abs(dx)) * abs(cos(theta)) - v_xs) / elapsedTime;
    v_y = Kp * (const_speed * (dy/abs(dy)) * abs(sin(theta)) - v_ys) + Kd * (const_speed * (dy/abs(dy)) * abs(sin(theta)) - v_ys) / elapsedTime;
    w = dphi;
    
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

if abs(position_x-x) <= threshold && abs(position_y-y) <= threshold
    
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_0,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_1,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_2,0,vrep.simx_opmode_blocking);
end
end

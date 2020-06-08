function [] = MoveTheta (x,y,position_d,psidformation,Num,clientID,vrep)  % only yaw control


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

for orien_0 = theta_sample
    for rover_radius = 15
        for wheel_radius = 5.22
            for dphi = 0/ 180 * pi
                for phi = orien_0
                    
                    
                    dist_x = zeros(500);
                    dist_y = zeros(500);
                    dist_x(1:500) = x-position_x;
                    dist_y(1:500) = y-position_y;
                    i = 1;
                    elapsedTime = 1;
                    threshold = 0.1;
                    thresholdphi=5/57.3;
                    
                    
                    [returnCode, orientations] = vrep.simxGetObjectOrientation(clientID,laser_sensor,-1,vrep.simx_opmode_buffer);
                    [returnCode,motor_0]=vrep.simxGetObjectHandle(clientID,strcat('motor0',Num),vrep.simx_opmode_blocking);
                    [returnCode,motor_1]=vrep.simxGetObjectHandle(clientID,strcat('motor1',Num),vrep.simx_opmode_blocking);
                    [returnCode,motor_2]=vrep.simxGetObjectHandle(clientID,strcat('motor2',Num),vrep.simx_opmode_blocking);
                    %%%% rover
                    [returnCode, orientationsR]=vrep.simxGetObjectOrientation(clientID,rover,-1,vrep.simx_opmode_blocking);
                    
                    psi_grip=pi-orientationsR(3); % the gripper yaw ralative to y axis in Vrep, range: [0,2*pi)
                    
                    
                    %control
                    while abs(position_x-x) >= threshold || abs(position_y-y) >= threshold
                        
                        tic;
                        %get object position for derivative
                        
                        
                        
                        [returnCode, orientations] = vrep.simxGetObjectOrientation(clientID,laser_sensor,-1,vrep.simx_opmode_buffer);
                        position_dx=position_d(:,1);
                        position_dy=position_d(:,2);
                        
                        const_speed = 10;
                        Kp = 0.75;
                        Kd = 2.25;
                        
                        dx = (x - position_x);
                        dy = (y - position_y);
                        
                        %% yaw control
                        
                        if dy>=0 && dx>=0  %% Yaw command psid is the direction points to destination,  psid range is [0, 2*pi)
                            psid=atan(dx/dy)
                        end
                        if dy<0 && dx>=0
                            psid=pi-atan(dx/(-dy))
                        end
                        if dy<0 && dx<0
                            psid=pi+atan(dx/dy)
                        end
                        if dy>=0 && dx<0
                            psid=2*pi-atan(-dx/dy)
                        end
                        
                        [returnCode, orientationsR]=vrep.simxGetObjectOrientation(clientID,rover,-1,vrep.simx_opmode_blocking); % the rover yaw ralative to -y axis in Vrep  range: [-pi,pi)
                        
                        psi_grip=pi-orientationsR(3);% transfor to the gripper yaw ralative to y axis in Vrep  range: [0,2*pi)
                        
                        error=psid-psi_grip; % the angle error range (-2*pi,2*pi) due to the range of psid and psi_grip are all [0, 2*pi)
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
                        theta = abs(atan(dy/dx));
                        v_xs = (position_dx - position_x) / elapsedTime;
                        v_ys = (position_dy - position_y) / elapsedTime;
                        
                        v_x = Kp * (const_speed * (dx/abs(dx)) * abs(cos(theta)) - v_xs) + Kd * (const_speed * (dx/abs(dx)) * abs(cos(theta)) - v_xs) / elapsedTime;
                        v_y = Kp * (const_speed * (dy/abs(dy)) * abs(sin(theta)) - v_ys) + Kd * (const_speed * (dy/abs(dy)) * abs(sin(theta)) - v_ys) / elapsedTime;
                        w = dpsi_grip;
                        %%
                        v   = ( v_x * cos(phi) + v_y * sin(phi) ) / 7.5;
                        v_n = (-v_x * sin(phi) + v_y * cos(phi) ) / 7.5;
                        
                        v0 = -v * sin(pi/3) + v_n * cos(pi/3) + w * rover_radius;
                        v1 =                - v_n             + w * rover_radius;
                        v2 =  v * sin(pi/3) + v_n * cos(pi/3) + w * rover_radius;
                        
                        
                        
                        %defining motor handles
                        %return code functions as a debug tool/error message
                        [returnCode,motor_0]=vrep.simxGetObjectHandle(clientID,strcat('motor0',Num),vrep.simx_opmode_blocking);
                        [returnCode,motor_1]=vrep.simxGetObjectHandle(clientID,strcat('motor1',Num),vrep.simx_opmode_blocking);
                        [returnCode,motor_2]=vrep.simxGetObjectHandle(clientID,strcat('motor2',Num),vrep.simx_opmode_blocking);
                        %setting motor speeds for straight line
                        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_0,v0,vrep.simx_opmode_blocking);
                        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_1,v1,vrep.simx_opmode_blocking);
                        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_2,v2,vrep.simx_opmode_blocking);
                        
                        %get object position
                        [returnCode,rover]=vrep.simxGetObjectHandle(clientID,strcat('rover',Num),vrep.simx_opmode_blocking);
                        [returnCode,position]=vrep.simxGetObjectPosition(clientID,rover,-1,vrep.simx_opmode_blocking);
                        position_x=position(:,1);
                        position_y=position(:,2);
                        
                        %record position
                        dist_x(i) = dist_x(i) - (x - position_x);
                        dist_y(i) = dist_y(i) - (y - position_y);
                        i = i + 1;
                        
                        elapsedTime = toc;
                        
                    end
                    
                    
                    
                    %control
                    while abs(psidformation-psi_grip) >= thresholdphi %% Reach near the target position, only rotate to the specified angle
                        
                        tic;
                        %get object position for derivative
                        
                        
                        [returnCode, orientations] = vrep.simxGetObjectOrientation(clientID,laser_sensor,-1,vrep.simx_opmode_buffer);
                        position_dx=position_d(:,1);
                        position_dy=position_d(:,2);
                        
                        const_speed = 10;
                        Kp = 0.75;
                        Kd = 2.25;
                        
                        dx = (x - position_x);
                        dy = (y - position_y);
                        
                        %% yaw control
                        [returnCode, orientationsR]=vrep.simxGetObjectOrientation(clientID,rover,-1,vrep.simx_opmode_blocking); % the rover yaw ralative to -y axis in Vrep  range: [-pi,pi)
                        psid=psidformation;
                        psi_grip=pi-orientationsR(3);% transfor to the gripper yaw ralative to y axis in Vrep  range: [0,2*pi)
                        
                        error=psid-psi_grip; % the angle error range (-2*pi,2*pi) due to the range of psid and psi_grip are all [0, 2*pi)
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
                        theta = abs(atan(dy/dx));
                        v_xs = (position_dx - position_x) / elapsedTime;
                        v_ys = (position_dy - position_y) / elapsedTime;
                        
                        v_x = 0;
                        v_y = 0;
                        w = dpsi_grip;
                        %%
                        v   = ( v_x * cos(phi) + v_y * sin(phi) ) / 7.5;
                        v_n = (-v_x * sin(phi) + v_y * cos(phi) ) / 7.5;
                        
                        v0 = -v * sin(pi/3) + v_n * cos(pi/3) + w * rover_radius;
                        v1 =                - v_n             + w * rover_radius;
                        v2 =  v * sin(pi/3) + v_n * cos(pi/3) + w * rover_radius;
                        
                        
                        
                        %defining motor handles
                        %return code functions as a debug tool/error message
                        [returnCode,motor_0]=vrep.simxGetObjectHandle(clientID,strcat('motor0',Num),vrep.simx_opmode_blocking);
                        [returnCode,motor_1]=vrep.simxGetObjectHandle(clientID,strcat('motor1',Num),vrep.simx_opmode_blocking);
                        [returnCode,motor_2]=vrep.simxGetObjectHandle(clientID,strcat('motor2',Num),vrep.simx_opmode_blocking);
                        %setting motor speeds for straight line
                        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_0,v0,vrep.simx_opmode_blocking);
                        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_1,v1,vrep.simx_opmode_blocking);
                        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_2,v2,vrep.simx_opmode_blocking);
                        
                        %get object position
                        [returnCode,rover]=vrep.simxGetObjectHandle(clientID,strcat('rover',Num),vrep.simx_opmode_blocking);
                        [returnCode,position]=vrep.simxGetObjectPosition(clientID,rover,-1,vrep.simx_opmode_blocking);
                        position_x=position(:,1);
                        position_y=position(:,2);
                        
                        %record position
                        dist_x(i) = dist_x(i) - (x - position_x);
                        dist_y(i) = dist_y(i) - (y - position_y);
                        i = i + 1;
                        
                        elapsedTime = toc;
                        
                    end
                    
                    
                    if  abs(position_x-x) <= threshold && abs(position_y-y) <= threshold && abs(psidformation-psi_grip)<= thresholdphi
                        
                        %return code functions as a debug tool/error message
                        [returnCode,motor_0]=vrep.simxGetObjectHandle(clientID,strcat('motor0',Num),vrep.simx_opmode_blocking);
                        [returnCode,motor_1]=vrep.simxGetObjectHandle(clientID,strcat('motor1',Num),vrep.simx_opmode_blocking);
                        [returnCode,motor_2]=vrep.simxGetObjectHandle(clientID,strcat('motor2',Num),vrep.simx_opmode_blocking);
                        %setting motor speeds for straight line
                        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_0,0,vrep.simx_opmode_blocking);
                        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_1,0,vrep.simx_opmode_blocking);
                        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_2,0,vrep.simx_opmode_blocking);
                    end
                end
            end
        end
    end
end
end

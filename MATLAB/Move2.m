function [] = Move2 (x,y,position_d,Num,clientID,vrep)


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
    for elapsedTime = 1
        for threshold = 0.1
            
            %control
            if abs(position_x-x) >= threshold || abs(position_y-y) >= threshold
                
                rover_radius = 15;
                wheel_radius = 5.22;
                dphi = 0/ 180 * pi;
                phi = orien_0;
                %get object position for derivative
                position_dx=position_d(:,1);
                position_dy=position_d(:,2);
                
                const_speed = 15;
                Kp = 0.75;
                Kd = 2.25;
                
                dx = (x - position_x);
                dy = (y - position_y);
                
                
                phi = theta_sample + (pi + 1.5);
                disp(phi);
                dphi = 2 / 180 * pi;
                theta = atan(dy/dx);
                
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
                
                elapsedTime = toc;
                
            else
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
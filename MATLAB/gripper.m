function []=gripper(distance,Num,clientID,vrep)

[returnCode,Gripper]=vrep.simxGetObjectHandle(clientID,strcat('P_Grip_right_angle_motor',Num),vrep.simx_opmode_oneshot_wait);
for threshold=0.1
    
    if abs(distance-0.32)<=threshold
        [returnCode]=vrep.simxSetJointForce(clientID,Gripper,200,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,Gripper,-0.01,vrep.simx_opmode_blocking);
        
    end
    
end
end



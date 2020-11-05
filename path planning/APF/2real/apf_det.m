function [ det_dist, han] = apf_det()


%constructs remote api object
vrep=remApi('remoteApi');
%destroy's any current connections to v-rep simulation
vrep.simxFinish(-1);
%establishes connection to v-rep simulation on port 19999
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
%Get object handles
    [~,C0]=vrep.simxGetObjectHandle(clientID,'C0',vrep.simx_opmode_blocking);
    [~,C1]=vrep.simxGetObjectHandle(clientID,'C1',vrep.simx_opmode_blocking);
    [~,C2]=vrep.simxGetObjectHandle(clientID,'C2',vrep.simx_opmode_blocking);
    [~,C3]=vrep.simxGetObjectHandle(clientID,'C3',vrep.simx_opmode_blocking);
    [~,C4]=vrep.simxGetObjectHandle(clientID,'C4',vrep.simx_opmode_blocking);
    [~,C5]=vrep.simxGetObjectHandle(clientID,'C5',vrep.simx_opmode_blocking);
    
    [returnCode,laser_sensor0]=vrep.simxGetObjectHandle(clientID,'laser_sensor0',vrep.simx_opmode_blocking);
    [returnCode] = vrep.simxGetObjectOrientation(clientID,laser_sensor0,-1,vrep.simx_opmode_streaming);
    [returnCode, orientations] = vrep.simxGetObjectOrientation(clientID,laser_sensor0,-1,vrep.simx_opmode_buffer);
    
     C_H = [C0,C1,C2,C3,C4,C5];
     
    [returnCode,dstate, detectedPoint,han,a] = vrep.simxReadProximitySensor(clientID, laser_sensor0, vrep.simx_opmode_blocking);

% providing the obstacle clearence value
if han == C0
    det_dist = 0.3+0.2;
elseif han == C1
    det_dist = 0.27+0.2; 
elseif han == C2
    det_dist = 0.3+0.2; 
elseif han == C3
    det_dist = 0.1+0.3; 
elseif han == C4
    det_dist = 0.8+0.2; 
elseif han == C5
    det_dist = 0.94+0.2; 
else
    det_dist = 0.94+0.3;
end

end

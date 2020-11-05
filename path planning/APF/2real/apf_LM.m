function [ output ] = apf_LM( test_point,over,obstacle)

%Compute function

k_att=10; % gain factor of attractive force
repu=0;  % initilize the repulsion 
k_rep=100; % gain factor of the repulsive force
% C_H = [68,64,65,66,67,209];
[detection,h] = apf_det(); % the obstacle avoidance reaction range, if the distance between obstacle and robot is smaller than the range, then repulsion affcet its movement, otherwise repulsion is 0
Ke=1;

% The extra POTENTIAL FIELD (Local Minima Avoidance)
if norm(test_point-over)<=detection
   ext_P=(-1/2)*(Ke/detection)*(norm(test_point-over))^2;
   
else
    ext_P=(-Ke)*(norm(test_point-over)-detection/2);
    
end 

% The ATTRACTION POTENTIAL FIELD  
attr=1/2*k_att*(norm(test_point-over))^2; % compute the atraction potential

% The REPULSION POTENTIAL FIELD  
for i=1:size(obstacle,2)
    if norm(test_point-obstacle(:,i))<=detection
      %repu=repu+1/2*k_rep*(1/norm(curr-obstacle(:,i))-1/detection)^2; % original method --- compute the repulsion potential      
      repu=repu+1/2*k_rep*((1/norm(test_point-obstacle(:,i))-1/detection)^2)*(norm(test_point-over))^3; % improved method --- solution to unreachable target

    else
        repu=repu+0;
    end
end

%output that is the direction of the rover
output=attr+repu+ext_P;
end

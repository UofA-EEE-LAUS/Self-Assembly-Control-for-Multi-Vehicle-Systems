function [ output ] = apf_LM( test_point,over,obstacle )
%Compute function 

k_att=10; % gain factor of attractive force
repu=0;  % initilize the repulsion 
k_rep=100; % gain factor of the repulsive force 
c0_w = .3; c1_w = 0.27; c2_w = .3; c3_w = .1; c4_w = .7; c5_r = 0.94;
obplot = [c0_w,c1_w,c2_w,c3_w,c4_w,c5_r];
detection = max(obplot)+0.1; % detection=0.9;  % the obstacle avoidance reaction range, if the distance between obstacle and robot is smaller than the range, then repulsion affcet its movement, otherwise repulsion is 0
Ke=1;

% The extra POTENTIAL FIELD for Local Minima Avoidance
if norm(test_point-over)<=detection
   ext_P=(-1/2)*(Ke/detection)*(norm(test_point-over))^2;
   
else
    ext_P=(-Ke)*(norm(test_point-over)-detection/2);
    
end 

% The ATTRACTION POTENTIAL FIELD function 
attr=1/2*k_att*(norm(test_point-over))^2; % compute the atraction potential

% The REPULSION POTENTIAL FIELD function 
for i=1:size(obstacle,2)
    if norm(test_point-obstacle(:,i))<=detection
      %repu=repu+1/2*k_rep*(1/norm(curr-obstacle(:,i))-1/detection)^2; % original method --- compute the repulsion potential      
      repu=repu+1/2*k_rep*((1/norm(test_point-obstacle(:,i))-1/detection)^2)*(norm(test_point-over))^3; % improved method --- solution to unreachable target

    else
        repu=repu+0;
    end
end

output=attr+repu;

end
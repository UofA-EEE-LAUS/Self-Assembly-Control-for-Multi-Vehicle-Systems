function [Totalforce] = apfforce(begin,goal,obstacles,radius,range,katt,krep,kextra)
%% Initiallization
Frepu=0;
%% Attractive Force
Fattr=1/2*katt*((norm(begin-goal))^2); 
%% Reputation Force
for i=1:size(obstacles,2)
    if norm(begin-obstacles(:,i))<=radius
      Frepu_o=1/2*krep*((1/norm(begin-obstacles(:,i)))-(1/radius))^2;      
      Freou_i=Frepu_o*(norm(begin-goal))^3;
      Frepu=Frepu+Freou_i;
    else
        Frepu=Frepu+0;
    end
end
%% Solution Of Unreachable Target(Force Of Extra Potential Field)
if norm(begin-goal)<=range
   Fextra=(-1/2)*(kextra/range)*(norm(begin-goal))^2; 
else
    Fextra=(-kextra)*(norm(begin-goal)-(range/2));    
end 
%% Total Force
Totalforce=Fattr+Frepu+Fextra;
end
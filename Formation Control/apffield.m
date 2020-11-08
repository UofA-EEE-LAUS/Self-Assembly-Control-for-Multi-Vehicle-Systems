function [Totalfield] = apffield(begin,goal,obstacles,radius,extraradius,katt,krep,kextra)
%% Initiallization
Frepu=0;
%% AttractivePotential Field
Fattr=1/2*katt*((norm(begin-goal))^2); 
%% Reputation Potential Field
for i=1:size(obstacles,2)
    if norm(begin-obstacles(:,i))<=radius
      Frepu_o=1/2*krep*((1/norm(begin-obstacles(:,i)))-(1/radius))^2;      
      Freou_i=Frepu_o*(norm(begin-goal))^3;
      Frepu=Frepu+Freou_i;
    else
        Frepu=Frepu+0;
    end
end
%% Solution Of Unreachable Target(Extra Potential Field)
if norm(begin-goal)<=extraradius
   Fextra=(-1/2)*(kextra/extraradius)*(norm(begin-goal))^2; 
else
    Fextra=(-kextra)*(norm(begin-goal)-(extraradius/2));    
end 
%% Total Potential Field
Totalfield=Fattr+Frepu+Fextra;
end
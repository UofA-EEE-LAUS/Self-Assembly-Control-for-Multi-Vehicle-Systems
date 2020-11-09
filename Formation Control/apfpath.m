function [ position ] = apfpath(begin,goal,obstacles,radius,extraradius,katt,krep,kextra,iterations,directionnumber)
%% Initiallization
count=1;
point=begin;
%% Generate Path In n Direction
while (norm(point-goal)>0.2) && (count<=iterations) 
    storage=point;
    for i=1:8 
        nposition(:,i)=[radius*sin((i-1)*(2*pi/directionnumber))+storage(1);radius*cos((i-1)*(2*pi/directionnumber))+storage(2)];
        field(:,i)=apffield(nposition(:,i),goal,obstacles,radius,extraradius,katt,krep,kextra); 
    end
    [~, If]=min(field);
    point=(storage+nposition(:,If))/2; 
    %% Solution For Local Minimum
    localminimum(:,count)=point; 
    if count > 2
        cur=localminimum(:,count);
        pre=localminimum(:,count-2);
        if (norm(cur-pre)) < 0.01 
            lmpoint=cur; 
            %% Solution For Visual Obstacels          
            for j=1:directionnumber
                nposition_vo(:,j)=[radius*sin((j-1)*(2*pi/directionnumber))+storage(1);radius*cos((j-1)*(2*pi/directionnumber))+storage(2)];
                vobstacles=obstacles;
                size_vo=size(vobstacles,2);
                vobstacles=[vobstacles(:,1:size_vo) lmpoint];
                field_vo(:,j)=apffield(nposition_vo(:,j),goal,vobstacles,extraradius,katt,krep,kextra);
            end
            [~, If_vo]=min(field_vo);
            point=(storage+nposition_vo(:,If_vo))/2;
        else
        end
    else
    end
    position(:,count)=point;
    count=count+1;
end
if count==1
    position(:,count)=point;
end
end

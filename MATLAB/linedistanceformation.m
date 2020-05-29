function [xr0,yr0,xr1,yr1,xr2,yr2]=linedistanceformation(x0,y0,d01,d02,theta01,theta02)
for xr0=x0
    for yr0=y0
        if theta01~=0
            theta01=0;
            if  d01*cos(theta01)~=0.35
                d01=0.35;
                xr1=xr0+d01*cos(theta01);
                yr1=yr0+d01*sin(theta01);
            end
        end
        if theta02~=0
            theta02=0;
            if d02*cos(theta02)~=0.7
                d02=0.7;
                xr2=xr0+d02*cos(theta02);
                yr2=yr0+d02*sin(theta02);
            end
        end
    end
end

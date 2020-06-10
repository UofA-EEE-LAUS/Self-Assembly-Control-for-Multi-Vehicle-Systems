function [xr0,yr0,xr1,yr1,xr2,yr2]=linedistanceformation(dx,dy,x1,y1,x2,y2)
for xr0=dx
    for yr0=dy
     xr0=x0
     yr0=y0
     d01=sqrt((abs(xr0-x1)^2)+(abs(yr0-y1)^2));
     theta01=atan(abs(yr0-y1)/abs(xr0-x1))*(180/pi);
        if theta01~=0
            theta01=0;
            if  d01*cos(theta01)~=0.32
                d01=0.32;
                xr1=xr0+d01*cos(theta01);
                yr1=yr0+d01*sin(theta01);
                d02=sqrt((abs(xr1-x2)^2)+(abs(yr1-y2)^2));
                theta02=atan(abs(yr1-y2)/abs(xr1-x2))*(180/pi);
            end
        end
        if theta02~=0
            theta02=0;
            if d02*cos(theta02)~=0.32
                d02=0.32;
                xr2=xr1+d02*cos(theta02);
                yr2=yr1+d02*sin(theta02);
            end
        end
    end
end

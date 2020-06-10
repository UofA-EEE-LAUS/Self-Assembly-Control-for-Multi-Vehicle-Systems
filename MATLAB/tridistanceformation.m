function [xr0,yr0,xr1,yr1,xr2,yr2]=tridistanceformation(x0,y0,x1,y1,x2,y2,d)
for xr0=x0
    for yr0=y0
        d01=sqrt((abs(x0-x1)^2)+(abs(y0-y1)^2));
        d02=sqrt((abs(x0-x2)^2)+(abs(y0-y2)^2));
        theta01=atan(abs(y0-y1)/abs(x0-x1))*(180/pi);
        theta02=atan(abs(y0-y2)/abs(x0-x2))*(180/pi);
        if theta01~=150
            theta01=150;
            if  d01~=d
                d01=d;
                xr1=x0+(d01*cos(theta01));
                yr1=y0+(d01*sin(theta01));
            end
        end
        if theta02~=120
            theta02=120;
            if d02~=d
                d02=d;
                xr2=x0+(d02*cos(theta02));
                yr2=y0+(d02*sin(theta02));
            end
        end
    end
end

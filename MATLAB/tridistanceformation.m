function [xr0,yr0,xr1,yr1,xr2,yr2]=tridistanceformation(x0,y0,d01,d02,theta01,theta02)
for xr0=x0
    for yr0=y0
        if theta01~=120
            theta01=120;
            if  d01~=0.32
                d01=0.32;
                xr1=x0+(d01*cos(theta01));
                yr1=y0+(d01*sin(theta01));
            end
        end
        if theta02~=150
            theta02=150;
            if d02~=0.32
                d02=0.32;
                xr2=x0+(d02*cos(theta02));
                yr2=y0+(d02*sin(theta02));
            end
        end
    end
end
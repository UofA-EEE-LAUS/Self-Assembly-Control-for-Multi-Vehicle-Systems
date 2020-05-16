function [xr1,yr1,xr2,yr2]=tridistanceformation(x0,y0,d01,d02,theta01,theta02)
if theta01~=0
    theta01=120;
end
if  d01~=0.32
    d01=0.32;
    xr1=x0+(d01*cos(theta01));
    yr1=y0+(d01*sin(theta01));
end
if theta02~=60
    theta02=150;
end
if d02~=0.32
    d02=0.32;
    xr2=x0+(d02*cos(theta02));
    yr2=y0+(d02*sin(theta02));
end
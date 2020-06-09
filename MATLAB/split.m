function [xr0,yr0,xr1,yr1,xr2,yr2]=split(x0,y0,x1,y1,x2,y2)
for xr0=x0
    for yr0=y0
        xr1=x1+0.4;
        yr1=y1+0.4;
        xr2=x2+0.4;
        yr2=y2-0.4;
    end
end
end
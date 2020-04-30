function [xr0,yr0,xr1,yr1,xr2,yr2]=lineformation(x0,y0,x1,y1,x2,y2)
xr0=x0;
yr0=y0;
  for y01=abs(y0-y1)
  if y01~=0
      yr1=y0;
  else
      yr1=y1;
  end
  end 
  for y02=abs(y0-y2)
      if y02~=0
          yr2=y0;
      else
          yr2=y2;
      end
  end
 for x01=abs(x0-x1)
     if x01~=1
         xr1=x0+1;
     else
         xr1=x1;
     end
 end
 for x02=abs(x0-x2)
     if x02~=2
         xr2=x0+2;
     else
         xr2=x2;
     end
 end
end
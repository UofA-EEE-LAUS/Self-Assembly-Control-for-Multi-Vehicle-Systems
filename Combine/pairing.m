function [xp0,yp0,yawp0,xp1,yp1,yawp1,xp2,yp2,yawp2,resultcostMat,resultcost,resultassignment]=pairing(num,x0,y0,x1,y1,x2,y2,xg0,yg0,yawg0,xg1,yg1,yawg1,xg2,yg2,yawg2)
resultcostMat=zeros([3 3*num]);
resultcost=zeros([1 num]);
resultassignment=zeros([3 3*num]);
xp0=zeros([1 num]);
yp0=zeros([1 num]);
yawp0=zeros([1 num]);
xp1=zeros([1 num]);
yp1=zeros([1 num]);
yawp1=zeros([1 num]);
xp2=zeros([1 num]);
yp2=zeros([1 num]);
yawp2=zeros([1 num]);
for i=1:num
    if i==1
        ux0(i)=x0;
        uy0(i)=y0;
        ux1(i)=x1;
        uy1(i)=y1;
        ux2(i)=x2;
        uy2(i)=y2;
        vx0(i)=xg0(i);
        vy0(i)=yg0(i);
        vx1(i)=xg1(i);
        vy1(i)=yg1(i);
        vx2(i)=xg2(i);
        vy2(i)=yg2(i);
        d00(i)=dis(ux0(i),uy0(i),vx0(i),vy0(i));
        d01(i)=dis(ux0(i),uy0(i),vx1(i),vy1(i));
        d02(i)=dis(ux0(i),uy0(i),vx2(i),vy2(i));
        d10(i)=dis(ux1(i),uy1(i),vx0(i),vy0(i));
        d11(i)=dis(ux1(i),uy1(i),vx1(i),vy1(i));
        d12(i)=dis(ux1(i),uy1(i),vx2(i),vy2(i));
        d20(i)=dis(ux2(i),uy2(i),vx0(i),vy0(i));
        d21(i)=dis(ux2(i),uy2(i),vx1(i),vy1(i));
        d22(i)=dis(ux2(i),uy2(i),vx2(i),vy2(i));
        resultcostMat(1:end,((i*3)-2):i*3)=[d00(i) d01(i) d02(i);d10(i) d11(i) d12(i);d20(i) d21(i) d22(i)];
        costMat=resultcostMat(1:end,((i*3)-2):i*3);
        [assignment,cost] = munkres(costMat);
        resultassignment(1:end,((i*3)-2):i*3)=assignment;
        resultcost(i)=cost;
        if resultassignment(1,3*i-2)==1&&resultassignment(1,3*i-1)==0&&resultassignment(1,3*i)==0
            xp0(i)=xg0(i);
            yp0(i)=yg0(i);
            yawp0(i)=yawg0(i);
        elseif resultassignment(1,3*i-2)==0&&resultassignment(1,3*i-1)==1&&resultassignment(1,3*i)==0
            xp0(i)=xg1(i);
            yp0(i)=yg1(i);
            yawp0(i)=yawg1(i);
        elseif resultassignment(1,3*i-2)==0&&resultassignment(1,3*i-1)==0&&resultassignment(1,3*i)==1
            xp0(i)=xg2(i);
            yp0(i)=yg2(i);
            yawp0(i)=yawg2(i);
        end
        if resultassignment(2,3*i-2)==1&&resultassignment(2,3*i-1)==0&&resultassignment(2,3*i)==0
            xp1(i)=xg0(i);
            yp1(i)=yg0(i);
            yawp1(i)=yawg0(i);
        elseif resultassignment(2,3*i-2)==0&&resultassignment(2,3*i-1)==1&&resultassignment(2,3*i)==0
            xp1(i)=xg1(i);
            yp1(i)=yg1(i);
            yawp1(i)=yawg1(i);
        elseif resultassignment(2,3*i-2)==0&&resultassignment(2,3*i-1)==0&&resultassignment(2,3*i)==1
            xp1(i)=xg2(i);
            yp1(i)=yg2(i);
            yawp1(i)=yawg2(i);
        end
        if resultassignment(3,3*i-2)==1&&resultassignment(3,3*i-1)==0&&resultassignment(3,3*i)==0
            xp2(i)=xg0(i);
            yp2(i)=yg0(i);
            yawp2(i)=yawg0(i);
        elseif resultassignment(3,3*i-2)==0&&resultassignment(3,3*i-1)==1&&resultassignment(3,3*i)==0
            xp2(i)=xg1(i);
            yp2(i)=yg1(i);
            yawp2(i)=yawg1(i);
        elseif resultassignment(3,3*i-2)==0&&resultassignment(3,3*i-1)==0&&resultassignment(3,3*i)==1
            xp2(i)=xg2(i);
            yp2(i)=yg2(i);
            yawp2(i)=yawg2(i);
        end
    else
        ux0(i)=xp0(i-1);
        uy0(i)=yp0(i-1);
        ux1(i)=xp1(i-1);
        uy1(i)=yp1(i-1);
        ux2(i)=xp2(i-1);
        uy2(i)=yp2(i-1);
        vx0(i)=xg0(i);
        vy0(i)=yg0(i);
        vx1(i)=xg1(i);
        vy1(i)=yg1(i);
        vx2(i)=xg2(i);
        vy2(i)=yg2(i);
        d00(i)=dis(ux0(i),uy0(i),vx0(i),vy0(i));
        d01(i)=dis(ux0(i),uy0(i),vx1(i),vy1(i));
        d02(i)=dis(ux0(i),uy0(i),vx2(i),vy2(i));
        d10(i)=dis(ux1(i),uy1(i),vx0(i),vy0(i));
        d11(i)=dis(ux1(i),uy1(i),vx1(i),vy1(i));
        d12(i)=dis(ux1(i),uy1(i),vx2(i),vy2(i));
        d20(i)=dis(ux2(i),uy2(i),vx0(i),vy0(i));
        d21(i)=dis(ux2(i),uy2(i),vx1(i),vy1(i));
        d22(i)=dis(ux2(i),uy2(i),vx2(i),vy2(i));
        resultcostMat(1:end,((i*3)-2):i*3)=[d00(i) d01(i) d02(i);d10(i) d11(i) d12(i);d20(i) d21(i) d22(i)];
        costMat=resultcostMat(1:end,((i*3)-2):i*3);
        [assignment,cost] = munkres(costMat);
        resultassignment(1:end,((i*3)-2):i*3)=assignment;
        resultcost(i)=cost;
        if resultassignment(1,3*i-2)==1&&resultassignment(1,3*i-1)==0&&resultassignment(1,3*i)==0
            xp0(i)=xg0(i);
            yp0(i)=yg0(i);
            yawp0(i)=yawg0(i);
        elseif resultassignment(1,3*i-2)==0&&resultassignment(1,3*i-1)==1&&resultassignment(1,3*i)==0
            xp0(i)=xg1(i);
            yp0(i)=yg1(i);
            yawp0(i)=yawg1(i);
        elseif resultassignment(1,3*i-2)==0&&resultassignment(1,3*i-1)==0&&resultassignment(1,3*i)==1
            xp0(i)=xg2(i);
            yp0(i)=yg2(i);
            yawp0(i)=yawg2(i);
        end
        if resultassignment(2,3*i-2)==1&&resultassignment(2,3*i-1)==0&&resultassignment(2,3*i)==0
            xp1(i)=xg0(i);
            yp1(i)=yg0(i);
            yawp1(i)=yawg0(i);
        elseif resultassignment(2,3*i-2)==0&&resultassignment(2,3*i-1)==1&&resultassignment(2,3*i)==0
            xp1(i)=xg1(i);
            yp1(i)=yg1(i);
            yawp1(i)=yawg1(i);
        elseif resultassignment(2,3*i-2)==0&&resultassignment(2,3*i-1)==0&&resultassignment(2,3*i)==1
            xp1(i)=xg2(i);
            yp1(i)=yg2(i);
            yawp1(i)=yawg2(i);
        end
        if resultassignment(3,3*i-2)==1&&resultassignment(3,3*i-1)==0&&resultassignment(3,3*i)==0
            xp2(i)=xg0(i);
            yp2(i)=yg0(i);
            yawp2(i)=yawg0(i);
        elseif resultassignment(3,3*i-2)==0&&resultassignment(3,3*i-1)==1&&resultassignment(3,3*i)==0
            xp2(i)=xg1(i);
            yp2(i)=yg1(i);
            yawp2(i)=yawg1(i);
        elseif resultassignment(3,3*i-2)==0&&resultassignment(3,3*i-1)==0&&resultassignment(3,3*i)==1
            xp2(i)=xg2(i);
            yp2(i)=yg2(i);
            yawp2(i)=yawg2(i);
        end
    end
    
end

figure
plot(xp0,yp0,'r*')
hold on
axis equal
plot(xp1,yp1,'g*')
hold on
axis equal
plot(xp2,yp2,'b*')
hold on
axis equal
C0_pos = [ 0.35, -1.075  ];
C1_pos = [ -0.145, -1.045 ];
%obstacles size
c0_w = 0.1; c1_w = 0.1;
%positions setting
obs0_x=C0_pos(:,1);    obs0_y=C0_pos(:,2);
obs1_x=C1_pos(:,1);    obs1_y=C1_pos(:,2);
%obstacles centre x and y coordinates
obs_pos = [[obs0_x obs0_y];[obs1_x obs1_y]];
%increase value for obs plotting
obplot = [c0_w,c1_w];
hold on
axis equal
% obstacle 1
ce = obs_pos(1,:);
w = obplot(1);
p1 = [(ce(1)) - (w / 2), (ce(2)) - (w / 2)];
p2 = [(ce(1)) + (w / 2), (ce(2)) - (w / 2)];
p3 = [(ce(1)) + (w / 2), (ce(2)) + (w / 2)];
p4 = [(ce(1)) - (w / 2), (ce(2)) + (w / 2)];
X=[p1(1),p2(1),p3(1),p4(1),p1(1)];
Y=[p1(2),p2(2),p3(2),p4(2),p1(2)];
fill(X,Y,'y')

% obstacle 2
ce = obs_pos(2,:);
w = obplot(2);
p1 = [(ce(1)) - (w / 2), (ce(2)) - (w / 2)];
p2 = [(ce(1)) + (w / 2), (ce(2)) - (w / 2)];
p3 = [(ce(1)) + (w / 2), (ce(2)) + (w / 2)];
p4 = [(ce(1)) - (w / 2), (ce(2)) + (w / 2)];
X=[p1(1),p2(1),p3(1),p4(1),p1(1)];
Y=[p1(2),p2(2),p3(2),p4(2),p1(2)];
fill(X,Y,'y')

hold on
for i=1:num
    plot([xp0(i) xp1(i) xp2(i) xp0(i)],[yp0(i) yp1(i) yp2(i) yp0(i)],'m')
end
hold on
axis equal
plot(xp0,yp0,'r-');
plot(xp1,yp1,'g--');
plot(xp2,yp2,'b:');
hold off

legend('Rover0','Rover1','Rover2','Ob1','Ob2')
title('Formation Points With Assignment Method')
xlabel('X')
ylabel('Y')
end
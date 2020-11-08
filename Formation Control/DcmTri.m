function [xt0,yt0,yawt0,xt1,yt1,yawt1,xt2,yt2,yawt2,angle] = DcmTri(x0,y0,dx,dy,num,rd,offset)

positiont0=zeros([3 num]);
positiont1=zeros([3 num]);
positiont2=zeros([3 num]);
trans=180/pi;

for i=1:num
    if i==1
        diffx=dx(i)-x0;
        diffy=dy(i)-y0;
        dphi=atan(diffy/diffx);
        if dx(i)>=x0
            if dphi>=0
                dtyaw(i)=3*pi/2-dphi;
            elseif dphi<0
                dtyaw(i)=pi/2-dphi;
            end
        elseif dx(i)<x0
            if dphi<0
                dtyaw(i)=pi/2-dphi;
            elseif dphi>=0
                dtyaw(i)=3*pi/2-dphi;
            end
        end
        
        
        Dcm=[cos(dtyaw(i)) sin(dtyaw(i))
            -sin(dtyaw(i)) cos(dtyaw(i))];
        r0=inv(Dcm)*([dx(i) dy(i)]');
        r1=[r0(1,:)-rd*cos(pi/6);r0(2,:)+rd*sin(pi/6)];
        r2=[r0(1,:)+rd*cos(pi/6);r0(2,:)+rd*sin(pi/6)];
        positionr1=Dcm*r1;
        positionr2=Dcm*r2;
        positiont0(1,i)=dx(i);
        positiont0(2,i)=dy(i);
        positiont1(1,i)=(positionr1(1,:))';
        positiont1(2,i)=(positionr1(2,:))';
        positiont2(1,i)=(positionr2(1,:))';
        positiont2(2,i)=(positionr2(2,:))';
        if dx(i)>=x0
            angle(i)=-(dtyaw(i)-offset)*trans;
        elseif dx(i)<x0
            angle(i)=(dtyaw(i)-offset)*trans;
        end
        if angle(i)<-90
            orientation=360+angle(i);
            if orientation>180
                orientation=360-orientation;
            end
        elseif angle(i)>180
            orientation=360-angle(i);
        elseif angle(i)>=-90 && angle(i)<=180
            orientation=angle(i);
        end
        positiont0(3,i)=orientation;
        positiont1(3,i)=orientation;
        positiont2(3,i)=orientation;
        
    else
        diffx=dx(i)-dx(i-1);
        diffy=dy(i)-dy(i-1);
        dphi=atan(diffy/diffx);
        
        %adjust rotation angle
        if dy(i)>0&&dy(i-1)>0&&dy(i)<dy(i-1)
            dtyaw(i)=pi/2-dphi;
        elseif dx(i)>=0&&dx(i)<dx(i-1)&&dy(i)<0&&dy(i)<=dy(i-1)&&dy(i-1)>=0
            dtyaw(i)=pi/2-dphi;
        elseif dx(i)<=0&&dx(i)>=dx(i-1)&&dy(i)<=dy(i-1)
            dtyaw(i)=3*pi/2-dphi;
        elseif dx(i)>0&&dx(i-1)>0&&dx(i)>=dx(i-1)&&dy(i)<0&&dy(i)<=dy(i-1)
            dtyaw(i)=pi/2-dphi;
        elseif dx(i)>0&&dx(i-1)>0&&dx(i)<dx(i-1)&&dy(i)>dy(i-1)
            dtyaw(i)=pi/2-dphi;
        elseif dx(i)>=0&&dx(i-1)<0&&dy(i)<0&&dy(i-1)<0&&dy(i)>dy(i-1)
            dtyaw(i)=pi/2-dphi;
        elseif dx(i-1)<dx(i)&&dx(i-1)<0&&dx(i)<0&&dy(i)<0&&dy(i-1)<0&&dy(i)<dy(i-1)
            dtyaw(i)=3*pi/2-dphi;
        elseif dx(i-1)>=0&&dx(i)>=0&&dx(i)<=dx(i-1)&&dy(i)<=0&&dy(i-1)>0
            if dphi>=0
                dtyaw(i)=3*pi/2-dphi;
            elseif dphi<0
                dtyaw(i)=pi/2-dphi;
            end
        elseif dy(i)<0&&dy(i-1)<0&&dy(i)>dy(i-1)
            dtyaw(i)=3*pi/2-dphi;
        elseif dx(i-1)<0&&dx(i)<0&&dx(i)>dx(i-1)
            dtyaw(i)=pi/2-dphi;
        elseif dx(i)>=0&&dy(i)<=0
            dtyaw(i)=3*pi/2-dphi;
        else
            if dphi>=0
                dtyaw(i)=pi/2-dphi;
            elseif dphi<0
                dtyaw(i)=3*pi/2-dphi;
            end
        end
        
        
        Dcm=[cos(dtyaw(i)) sin(dtyaw(i))
            -sin(dtyaw(i)) cos(dtyaw(i))];
        r0=inv(Dcm)*([dx(i) dy(i)]');
        r1=[r0(1,:)-rd*cos(pi/6);r0(2,:)+rd*sin(pi/6)];
        r2=[r0(1,:)+rd*cos(pi/6);r0(2,:)+rd*sin(pi/6)];
        positionr1=Dcm*r1;
        positionr2=Dcm*r2;
        positiont0(1,i)=dx(i);
        positiont0(2,i)=dy(i);
        positiont1(1,i)=(positionr1(1,:))';
        positiont1(2,i)=(positionr1(2,:))';
        positiont2(1,i)=(positionr2(1,:))';
        positiont2(2,i)=(positionr2(2,:))';
        if dx(i)>=dx(i-1)
            angle(i)=-(dtyaw(i)-offset)*trans;
        elseif dx(i)<dx(i-1)
            angle(i)=(dtyaw(i)-offset)*trans;
        end
        if angle(i)<-90
            orientation=360+angle(i);
            if orientation>180
                orientation=360-orientation;
            end
        elseif angle(i)>180
            orientation=360-angle(i);
        elseif angle(i)>=-90 && angle(i)<=180
            orientation=angle(i);
        end
        positiont0(3,i)=orientation;
        positiont1(3,i)=orientation;
        positiont2(3,i)=orientation;
    end
end

xt0=positiont0(1,:);
yt0=positiont0(2,:);
xt1=positiont1(1,:);
yt1=positiont1(2,:);
xt2=positiont2(1,:);
yt2=positiont2(2,:);
yawt0=positiont0(3,:);
yawt1=positiont1(3,:);
yawt2=positiont2(3,:);

figure
plot(xt0,yt0,'r*')
hold on
axis equal
plot(xt1,yt1,'g*')
hold on
axis equal
plot(xt2,yt2,'b*')
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
    plot([xt0(i) xt1(i) xt2(i) xt0(i)],[yt0(i) yt1(i) yt2(i) yt0(i)],'m')
end
plot(xt0,yt0,'r-');
plot(xt1,yt1,'g--');
plot(xt2,yt2,'b:');
hold off

legend('Rover0','Rover1','Rover2','Ob1','Ob2')
hold on
title('Triangle Formation Points With Rotation Matrix')
xlabel('X')
ylabel('Y')
end

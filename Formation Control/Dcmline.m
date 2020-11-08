function [xl0,yl0,yawl0,xl1,yl1,yawl1,xl2,yl2,yawl2,angle] = Dcmline(x0,y0,dx,dy,num,rd,offset)

position0=zeros([3 num]);
position1=zeros([3 num]);
position2=zeros([3 num]);
trans=180/pi;

for i=1:num
    if i==1
        diffx=dx(i)-x0;
        diffy=dy(i)-y0;
        dphi=atan(diffy/diffx);
        if dx(i)>=x0
            if dphi>=0
                dlyaw(i)=3*pi/2-dphi;
            elseif dphi<0
                dlyaw(i)=pi/2-dphi;
            end
        elseif dx(i)<x0
            if dphi<0
                dlyaw(i)=pi/2-dphi;
            elseif dphi>=0
                dlyaw(i)=3*pi/2-dphi;
            end
        end
        Dcm=[cos(dlyaw(i)) sin(dlyaw(i))
            -sin(dlyaw(i)) cos(dlyaw(i))];
        r0=inv(Dcm)*([dx(i) dy(i)]');
        r1=[r0(1,:)-rd;r0(2,:)];
        r2=[r0(1,:)-2*rd;r0(2,:)];
        positionr1=Dcm*r1;
        positionr2=Dcm*r2;
        position0(1,i)=dx(i);
        position0(2,i)=dy(i);
        position1(1,i)=(positionr1(1,:))';
        position1(2,i)=(positionr1(2,:))';
        position2(1,i)=(positionr2(1,:))';
        position2(2,i)=(positionr2(2,:))';
        if dx(i)>=x0
            angle(i)=-(dlyaw(i)-offset)*trans;
        elseif dx(i)<x0
            angle(i)=(dlyaw(i)-offset)*trans;
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
        position0(3,i)=orientation;
        position1(3,i)=orientation;
        position2(3,i)=orientation;
        
    elseif i>1&&i<=num
        diffx=dx(i)-dx(i-1);
        diffy=dy(i)-dy(i-1);
        dphi=atan(diffy/diffx);
        
        %adjust rotation angle
        if dy(i)>0&&dy(i-1)>0&&dy(i)<dy(i-1)
            dlyaw(i)=pi/2-dphi;
        elseif dx(i)>=0&&dx(i)<dx(i-1)&&dy(i)<0&&dy(i)<=dy(i-1)&&dy(i-1)>=0
            dlyaw(i)=pi/2-dphi;
        elseif dx(i)<=0&&dx(i)>=dx(i-1)&&dy(i)<=dy(i-1)
            dlyaw(i)=3*pi/2-dphi;
        elseif dx(i)>0&&dx(i-1)>0&&dx(i)>=dx(i-1)&&dy(i)<0&&dy(i)<=dy(i-1)
            dlyaw(i)=pi/2-dphi;
        elseif dx(i)>0&&dx(i-1)>0&&dx(i)<dx(i-1)&&dy(i)>dy(i-1)
            dlyaw(i)=pi/2-dphi;
        elseif dx(i)>=0&&dx(i-1)<0&&dy(i)<0&&dy(i-1)<0&&dy(i)>dy(i-1)
            dlyaw(i)=pi/2-dphi;
        elseif dx(i-1)<dx(i)&&dx(i-1)<0&&dx(i)<0&&dy(i)<0&&dy(i-1)<0&&dy(i)<dy(i-1)
            dlyaw(i)=3*pi/2-dphi;
        elseif dx(i-1)>=0&&dx(i)>=0&&dx(i)<=dx(i-1)&&dy(i)<=0&&dy(i-1)>0
            if dphi>=0
                dlyaw(i)=3*pi/2-dphi;
            elseif dphi<0
                dlyaw(i)=pi/2-dphi;
            end
        elseif dy(i)<0&&dy(i-1)<0&&dy(i)>dy(i-1)
            dlyaw(i)=3*pi/2-dphi;
        elseif dx(i-1)<0&&dx(i)<0&&dx(i)>dx(i-1)
            dlyaw(i)=pi/2-dphi;
        elseif dx(i)>=0&&dy(i)<=0
            dlyaw(i)=3*pi/2-dphi;
        else
            if dphi>=0
                dlyaw(i)=pi/2-dphi;
            elseif dphi<0
                dlyaw(i)=3*pi/2-dphi;
            end
        end
        
        Dcm=[cos(dlyaw(i)) sin(dlyaw(i))
            -sin(dlyaw(i)) cos(dlyaw(i))];
        r0=inv(Dcm)*([dx(i) dy(i)]');
        r1=[r0(1,:)-rd;r0(2,:)];
        r2=[r0(1,:)-2*rd;r0(2,:)];
        positionr1=Dcm*r1;
        positionr2=Dcm*r2;
        position0(1,i)=dx(i);
        position0(2,i)=dy(i);
        position1(1,i)=(positionr1(1,:))';
        position1(2,i)=(positionr1(2,:))';
        position2(1,i)=(positionr2(1,:))';
        position2(2,i)=(positionr2(2,:))';
        if dx(i)>=dx(i-1)
            angle(i)=-(dlyaw(i)-offset)*trans;
        elseif dx(i)<dx(i-1)
            angle(i)=(dlyaw(i)-offset)*trans;
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
        position0(3,i)=orientation;
        position1(3,i)=orientation;
        position2(3,i)=orientation;
    end
    
    xl0=position0(1,:);
    yl0=position0(2,:);
    yawl0=position0(3,:);
    xl1=position1(1,:);
    yl1=position1(2,:);
    yawl1=position1(3,:);
    xl2=position2(1,:);
    yl2=position2(2,:);
    yawl2=position2(3,:);
end
figure
plot(xl0,yl0,'r*')
hold on
axis equal
plot(xl1,yl1,'g*')
hold on
axis equal
plot(xl2,yl2,'b*')
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
    plot([xl0(i) xl1(i) xl2(i)],[yl0(i) yl1(i) yl2(i)],'m')
end
plot(xl0,yl0,'r-');
plot(xl1,yl1,'g--');
plot(xl2,yl2,'b:');
hold off

legend('Rover0','Rover1','Rover2','Ob1','Ob2')
title('Line Formation Points With Rotation Matrix')
xlabel('X')
ylabel('Y')

end
% function [xl0,yl0,yawl0,xl1,yl1,yawl1,xl2,yl2,yawl2] = Dcmline(dx,dy,num,rd)

position0=zeros([3 num]);
position1=zeros([3 num]);
position2=zeros([3 num]);

for i=1:num
    
    if i==1
        position0(1,i)=dx(i);
        position0(2,i)=dy(i);
        position1(1,i)=dx(i);
        position1(2,i)=dy(i)-rd;
        position2(1,i)=dx(i);
        position2(2,i)=dy(i)-2*rd;
        position0(3,i)=pi/2-offset;
        position1(3,i)=pi/2-offset;
        position2(3,i)=pi/2-offset;
    else
        diffx=dx(i)-dx(i-1);
        diffy=dy(i)-dy(i-1);
        dphi=atan(diffy/diffx);
       
 
            dyaw=pi/2-dphi-offset;

        
        Dcm=[cos(dyaw) sin(dyaw)
             -sin(dyaw) cos(dyaw)];
        r0=inv(Dcm)*([dx(i) dy(i)]');
        r1=[r0(1,:);r0(2,:)-rd];
        r2=[r0(1,:);r0(2,:)-2*rd];
        positionr1=Dcm*r1;
        positionr2=Dcm*r2;
        position0(1,i)=dx(i);
        position0(2,i)=dy(i);
        position1(1,i)=(positionr1(1,:))';
        position1(2,i)=(positionr1(2,:))';
        position2(1,i)=(positionr2(1,:))';
        position2(2,i)=(positionr2(2,:))';
        position0(3,i)=dyaw;
        position1(3,i)=dyaw;
        position2(3,i)=dyaw;
    end
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

figure
for i=1:num
    
    plot(dx,dy,'r*')
    hold on
    axis equal
    plot(xl0,yl0,'r--')
    hold on
    axis equal
    plot(xl1,yl1,'g--')
    hold on
    axis equal
    plot(xl1,yl1,'g*')
    hold on
    axis equal
    plot(xl2,yl2,'b--')
    hold on
    axis equal
    plot(xl2,yl2,'b*')
    hold on
    axis equal
    plot([xl0(i) xl1(i) xl2(i)],[yl0(i) yl1(i) yl2(i)],'m')
    hold on
    axis equal
    title('DCM Line Formation Points')
    xlabel('X')
    ylabel('Y')
end
% end
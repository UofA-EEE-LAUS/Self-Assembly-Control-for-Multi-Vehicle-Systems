%clear();
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
%%
if (clientID>-1)
    disp('Connected')
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);%start Vrep simulation
    %% get rover
    [returnCode,rover0]=vrep.simxGetObjectHandle(clientID,strcat('rover#1'),vrep.simx_opmode_blocking);
    [returnCode,position0]=vrep.simxGetObjectPosition(clientID,rover0,-1,vrep.simx_opmode_blocking);
    [returnCode,rover1]=vrep.simxGetObjectHandle(clientID,strcat('rover#2'),vrep.simx_opmode_blocking);
    [returnCode,position1]=vrep.simxGetObjectPosition(clientID,rover1,-1,vrep.simx_opmode_blocking);
    [returnCode,rover2]=vrep.simxGetObjectHandle(clientID,strcat('rover#3'),vrep.simx_opmode_blocking);
    [returnCode,position2]=vrep.simxGetObjectPosition(clientID,rover2,-1,vrep.simx_opmode_blocking);
    r0=zeros([2 num+1]);
    r1=zeros([2 num+1]);
    r2=zeros([2 num+1]);
    r0(1,1)=position0(:,1);
    r0(2,1)=position0(:,2);
    r1(1,1)=position1(:,1);
    r1(2,1)=position1(:,2);
    r2(1,1)=position2(:,1);
    r2(2,1)=position2(:,2);
%% Pairing
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
        ux0(i)=position0(:,1);
        uy0(i)=position0(:,2);
        ux1(i)=position1(:,1);
        uy1(i)=position1(:,2);
        ux2(i)=position2(:,1);
        uy2(i)=position2(:,2);
        vx0(i)=xl0(i);
        vy0(i)=yl0(i);
        vx1(i)=xl1(i);
        vy1(i)=yl1(i);
        vx2(i)=xl2(i);
        vy2(i)=yl2(i);
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
            xp0(i)=xl0(i);
            yp0(i)=yl0(i);
            yawp0(i)=yawl0(i);
        elseif resultassignment(1,3*i-2)==0&&resultassignment(1,3*i-1)==1&&resultassignment(1,3*i)==0
            xp0(i)=xl1(i);
            yp0(i)=yl1(i);
            yawp0(i)=yawl1(i);
        elseif resultassignment(1,3*i-2)==0&&resultassignment(1,3*i-1)==0&&resultassignment(1,3*i)==1
            xp0(i)=xl2(i);
            yp0(i)=yl2(i);
            yawp0(i)=yawl2(i);
        end
        if resultassignment(2,3*i-2)==1&&resultassignment(2,3*i-1)==0&&resultassignment(2,3*i)==0
            xp1(i)=xl0(i);
            yp1(i)=yl0(i);
            yawp1(i)=yawl0(i);
        elseif resultassignment(2,3*i-2)==0&&resultassignment(2,3*i-1)==1&&resultassignment(2,3*i)==0
            xp1(i)=xl1(i);
            yp1(i)=yl1(i);
            yawp1(i)=yawl1(i);
        elseif resultassignment(2,3*i-2)==0&&resultassignment(2,3*i-1)==0&&resultassignment(2,3*i)==1
            xp1(i)=xl2(i);
            yp1(i)=yl2(i);
            yawp1(i)=yawl2(i);
        end
        if resultassignment(3,3*i-2)==1&&resultassignment(3,3*i-1)==0&&resultassignment(3,3*i)==0
            xp2(i)=xl0(i);
            yp2(i)=yl0(i);
            yawp2(i)=yawl0(i);
        elseif resultassignment(3,3*i-2)==0&&resultassignment(3,3*i-1)==1&&resultassignment(3,3*i)==0
            xp2(i)=xl1(i);
            yp2(i)=yl1(i);
            yawp2(i)=yawl1(i);
        elseif resultassignment(3,3*i-2)==0&&resultassignment(3,3*i-1)==0&&resultassignment(3,3*i)==1
            xp2(i)=xl2(i);
            yp2(i)=yl2(i);
            yawp2(i)=yawl2(i);
        end
    else
        ux0(i)=xp0(i-1);
        uy0(i)=yp0(i-1);
        ux1(i)=xp1(i-1);
        uy1(i)=yp1(i-1);
        ux2(i)=xp2(i-1);
        uy2(i)=yp2(i-1);
        vx0(i)=xl0(i);
        vy0(i)=yl0(i);
        vx1(i)=xl1(i);
        vy1(i)=yl1(i);
        vx2(i)=xl2(i);
        vy2(i)=yl2(i);
    end
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
        xp0(i)=xl0(i);
        yp0(i)=yl0(i);
        yawp0(i)=yawl0(i);
    elseif resultassignment(1,3*i-2)==0&&resultassignment(1,3*i-1)==1&&resultassignment(1,3*i)==0
        xp0(i)=xl1(i);
        yp0(i)=yl1(i);
        yawp0(i)=yawl1(i);
    elseif resultassignment(1,3*i-2)==0&&resultassignment(1,3*i-1)==0&&resultassignment(1,3*i)==1
        xp0(i)=xl2(i);
        yp0(i)=yl2(i);
        yawp0(i)=yawl2(i);
    end
    if resultassignment(2,3*i-2)==1&&resultassignment(2,3*i-1)==0&&resultassignment(2,3*i)==0
        xp1(i)=xl0(i);
        yp1(i)=yl0(i);
        yawp1(i)=yawl0(i);
    elseif resultassignment(2,3*i-2)==0&&resultassignment(2,3*i-1)==1&&resultassignment(2,3*i)==0
        xp1(i)=xl1(i);
        yp1(i)=yl1(i);
        yawp1(i)=yawl1(i);
    elseif resultassignment(2,3*i-2)==0&&resultassignment(2,3*i-1)==0&&resultassignment(2,3*i)==1
        xp1(i)=xl2(i);
        yp1(i)=yl2(i);
        yawp1(i)=yawl2(i);
    end
    if resultassignment(3,3*i-2)==1&&resultassignment(3,3*i-1)==0&&resultassignment(3,3*i)==0
        xp2(i)=xl0(i);
        yp2(i)=yl0(i);
        yawp2(i)=yawl0(i);
    elseif resultassignment(3,3*i-2)==0&&resultassignment(3,3*i-1)==1&&resultassignment(3,3*i)==0
        xp2(i)=xl1(i);
        yp2(i)=yl1(i);
        yawp2(i)=yawl1(i);
    elseif resultassignment(3,3*i-2)==0&&resultassignment(3,3*i-1)==0&&resultassignment(3,3*i)==1
        xp2(i)=xl2(i);
        yp2(i)=yl2(i);
        yawp2(i)=yawl2(i);
    end  
end
%% motion
    
    for i=1:num
        while 1
            currenti=i;
            %inputCoordinates=input('rover positions = ');%%%Taking inputs the number of inputs are depending on the number of rovers in the Vrep environment
            %if there is only one rover, it takes [x1 y1 angle1].
            inputCoordinates= [xp0(i) yp0(i) yawp0(i) xp1(i) yp1(i) yawp1(i) xp2(i) yp2(i) yawp2(i)];%%%Taking inputs the number of inputs are depending on the number of rovers in the Vrep environment
            %if there is only one rover, it takes [x1 y1 angle1].
            %if there are three rovers, it takes [x1 y1 angle1 x2 y2 angle2 x3 y3 angle3]
            
            if(inputCoordinates == 's')
                %this program terminate if user inputs 's'
                break;
            end
            
            
            packedData   = vrep.simxPackFloats(inputCoordinates);%covert into floats data pack
            [returnCode] = vrep.simxWriteStringStream(clientID,'stringname',packedData,vrep.simx_opmode_oneshot); %write the String to the handle
            
            [returnCode,rover0]=vrep.simxGetObjectHandle(clientID,strcat('rover#1'),vrep.simx_opmode_blocking);
            [returnCode,position0]=vrep.simxGetObjectPosition(clientID,rover0,-1,vrep.simx_opmode_blocking);
            [returnCode,rover1]=vrep.simxGetObjectHandle(clientID,strcat('rover#2'),vrep.simx_opmode_blocking);
            [returnCode,position1]=vrep.simxGetObjectPosition(clientID,rover1,-1,vrep.simx_opmode_blocking);
            [returnCode,rover2]=vrep.simxGetObjectHandle(clientID,strcat('rover#3'),vrep.simx_opmode_blocking);
            [returnCode,position2]=vrep.simxGetObjectPosition(clientID,rover2,-1,vrep.simx_opmode_blocking);
            xr0(i)=position0(:,1);
            yr0(i)=position0(:,2);
            xr1(i)=position1(:,1);
            yr1(i)=position1(:,2);
            xr2(i)=position2(:,1);
            yr2(i)=position2(:,2);
            r0(1,i+1)=xr0(i);
            r0(2,i+1)=yr0(i);
            r1(1,i+1)=xr1(i);
            r1(2,i+1)=yr1(i);
            r2(1,i+1)=xr2(i);
            r2(2,i+1)=yr2(i);
            err0(i)=norm([xp0(i)-xr0(i),yp0(i)-yr0(i)],2);
            err1(i)=norm([xp1(i)-xr1(i),yp1(i)-yr1(i)],2);
            err2(i)=norm([xp2(i)-xr2(i),yp2(i)-yr2(i)],2);
            threshold=0.02;
            if abs(err0(i))<=threshold&&abs(err1(i))<=threshold&&abs(err2(i))<=threshold&&i<num
                break
            elseif abs(err0(i))<=threshold&&abs(err1(i))<=threshold&&abs(err2(i))<=threshold&&i==num
                break
            end
        end
    end
    %
    vrep.simxFinish(-1);
end

vrep.delete();

xm0=r0(1,:);
ym0=r0(2,:);
xm1=r1(1,:);
ym1=r1(2,:);
xm2=r2(1,:);
ym2=r2(2,:);

figure(1)


plot(xm0,ym0,'r*')
hold on
axis equal
plot(xm1,ym1,'g*')
hold on
axis equal
plot(xm2,ym2,'b*')
hold on
axis equal
plot(xp0,yp0,'ro');
hold on
axis equal
plot(xp1,yp1,'go');
hold on
axis equal
plot(xp2,yp2,'bo');
hold on
axis equal

C0_pos = [ 1.108, 0.34 ];
C1_pos = [ 1.072, - 0.18 ];
%obstacles size
c0_w = 0.1; c1_w = 0.1;
%positions setting
obs0_x=C0_pos(:,1);    obs0_y=C0_pos(:,2);
obs1_x=C1_pos(:,1);    obs1_y=C1_pos(:,2);
%obstacles centre x and y coordinates
obs_pos = [[obs0_x obs0_y];[obs1_x obs1_y]];

%increase value for obs plotting
obplot = [c0_w,c1_w];

% obstacle 1
ce = obs_pos(1,:);
w = obplot(1);
p1 = [(ce(1)) - (w / 2), (ce(2)) - (w / 2)];
p2 = [(ce(1)) + (w / 2), (ce(2)) - (w / 2)];
p3 = [(ce(1)) + (w / 2), (ce(2)) + (w / 2)];
p4 = [(ce(1)) - (w / 2), (ce(2)) + (w / 2)];
X=[p1(1),p2(1),p3(1),p4(1),p1(1)];
Y=[p1(2),p2(2),p3(2),p4(2),p1(2)];
plot(X,Y,'-y')

% obstacle 2
ce = obs_pos(2,:);
w = obplot(2);
p1 = [(ce(1)) - (w / 2), (ce(2)) - (w / 2)];
p2 = [(ce(1)) + (w / 2), (ce(2)) - (w / 2)];
p3 = [(ce(1)) + (w / 2), (ce(2)) + (w / 2)];
p4 = [(ce(1)) - (w / 2), (ce(2)) + (w / 2)];
X=[p1(1),p2(1),p3(1),p4(1),p1(1)];
Y=[p1(2),p2(2),p3(2),p4(2),p1(2)];
plot(X,Y,'-y')
hold on
for i=1:num+1
    if i>1&&i<=num+1
        plot([xm0(i) xm1(i) xm2(i)],[ym0(i) ym1(i) ym2(i)],'m-')
    end
end
for i=1:num
    plot([xp0(i) xp1(i) xp2(i)],[yp0(i) yp1(i) yp2(i)],'k--')
end
plot(xm0,ym0,'r-');
plot(xm1,ym1,'g--');
plot(xm2,ym2,'b:');
hold off
legend('Rover0','Rover1','Rover2','Goal0','Goal1','Goal2','Ob1','Ob2')
title('Motion Of Rovers(Line With Pairing)');
xlabel('X');
ylabel('Y');






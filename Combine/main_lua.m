clear;
clc;
tic;
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
%%
if (clientID>-1)
    disp('Connected')
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);%start Vrep simulation
    %% obstacle avoidance
    %------------simulation of obstacle avoidance----------------%
    % to load the data
    load('test.mat')
    %Get object handles
    [returnCode,target_goal]=vrep.simxGetObjectHandle(clientID,'Goal',vrep.simx_opmode_blocking);
    [returnCode,target_pos]=vrep.simxGetObjectPosition(clientID,target_goal,-1,vrep.simx_opmode_blocking);
    
    ini_rover_pos_x=0;
    ini_rover_pos_y=0;
    tar_pos_x=target_pos(:,1);
    tar_pos_y=target_pos(:,2);
    
    sx=size(x,2);
    obstacles = zeros([2,sx]);
    
    for s = 1:sx
        C_pos = [x(:,s)*(-1); y(:,s)*(-1)];
        obstacles(:,s) = C_pos;
        s = s+1;
    end
    
    
    %iniial position
    begin=[ini_rover_pos_x;
        ini_rover_pos_y];
    
    %destination
    over=[tar_pos_x;
        tar_pos_y];
    
    %APF range
    range=0.6;
    
    figure(1);
    axis([-1.5 2 -3 3]);
    hold on;
    plot(begin(1),begin(2),'+k','MarkerSize',20);
    plot(over(1),over(2),'+b','MarkerSize',20);
    
    
    for s = 1:sx
        plot(x*(-1),y*(-1),'*')
        hold on;
    end
    xlabel('X');
    ylabel('Y');
    title('Path representaion (APF) for one rover');
    text(6,8,'+ the initial position','Color','k','FontSize',10);
    text(6,7,'+ the goal','Color','b','FontSize',10);
    text(6,6,'[] obstacles','Color','r','FontSize',10);
    %the path stored in here
    point= apf_Path(begin,over,obstacles,range);
    
    %move the rover based on the points in Path
    num=size(point,2);%get the number of points in the Path
    xpp=point(1,:);
    ypp=point(2,:);
    
    %% get rover
    [returnCode,rover0]=vrep.simxGetObjectHandle(clientID,strcat('rover0'),vrep.simx_opmode_blocking);
    [returnCode,position0]=vrep.simxGetObjectPosition(clientID,rover0,-1,vrep.simx_opmode_blocking);
    [returnCode,yaw0]=vrep.simxGetObjectOrientation(clientID,rover0,-1,vrep.simx_opmode_blocking);
    [returnCode,rover1]=vrep.simxGetObjectHandle(clientID,strcat('rover1'),vrep.simx_opmode_blocking);
    [returnCode,position1]=vrep.simxGetObjectPosition(clientID,rover1,-1,vrep.simx_opmode_blocking);
    [returnCode,yaw1]=vrep.simxGetObjectOrientation(clientID,rover1,-1,vrep.simx_opmode_blocking);
    [returnCode,rover2]=vrep.simxGetObjectHandle(clientID,strcat('rover2'),vrep.simx_opmode_blocking);
    [returnCode,position2]=vrep.simxGetObjectPosition(clientID,rover2,-1,vrep.simx_opmode_blocking);
    [returnCode,yaw2]=vrep.simxGetObjectOrientation(clientID,rover2,-1,vrep.simx_opmode_blocking);
    r0=zeros([2 num+1]);
    r1=zeros([2 num+1]);
    r2=zeros([2 num+1]);
    yaw=zeros([3 num+1]);
    r0(1,1)=position0(:,1);
    r0(2,1)=position0(:,2);
    r1(1,1)=position1(:,1);
    r1(2,1)=position1(:,2);
    r2(1,1)=position2(:,1);
    r2(2,1)=position2(:,2);
    yaw(1,1)=yaw0(:,3)*180/pi;
    yaw(2,1)=yaw1(:,3)*180/pi;
    yaw(3,1)=yaw2(:,3)*180/pi;
    %% line formation
    [returnCode,rover0]=vrep.simxGetObjectHandle(clientID,strcat('rover0'),vrep.simx_opmode_blocking);
    [returnCode,position0]=vrep.simxGetObjectPosition(clientID,rover0,-1,vrep.simx_opmode_blocking);
    [returnCode,rover1]=vrep.simxGetObjectHandle(clientID,strcat('rover1'),vrep.simx_opmode_blocking);
    [returnCode,position1]=vrep.simxGetObjectPosition(clientID,rover1,-1,vrep.simx_opmode_blocking);
    [returnCode,rover2]=vrep.simxGetObjectHandle(clientID,strcat('rover2'),vrep.simx_opmode_blocking);
    [returnCode,position2]=vrep.simxGetObjectPosition(clientID,rover2,-1,vrep.simx_opmode_blocking);
    x0=position0(:,1);
    y0=position0(:,2);
    x1=position1(:,1);
    y1=position1(:,2);
    x2=position2(:,1);
    y2=position2(:,2);
    d=0.52;
    offset=5/6*pi;
    [xl0,yl0,yawl0,xl1,yl1,yawl1,xl2,yl2,yawl2,angle] = Dcmline(x0,y0,xpp,ypp,num,d,offset);
    %% pairing
    [xp0,yp0,yawp0,xp1,yp1,yawp1,xp2,yp2,yawp2,resultcostMat,resultcost,resultassignment]=pairing(num,x0,y0,x1,y1,x2,y2,xl0,yl0,yawl0,xl1,yl1,yawl1,xl2,yl2,yawl2);
    %% motion
    
    for i=1:num
        while 1
            currenti=i;
            %inputCoordinates=input('rover positions = ');%%%Taking inputs the number of inputs are depending on the number of rovers in the Vrep environment
            %if there is only one rover, it takes [x1 y1 angle1].
            if yawp0(i)<0
                yawf0(i)=yawp0(i)*(-1);
            else
                yawf0(i)=yawp0(i);
            end
            if yawp1(i)<0
                yawf1(i)=yawp1(i)*(-1);
            else
                yawf1(i)=yawp1(i);
            end
            if yawp2(i)<0
                yawf2(i)=yawp2(i)*(-1);
            else
                yawf2(i)=yawp2(i);
            end
            inputCoordinates= [xp0(i) yp0(i) yawp0(i) xp1(i) yp1(i) yawp1(i) xp2(i) yp2(i) yawp2(i)];%%%Taking inputs the number of inputs are depending on the number of rovers in the Vrep environment
            %if there is only one rover, it takes [x1 y1 angle1].
            %if there are three rovers, it takes [x1 y1 angle1 x2 y2 angle2 x3 y3 angle3]
            
            if(inputCoordinates == 's')
                %this program terminate if user inputs 's'
                break;
            end
            
            
            packedData   = vrep.simxPackFloats(inputCoordinates);%covert into floats data pack
            [returnCode] = vrep.simxWriteStringStream(clientID,'stringname',packedData,vrep.simx_opmode_oneshot); %write the String to the handle
            
            [returnCode,rover0]=vrep.simxGetObjectHandle(clientID,strcat('rover0'),vrep.simx_opmode_blocking);
            [returnCode,position0]=vrep.simxGetObjectPosition(clientID,rover0,-1,vrep.simx_opmode_blocking);
            [returnCode,yaw0]=vrep.simxGetObjectOrientation(clientID,rover0,-1,vrep.simx_opmode_blocking);
            [returnCode,rover1]=vrep.simxGetObjectHandle(clientID,strcat('rover1'),vrep.simx_opmode_blocking);
            [returnCode,position1]=vrep.simxGetObjectPosition(clientID,rover1,-1,vrep.simx_opmode_blocking);
            [returnCode,yaw1]=vrep.simxGetObjectOrientation(clientID,rover1,-1,vrep.simx_opmode_blocking);
            [returnCode,rover2]=vrep.simxGetObjectHandle(clientID,strcat('rover2'),vrep.simx_opmode_blocking);
            [returnCode,position2]=vrep.simxGetObjectPosition(clientID,rover2,-1,vrep.simx_opmode_blocking);
            [returnCode,yaw2]=vrep.simxGetObjectOrientation(clientID,rover2,-1,vrep.simx_opmode_blocking);
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
            yaw(1,i+1)=yaw0(:,3)*180/pi;
            yaw(2,i+1)=yaw1(:,3)*180/pi;
            yaw(3,i+1)=yaw2(:,3)*180/pi;
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
   

xm0=r0(1,:);
ym0=r0(2,:);
xm1=r1(1,:);
ym1=r1(2,:);
xm2=r2(1,:);
ym2=r2(2,:);

pause(10)

% 
% figure
% 
% plot(xm0,ym0,'r*')
% hold on
% axis equal
% plot(xm1,ym1,'g*')
% hold on
% axis equal
% plot(xm2,ym2,'b*')
% hold on
% axis equal
% plot(xp0,yp0,'ro');
% hold on
% axis equal
% plot(xp1,yp1,'go');
% hold on
% axis equal
% plot(xp2,yp2,'bo');
% hold on
% axis equal
% 
% C0_pos = [ 0.35, -1.075  ];
% C1_pos = [ -0.145, -1.045 ];
% %obstacles size
% c0_w = 0.1; c1_w = 0.1;
% %positions setting
% obs0_x=C0_pos(:,1);    obs0_y=C0_pos(:,2);
% obs1_x=C1_pos(:,1);    obs1_y=C1_pos(:,2);
% %obstacles centre x and y coordinates
% obs_pos = [[obs0_x obs0_y];[obs1_x obs1_y]];
% %increase value for obs plotting
% obplot = [c0_w,c1_w];
% hold on
% axis equal
% % obstacle 1
% ce = obs_pos(1,:);
% w = obplot(1);
% p1 = [(ce(1)) - (w / 2), (ce(2)) - (w / 2)];
% p2 = [(ce(1)) + (w / 2), (ce(2)) - (w / 2)];
% p3 = [(ce(1)) + (w / 2), (ce(2)) + (w / 2)];
% p4 = [(ce(1)) - (w / 2), (ce(2)) + (w / 2)];
% X=[p1(1),p2(1),p3(1),p4(1),p1(1)];
% Y=[p1(2),p2(2),p3(2),p4(2),p1(2)];
% fill(X,Y,'y')
% 
% % obstacle 2
% ce = obs_pos(2,:);
% w = obplot(2);
% p1 = [(ce(1)) - (w / 2), (ce(2)) - (w / 2)];
% p2 = [(ce(1)) + (w / 2), (ce(2)) - (w / 2)];
% p3 = [(ce(1)) + (w / 2), (ce(2)) + (w / 2)];
% p4 = [(ce(1)) - (w / 2), (ce(2)) + (w / 2)];
% X=[p1(1),p2(1),p3(1),p4(1),p1(1)];
% Y=[p1(2),p2(2),p3(2),p4(2),p1(2)];
% fill(X,Y,'y')
% 
% hold on
% for i=1:num+1
%     if i>1&&i<=num+1
%         plot([xm0(i) xm1(i) xm2(i)],[ym0(i) ym1(i) ym2(i)],'m-')
%     end
% end
% for i=1:num
%     plot([xp0(i) xp1(i) xp2(i)],[yp0(i) yp1(i) yp2(i)],'k--')
% end
% plot(xm0,ym0,'r-');
% plot(xm1,ym1,'g--');
% plot(xm2,ym2,'b:');
% hold off
% 
% legend('Rover0','Rover1','Rover2','Goal0','Goal1','Goal2','Ob1','Ob2')
% title('Motion Of Rovers(Line With Pairing)');
% xlabel('X');
% ylabel('Y');

%% Prepare for self-assembling
    xs0=-0.1768;
    ys0=-1.6339;
    yaws0=60;
    xs1=-1.0268;
    ys1=-1.6339;
    yaws1=60;
    xs2=-0.5918;
    ys2=-1.6339;
    yaws2=60;
    while 1
        
        %inputCoordinates=input('rover positions = ');%%%Taking inputs the number of inputs are depending on the number of rovers in the Vrep environment
        %if there is only one rover, it takes [x1 y1 angle1].
        inputCoordinates= [xs0 ys0 yaws0 xs1 ys1 yaws1 xs2 ys2 yaws2];%%%Taking inputs the number of inputs are depending on the number of rovers in the Vrep environment
        %if there is only one rover, it takes [x1 y1 angle1].
        %if there are three rovers, it takes [x1 y1 angle1 x2 y2 angle2 x3 y3 angle3]
        
        if(inputCoordinates == 's')
            %this program terminate if user inputs 's'
            break;
        end
        
        
        packedData   = vrep.simxPackFloats(inputCoordinates);%covert into floats data pack
        [returnCode] = vrep.simxWriteStringStream(clientID,'stringname',packedData,vrep.simx_opmode_oneshot); %write the String to the handle
        [returnCode,rover0]=vrep.simxGetObjectHandle(clientID,strcat('rover0'),vrep.simx_opmode_blocking);
        [returnCode,position0]=vrep.simxGetObjectPosition(clientID,rover0,-1,vrep.simx_opmode_blocking);
        [returnCode,yaw0]=vrep.simxGetObjectOrientation(clientID,rover0,-1,vrep.simx_opmode_blocking);
        [returnCode,rover1]=vrep.simxGetObjectHandle(clientID,strcat('rover1'),vrep.simx_opmode_blocking);
        [returnCode,position1]=vrep.simxGetObjectPosition(clientID,rover1,-1,vrep.simx_opmode_blocking);
        [returnCode,yaw1]=vrep.simxGetObjectOrientation(clientID,rover1,-1,vrep.simx_opmode_blocking);
        [returnCode,rover2]=vrep.simxGetObjectHandle(clientID,strcat('rover2'),vrep.simx_opmode_blocking);
        [returnCode,position2]=vrep.simxGetObjectPosition(clientID,rover2,-1,vrep.simx_opmode_blocking);
        [returnCode,yaw2]=vrep.simxGetObjectOrientation(clientID,rover2,-1,vrep.simx_opmode_blocking);
        xc0=position0(:,1);
        yc0=position0(:,2);
        xc1=position1(:,1);
        yc1=position1(:,2);
        xc2=position2(:,1);
        yc2=position2(:,2);
        e0=norm([xs0-xc0,ys0-yc1],2);
        e1=norm([xs1-xc1,ys1-yc1],2);
        e2=norm([xs2-xc2,ys2-yc2],2);
        threshold=0.015;
        if abs(e0)<=threshold&&abs(e1)<=threshold&&abs(e2)<=threshold
            break
        end
    end
    pause(10)
    closesignal=1;
    [returnCode]=vrep.simxSetIntegerSignal(clientID,'closesignal',closesignal,vrep.simx_opmode_oneshot);
    pause(10)
    
    [returnCode,rover0]=vrep.simxGetObjectHandle(clientID,strcat('rover0'),vrep.simx_opmode_blocking);
    [returnCode,position0]=vrep.simxGetObjectPosition(clientID,rover0,-1,vrep.simx_opmode_blocking);
    [returnCode,rover1]=vrep.simxGetObjectHandle(clientID,strcat('rover1'),vrep.simx_opmode_blocking);
    [returnCode,position1]=vrep.simxGetObjectPosition(clientID,rover1,-1,vrep.simx_opmode_blocking);
    [returnCode,rover2]=vrep.simxGetObjectHandle(clientID,strcat('rover2'),vrep.simx_opmode_blocking);
    [returnCode,position2]=vrep.simxGetObjectPosition(clientID,rover2,-1,vrep.simx_opmode_blocking);
    d1=position0(:,1)-position1(:,1);
    d2=position1(:,1)-position2(:,1);
    
    while 1
        xb2=2.3;
        xb1=xb2+d2;
        xb0=xb1+d1;
        yb2=ys2;
        yb1=ys1;
        yb0=ys0;
        yawb2=yaws2;
        yawb1=yaws1;
        yawb0=yaws0;
        %inputCoordinates=input('rover positions = ');%%%Taking inputs the number of inputs are depending on the number of rovers in the Vrep environment
        %if there is only one rover, it takes [x1 y1 angle1].
        inputCoordinates= [xb0 yb0 yawb0 xb2 yb2 yawb2 xb1 yb1 yawb1];%%%Taking inputs the number of inputs are depending on the number of rovers in the Vrep environment
        %if there is only one rover, it takes [x1 y1 angle1].
        %if there are three rovers, it takes [x1 y1 angle1 x2 y2 angle2 x3 y3 angle3]
        
        if(inputCoordinates == 's')
            %this program terminate if user inputs 's'
            break;
        end
        
        
        packedData   = vrep.simxPackFloats(inputCoordinates);%covert into floats data pack
        [returnCode] = vrep.simxWriteStringStream(clientID,'stringname',packedData,vrep.simx_opmode_oneshot); %write the String to the handle
        [returnCode,rover0]=vrep.simxGetObjectHandle(clientID,strcat('rover0'),vrep.simx_opmode_blocking);
        [returnCode,position0]=vrep.simxGetObjectPosition(clientID,rover0,-1,vrep.simx_opmode_blocking);
        [returnCode,yaw0]=vrep.simxGetObjectOrientation(clientID,rover0,-1,vrep.simx_opmode_blocking);
        [returnCode,rover1]=vrep.simxGetObjectHandle(clientID,strcat('rover1'),vrep.simx_opmode_blocking);
        [returnCode,position1]=vrep.simxGetObjectPosition(clientID,rover1,-1,vrep.simx_opmode_blocking);
        [returnCode,yaw1]=vrep.simxGetObjectOrientation(clientID,rover1,-1,vrep.simx_opmode_blocking);
        [returnCode,rover2]=vrep.simxGetObjectHandle(clientID,strcat('rover2'),vrep.simx_opmode_blocking);
        [returnCode,position2]=vrep.simxGetObjectPosition(clientID,rover2,-1,vrep.simx_opmode_blocking);
        [returnCode,yaw2]=vrep.simxGetObjectOrientation(clientID,rover2,-1,vrep.simx_opmode_blocking);
        xc0=position0(:,1);
        yc0=position0(:,2);
        xc1=position1(:,1);
        yc1=position1(:,2);
        xc2=position2(:,1);
        yc2=position2(:,2);
        eb0=norm([xb0-xc0,yb0-yc1],2);
        eb1=norm([xb1-xc1,yb1-yc1],2);
        eb2=norm([xb2-xc2,yb2-yc2],2);
        threshold=0.01;
        if abs(eb0)<=threshold&&abs(eb1)<=threshold&&abs(eb2)<=threshold
            break
        end
    end
%
    vrep.simxFinish(-1);
end

vrep.delete();
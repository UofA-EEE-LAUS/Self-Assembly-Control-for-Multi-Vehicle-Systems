%MATLAB script for the rover formation
%---------------------------------SETUP----------------------------------%
% load api library
vrep=remApi('remoteApi');
% close all the potential link
vrep.simxFinish(-1);
%set up connection to v-rep simulation on port 19999
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
% open the synchronous mode to control the objects in vrep
vrep.simxSynchronous(clientID,true);
% Simulation Initialization
vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
%clientID is -1 if the connection to the server was NOT possible
if (clientID>-1)
    disp('connected to v-rep');
%------------------------------CODE HERE------------------------------%
[returnCode,rover]=vrep.simxGetObjectHandle(clientID,strcat('rover'),vrep.simx_opmode_blocking);
[returnCode,laser_sensor]=vrep.simxGetObjectHandle(clientID,strcat('laser_sensor'),vrep.simx_opmode_oneshot_wait);
[returnCode,Xo]=vrep.simxGetObjectPosition(clientID,rover,-1,vrep.simx_opmode_blocking);
[returnCode] = vrep.simxGetObjectOrientation(clientID,laser_sensor,-1,vrep.simx_opmode_streaming);
[returnCode, orientations] = vrep.simxGetObjectOrientation(clientID,laser_sensor,-1,vrep.simx_opmode_buffer);

%Xo=[0 0];%起点位置
k=15;%计算引力需要的增益系数
m=8;%计算斥力的增益系数，都是自己设定的。intial 4
Po=5;%Obstacle affects the distance. When the distance between the obstacle and the car is greater than this distance,
%the repulsion is 0, that is, it is not affected by the obstacle. I also set initial 2.5 myself.
n=32;
l=0.1;
J=600;

%Xsum=[20 20;8 4;9 4;10 4;11 4;12 4;8 8;9 8;10 8;11 8;12 8;8 5;8 6;8 7;12 5;12 6;12 7;8 12;9 12;10 12;11 12;12 12;8 16;9 16;10 16;11 16;12 16;8 13;8 14;8 15;12 13;12 14;12 15];%This vector is (n + 1) * 2 dimensions, where [10 10] is the target position, and the rest are obstacle positions.Xj=Xo;%j=1
[returnCodea,Obstacles]=vrep.simxGetObjectHandle(clientID,strcat('Cuboid'),vrep.simx_opmode_blocking);
[returnCodea,Xsum]=vrep.simxGetObjectPosition(clientID,Obstacles,-1,vrep.simx_opmode_blocking);
[returnCodea] = vrep.simxGetObjectOrientation(clientID,laser_sensor,-1,vrep.simx_opmode_streaming);
[returnCodea, orientations_1] = vrep.simxGetObjectOrientation(clientID,laser_sensor,-1,vrep.simx_opmode_buffer);
 Xj=Xo;
for j=1:J%循环开始
   Goal(j,1)=Xj(:,1);
   Goal(j,2)=Xj(:,2);
% Goal(j,1)=Xj(1);
% Goal(j,2)=Xj(2);

Theta=compute_angle(Xj,Xsum,n);
Angle=Theta(1);
angle_at=Theta(1);
[Fatx,Faty]=compute_Attract(Xj,Xsum,k,Angle);
for i=1:n
angle_re(i)=Theta(i+1);%计算斥力用的角度，是个向量，因为有n个障碍，就有n个角度。
end

%调用计算斥力模块
[Yrerxx,Yreryy]=compute_repulsion(Xj,Xsum,m,angle_re,n,Po);%计算出斥力在x,y方向的分量数组。
%计算合力和方向，这有问题，应该是数，每个j循环的时候合力的大小应该是一个唯一的数，不是数组。应该把斥力的所有分量相加，引力所有分量相加。
Fsumyj=Faty+Yreryy;%y方向的合力
Fsumxj=Fatx+Yrerxx;%x方向的合力
Position_angle(j)=atan(Fsumyj/Fsumxj);%合力与x轴方向的夹角向量
%计算车的下一步位置
if Fsumyj < 0 && Fsumxj <0
   Xnext(1)=Xj(1)-l*cos(Position_angle(j));
   Xnext(2)=Xj(2)-l*sin(Position_angle(j));
else
   Xnext(1)=Xj(1)+l*cos(Position_angle(j));
   Xnext(2)=Xj(2)+l*sin(Position_angle(j));
end
%保存车的每一个位置在向量中
Xj=Xnext;
%判断
if ((Xj(1)-Xsum(1,1))>0)&((Xj(2)-Xsum(1,2))>0)%是应该完全相等的时候算作到达，还是只是接近就可以？现在按完全相等的时候编程。
%K=j%记录迭代到多少次，到达目标。
break;
end
end
K=j;
Goal(K,1)=Xsum(1,1);%把路径向量的最后一个点赋值为目标
Goal(K,2)=Xsum(1,2);

%***********************************画出障碍，起点，目标，路径点*************************
%画出路径
X=Goal(:,1);
Y=Goal(:,2);
%路径向量Goal是二维数组,X,Y分别是数组的x,y元素的集合，是两个一维数组。
% x=[8 9 10 11 12 8 9 10 11 12 8 8 8 12 12 12 8 9 10 11 12 8 9 10 11 12 8 8 8 12 12 12 ];%障碍的x坐标
% y=[4 4 4 4 4 8 8 8 8 8 5 6 7 5 6 7 12 12 12 12 12 16 16 16 16 16 13 14 15 13 14 15];
% plot(x,y,'o',Xsum(1,1),Xsum(1,2),'v',0,0,'ms',X,Y,'.r');



%------------------------------CODE HERE------------------------------%
%destroy connection to v-rep simulation
    vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot_wait);
else
    disp('Failed connecting to remote API server');
end
vrep.delete()

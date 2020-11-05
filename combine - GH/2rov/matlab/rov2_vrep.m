%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%MATLAB script for controlling the 3 omni-wheel rover platform

%------------------------------INSTRUCTIONS--%-----------------------------%
%{
1.  Ensure the files inside of
    C:\Program Files\V-REP3\V-REP_PRO_EDU\programming\remoteApiBindings\matlab\matlab
    C:\Program Files\V-REP3\V-REP_PRO_EDU\programming\remoteApiBindings\lib\lib\Windows\64Bit
    are copied into the current MATLAB workspace

2.  In v-rep, ensure that the line:
    simRemoteApi.start(19999)
    is inside a non-threaded child script, under sysCall_init()
    (this runs once and starts the internal server for MATLAB to connect to)

3.  For more information on API functions, please see
    http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsMatlab.htm
%}

%---------------------------------SCRIPT----------------------------------%

tic
%constructs remote api object
vrep=remApi('remoteApi');
%destroy's any current connections to v-rep simulation
vrep.simxFinish(-1);
%establishes connection to v-rep simulation on port 19999
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

%clientID is -1 if the connection to the server was NOT possible
if (clientID>-1)
    disp('connected to v-rep');
    
    %------------------------------CODE HERE------------------------------%
    % al the position here is taken from the V-REP scene
         %------------simulation of obstacle avoidance----------------% 
    
    %Get object handles
    [~,C0]=vrep.simxGetObjectHandle(clientID,'C2',vrep.simx_opmode_blocking);
    [~,C1]=vrep.simxGetObjectHandle(clientID,'C3',vrep.simx_opmode_blocking);
    [~,C2]=vrep.simxGetObjectHandle(clientID,'C4',vrep.simx_opmode_blocking);
    [~,C3]=vrep.simxGetObjectHandle(clientID,'C6',vrep.simx_opmode_blocking);
    [~,C4]=vrep.simxGetObjectHandle(clientID,'C8',vrep.simx_opmode_blocking);
    
    [~,ini_rover]=vrep.simxGetObjectHandle(clientID,'rover0',vrep.simx_opmode_blocking);
    [~,ini_rover1]=vrep.simxGetObjectHandle(clientID,'rover1',vrep.simx_opmode_blocking);
    [returnCode,target_goal]=vrep.simxGetObjectHandle(clientID,'goal',vrep.simx_opmode_blocking);
    [returnCode,cam0]=vrep.simxGetObjectHandle(clientID,'Vision_sensor0',vrep.simx_opmode_blocking);

     %Get obstacles position 
    [returnCode,C0_pos]=vrep.simxGetObjectPosition(clientID,C0,-1,vrep.simx_opmode_blocking);
    [returnCode,C1_pos]=vrep.simxGetObjectPosition(clientID,C1,-1,vrep.simx_opmode_blocking);
    [returnCode,C2_pos]=vrep.simxGetObjectPosition(clientID,C2,-1,vrep.simx_opmode_blocking);
    [returnCode,C3_pos]=vrep.simxGetObjectPosition(clientID,C3,-1,vrep.simx_opmode_blocking);
    [returnCode,C4_pos]=vrep.simxGetObjectPosition(clientID,C4,-1,vrep.simx_opmode_blocking);
   
    [returnCode,ini_rover0_pos]=vrep.simxGetObjectPosition(clientID,ini_rover,-1,vrep.simx_opmode_blocking);
    [returnCode,ini_rover1_pos]=vrep.simxGetObjectPosition(clientID,ini_rover1,-1,vrep.simx_opmode_blocking);
    [returnCode,target_pos]=vrep.simxGetObjectPosition(clientID,target_goal,-1,vrep.simx_opmode_blocking);
      
    %obstacles size
    w = 0.1;  
        
    %positions setting 
    obs0_x=C0_pos(:,1);    obs0_y=C0_pos(:,2);
    obs1_x=C1_pos(:,1);    obs1_y=C1_pos(:,2);
    obs2_x=C2_pos(:,1);    obs2_y=C2_pos(:,2);
    obs3_x=C3_pos(:,1);    obs3_y=C3_pos(:,2);
    obs4_x=C4_pos(:,1);    obs4_y=C4_pos(:,2);

    ini_rover_pos_x=ini_rover0_pos(:,1);
    ini_rover_pos_y=ini_rover0_pos(:,2);
    tar_pos_x=target_pos(:,1);
    tar_pos_y=target_pos(:,2);

    obstacles=[ obs0_x obs1_x obs2_x obs3_x obs4_x;
                obs0_y obs1_y obs2_y obs3_y obs4_y];

    rover_obs = [ini_rover1_pos(:,1);
                 ini_rover1_pos(:,2)]
            
    %obstacles centre x and y coordinates
    obs_pos = [[obstacles(1,1) obstacles(2,1)];
               [obstacles(1,2) obstacles(2,2)];
               [obstacles(1,3) obstacles(2,3)];
               [obstacles(1,4) obstacles(2,4)];
               [obstacles(1,5) obstacles(2,5)]];
    rov_pos = [[rover_obs(1,1) rover_obs(2,1)]];
    
    %increase value for obs plotting
    w =0.1;
    
    %iniial position
    begin =[ini_rover_pos_x;
            ini_rover_pos_y];
    
    %destination 
    over=[tar_pos_x;
          tar_pos_y];
   
            %---------plotting the simulation results in Matlab--------%
    
    figure(1);
    axis([-1.5 2 -3 3]);
    hold on;
    plot(begin(1),begin(2),'+k','MarkerSize',20);
    plot(over(1),over(2),'+b','MarkerSize',20);

       % obstacle 0
        ce = obs_pos(1,:)
        p1 = [(ce(1)) - (w / 2), (ce(2)) - (w / 2)];
        p2 = [(ce(1)) + (w / 2), (ce(2)) - (w / 2)];
        p3 = [(ce(1)) + (w / 2), (ce(2)) + (w / 2)];
        p4 = [(ce(1)) - (w / 2), (ce(2)) + (w / 2)];
        X=[p1(1),p2(1),p3(1),p4(1),p1(1)]
        Y=[p1(2),p2(2),p3(2),p4(2),p1(2)]
        plot(X,Y,'-r')   
        
        % obstacle 1
        ce = obs_pos(2,:)
        p1 = [(ce(1)) - (w / 2), (ce(2)) - (w / 2)];
        p2 = [(ce(1)) + (w / 2), (ce(2)) - (w / 2)];
        p3 = [(ce(1)) + (w / 2), (ce(2)) + (w / 2)];
        p4 = [(ce(1)) - (w / 2), (ce(2)) + (w / 2)];
        X=[p1(1),p2(1),p3(1),p4(1),p1(1)]
        Y=[p1(2),p2(2),p3(2),p4(2),p1(2)]
        plot(X,Y,'-r')
        
        % obstacle 2
        ce = obs_pos(3,:)
        p1 = [(ce(1)) - (w / 2), (ce(2)) - (w / 2)];
        p2 = [(ce(1)) + (w / 2), (ce(2)) - (w / 2)];
        p3 = [(ce(1)) + (w / 2), (ce(2)) + (w / 2)];
        p4 = [(ce(1)) - (w / 2), (ce(2)) + (w / 2)];
        X=[p1(1),p2(1),p3(1),p4(1),p1(1)]
        Y=[p1(2),p2(2),p3(2),p4(2),p1(2)]
        plot(X,Y,'-r')
        
        % obstacle 3
        ce = obs_pos(4,:)
        p1 = [(ce(1)) - (w / 2), (ce(2)) - (w / 2)];
        p2 = [(ce(1)) + (w / 2), (ce(2)) - (w / 2)];
        p3 = [(ce(1)) + (w / 2), (ce(2)) + (w / 2)];
        p4 = [(ce(1)) - (w / 2), (ce(2)) + (w / 2)];
        X=[p1(1),p2(1),p3(1),p4(1),p1(1)]
        Y=[p1(2),p2(2),p3(2),p4(2),p1(2)]
        plot(X,Y,'-r')
        
        % obstacle 4
        ce = obs_pos(5,:)
        p1 = [(ce(1)) - (w / 2), (ce(2)) - (w / 2)];
        p2 = [(ce(1)) + (w / 2), (ce(2)) - (w / 2)];
        p3 = [(ce(1)) + (w / 2), (ce(2)) + (w / 2)];
        p4 = [(ce(1)) - (w / 2), (ce(2)) + (w / 2)];
        X=[p1(1),p2(1),p3(1),p4(1),p1(1)]
        Y=[p1(2),p2(2),p3(2),p4(2),p1(2)]
        plot(X,Y,'-r')
    
         % rover 1
        ce = rov_pos(1,:)
        viscircles(ce,0.15);
        
    xlabel('X');
    ylabel('Y');
    title('Path representaion (APF) for one rover');
    text(6,8,'+ the initial position','Color','k','FontSize',10);
    text(6,7,'+ the goal','Color','b','FontSize',10);
    text(6,6,'[] obstacles','Color','r','FontSize',10);
       
    
    %the path stored in here 
    point= apf_Path(begin,over,obstacles,rover_obs);
    
    %move the rover based on the points in Path
    num=size(point,2);%get total number of steps taken by rover in the Path
    
    for i=1:num
        point= apf_Path(begin,over,obstacles,rover_obs);
        tem=point(:,i);
        x=tem(1);
        y=tem(2);
%         apf_MoveSpin(x,y);        
    end
    
    %set all 3 motor speed to 0 to stop movement of rover0 
    %handles 
    [returnCode,motor_0]=vrep.simxGetObjectHandle(clientID,'motor00',vrep.simx_opmode_blocking);
    [returnCode,motor_1]=vrep.simxGetObjectHandle(clientID,'motor10',vrep.simx_opmode_blocking);
    [returnCode,motor_2]=vrep.simxGetObjectHandle(clientID,'motor20',vrep.simx_opmode_blocking);
    
    %setting motor speed as 0
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_0,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_1,0,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,motor_2,0,vrep.simx_opmode_blocking);
    
    %destroy connection to v-rep simulation
    vrep.simxFinish(-1);
else
    disp('Failed connecting to remote API server');
end

vrep.delete()

total_time=toc
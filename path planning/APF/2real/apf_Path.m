function [ point ] = apf_Path()

tic
%constructs remote api object
vrep=remApi('remoteApi');
%destroy's any current connections to v-rep simulation
vrep.simxFinish(-1);
%establishes connection to v-rep simulation on port 19999
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

%clientID is -1 if the connection to the server was NOT possible
if (clientID>-1)    
     [~,C0]=vrep.simxGetObjectHandle(clientID,'C0',vrep.simx_opmode_blocking);
     [~,C1]=vrep.simxGetObjectHandle(clientID,'C1',vrep.simx_opmode_blocking);
     [~,C2]=vrep.simxGetObjectHandle(clientID,'C2',vrep.simx_opmode_blocking);     
     [~,C3]=vrep.simxGetObjectHandle(clientID,'C3',vrep.simx_opmode_blocking);
     [~,C4]=vrep.simxGetObjectHandle(clientID,'C4',vrep.simx_opmode_blocking);
     [~,C5]=vrep.simxGetObjectHandle(clientID,'C5',vrep.simx_opmode_blocking);
     [returnCode,target_goal]=vrep.simxGetObjectHandle(clientID,'goal',vrep.simx_opmode_blocking);
     [returnCode,ini_rover]=vrep.simxGetObjectHandle(clientID,'rover0',vrep.simx_opmode_blocking);
     
     %Get obstacles position 
    [returnCode,C0_pos]=vrep.simxGetObjectPosition(clientID,C0,-1,vrep.simx_opmode_blocking);
    [returnCode,C1_pos]=vrep.simxGetObjectPosition(clientID,C1,-1,vrep.simx_opmode_blocking);
    [returnCode,C2_pos]=vrep.simxGetObjectPosition(clientID,C2,-1,vrep.simx_opmode_blocking);
    [returnCode,C3_pos]=vrep.simxGetObjectPosition(clientID,C3,-1,vrep.simx_opmode_blocking);
    [returnCode,C4_pos]=vrep.simxGetObjectPosition(clientID,C4,-1,vrep.simx_opmode_blocking);
    [returnCode,C5_pos]=vrep.simxGetObjectPosition(clientID,C5,-1,vrep.simx_opmode_blocking);
    [returnCode,ini_rover0_pos]=vrep.simxGetObjectPosition(clientID,ini_rover,-1,vrep.simx_opmode_blocking);
    [returnCode,target_pos]=vrep.simxGetObjectPosition(clientID,target_goal,-1,vrep.simx_opmode_blocking);
    
    %positions setting 
    obs0_x=C0_pos(:,1);    obs0_y=C0_pos(:,2);
    obs1_x=C1_pos(:,1);    obs1_y=C1_pos(:,2);
    obs2_x=C2_pos(:,1);    obs2_y=C2_pos(:,2); 
    obs3_x=C3_pos(:,1);    obs3_y=C3_pos(:,2);
    obs4_x=C4_pos(:,1);    obs4_y=C4_pos(:,2);
    obs5_x=C5_pos(:,1);    obs5_y=C5_pos(:,2);
    
    obstacle=[ obs0_x   obs1_x    obs2_x    obs3_x      obs4_x      obs5_x;
               obs0_y   obs1_y    obs2_y    obs3_y      obs4_y      obs5_y];

    ini_rover_pos_x=ini_rover0_pos(:,1);
    ini_rover_pos_y=ini_rover0_pos(:,2);
    tar_pos_x=target_pos(:,1);
    tar_pos_y=target_pos(:,2);
    
    %iniial position
    begin=[ini_rover_pos_x;
           ini_rover_pos_y];
    
    %destination 
    over=[tar_pos_x;
          tar_pos_y];

%PATH_PLAN
iters=1; % iterations
curr=begin;
testR=0.5;   % radius of the circle, in which the current point is the centre. The threshold of MoveSpin Function is 0.1, the step length here should be bigger than the threshold
sp_len=testR/2; % the step length
iters_num=500;

while (norm(curr-over)>0.2) && (iters<=iters_num) % the error is 0.2
    
    ori_curr=curr; % stored the original position value

    % at first, find out the coordinates of 8 points around the robot
    for i=1:8  % the potential moving direction of robot is 360 degrees (2pi), then dividing the potential direction into 8 points (45 degrees between each point)
        testPoint(:,i)=[testR*sin((i-1)*pi/4)+ori_curr(1);testR*cos((i-1)*pi/4)+ori_curr(2)]; % the coordinate of these 8 points
%         testOut(:,i)=apf_Algrith(testPoint(:,i),over,obstacle); % the output is the resultant
        testOut(:,i)=apf_LM(testPoint(:,i),over,obstacle); % the output is the resultant
        
    end
    [~, num]=min(testOut); % find out the smallest one, since in the APF, the robot always moves to the point with the smallest potential energy  
    
    %the step length of iteration is sp_len
    curr=(ori_curr+testPoint(:,num))/2; % step length half of the circle radius is 0.2, thus the coordinate has to reduce to the half of the result
    
    %-------- strategy to escape local minima -------------%
        
       LM_test(:,iters)=curr; %tempory value for testing if the point is in LM 
    
    if iters > 2 %identify if the robot is trapped in local mimima (LM)--- identify this every 4 points (1&5,2&6,3&7.....)  
       tem_a=LM_test(:,iters); 
       tem_b=LM_test(:,iters-2);
       
       if (norm(tem_a-tem_b)) < 0.01  % if robot is in LM, then the distance must be equal
          LM_point=tem_a; % this point is the LM point 
    
%      %---------- Method 2: virtual obstacle  -------------%
          n=8; % the number of potential point around the current position 
          for j=1:n  
              testPoint_m2(:,j)=[testR*sin((j-1)*(2*pi/n))+ori_curr(1);testR*cos((j-1)*(2*pi/n))+ori_curr(2)];
              
              %crate a new matrix to store the new obstacles
              new_ob=obstacle;
              ob_size=size(new_ob,2);
              new_ob=[new_ob(:,1:ob_size) LM_point];
              testOut_m2(:,j)=apf_LM(testPoint_m2(:,j),over,new_ob); 
    
          end
           
              [~, num]=min(testOut_m2); 
    
              curr=(ori_curr+testPoint_m2(:,num))/2; 
% 
%       %----------- end of method 2 -----------------%
       else 

       end 
       
    else 
        
    end 
%--------------- end of strategy of escaping LM----------------%
    
%     plot(curr(1),curr(2),'og');
    point(:,iters)=curr;
%     num11=size(point,2)
% for i=1:num11
%     tem=point(:,i);
%     x=tem(1);
%     y=tem(2);
%     apf_MoveSpin(x,y);
% end       
        
   pause(0.01);
    iters=iters+1;
    
end


 if iters==1
    point(:,iters)=curr;
 end
end

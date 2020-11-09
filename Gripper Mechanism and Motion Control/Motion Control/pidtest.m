clear();
i=1;
vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
%%
if (clientID>-1)
    disp('Connected')
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);%start Vrep simulation

    [returnCode,rover]=vrep.simxGetObjectHandle(clientID,'rover0',vrep.simx_opmode_blocking);



    
    inputCoordinates=input('rover positions = ');
        H = zeros(200,4);
     t1=clock;
%% 
    while 1
         
         [returnCode,position]=vrep.simxGetObjectPosition(clientID,rover,-1,vrep.simx_opmode_blocking);
         [returnCode,orientation]=vrep.simxGetObjectOrientation(clientID,rover,-1,vrep.simx_opmode_blocking);
         

%       %%%Taking inputs the number of inputs are depending on the number of rovers in the Vrep environment
         t2=clock;
         et=etime(t2,t1);

         H(i,:)=[position(1,1),position(1,2),orientation(1,3),et];
         %if there is only one rover, it takes [x1 y1 angle1].
         %if there are three rovers, it takes [x1 y1 angle1 x2 y2 angle2 x3 y3 angle3]
         i=i+1;
         
         outputCoordinates=[inputCoordinates(1,1) inputCoordinates(1,2) inputCoordinates(1,3)];
         packedData   = vrep.simxPackFloats(outputCoordinates);%covert into floats data pack
         [returnCode] = vrep.simxWriteStringStream(clientID,'stringname',packedData,vrep.simx_opmode_oneshot); %write the String to the handle

      
           
          if(i==200)
             %this program terminate if user inputs 's'
             break;
          	
            
         end

      
         

    end
    
        %%

        
    vrep.simxFinish(-1);
end
%%
vrep.delete();

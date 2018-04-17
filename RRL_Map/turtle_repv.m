%==========================================================================

% MATLAB code for Project 3 (Planning Class)
% Point robot planning using A* Algorithm
% Written by Yash Shah (115710498)
% email ID: ysshah95@umd.edu
% 
% Implementation of A* Algorithm for finding a shortest path 
% between two points in an area (with obstacles)

%==========================================================================

% clc
% clear all

load('Path.mat','path');

vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

% initialize velocity
linear.x = 0;
linear.y = 0;
linear.z = 0;

angular.x = 0;
angular.y = 0;
angular.z = 0;

timeStamp = 0;

fileID = fopen('velocity.txt','w');
fprintf(fileID,'%6s %12s %18s %24s %30s %36s %42s\r\n','timeStamp','linear.x',...
    'linear.y', 'linear.z', 'angular.x', 'angular.y', 'angular.z');

if (clientID>-1)
    disp('Connected to Vrep');
    
    % Get Object Handles from Vrep
    
    [returnCode,Robot]=vrep.simxGetObjectHandle(clientID,'Turtlebot2',vrep.simx_opmode_blocking)  ;  
    [returnCode,left_Motor]=vrep.simxGetObjectHandle(clientID,'wheel_left_joint',vrep.simx_opmode_blocking);
    [returnCode,right_Motor]=vrep.simxGetObjectHandle(clientID,'wheel_right_joint',vrep.simx_opmode_blocking);
    
    % Get the position and Angle of the Robot from Vrep
    
    %     [returnCode,position]=vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_blocking);
    %     [returnCode,angle]=vrep.simxGetObjectOrientation(clientID,Robot,-1,vrep.simx_opmode_blocking);

    % Import the Path generated from A* Algorithm 
    
    path = flipud(path);
    Pathx = path(:,1);
    Pathy = path(:,2);
    
    k = length(path);
    
    % Initialize the Current and Target Position
    
    current_pos =[round(position(1,1),1),round(position(1,2),1)];
    target_pos = [Pathx(k),Pathy(k)];

    radius = 0.038;
    dist_wheels=0.3175;
    i = 2;
    
    
    
%     norm(current_pos - [Pathx(i),Pathy(i)])
    
%     while norm(current_pos - [Pathx(i),Pathy(i)]) < 0.2
%         
%         i=i+1;
%     end

    tic

    while norm(current_pos - target_pos) > 0.2
    
        % Get the coordinates of the next point
        next_pos = [Pathx(i),Pathy(i)];
        i = i+1 ;

%         dist = norm(next_pos - current_pos);
%         if dist < 0.2
%             continue;
%         end

        % Calculate the slope of current position and next position
        slope = atan2(round((next_pos(2)-current_pos(2)),1),round((next_pos(1) -current_pos(1)),1));
    
        % Transform the slope Variable such that it matched the angle
        % configuration recieved from the Vrep
        
        if slope > 3.135 | slope < -3.135
            slope = 0;
        else
            slope = slope + 3.14;
        end
        
        % Get the abgle of the Robot in Real Time from Vrep

        [returnCode,angle]=vrep.simxGetObjectOrientation(clientID,Robot,-1,vrep.simx_opmode_blocking);
        
        % Calculate the velocity of both weheels
        
        time = 0.5;
        vel_right = ((slope - angle(3))*dist_wheels)/(2*radius*time);
        vel_left = -vel_right;
        while abs(vel_right) > 2 
            time = time + 0.5 ;
            vel_right = ((slope - angle(3))*dist_wheels)/(2*radius*time);
            vel_left = -vel_right;
        end
        
        timeStamp = toc
        [x_dot y_dot theta_dot] = extract_velocity(vel_right, vel_left, slope)
        line = [timeStamp x_dot y_dot linear.z theta_dot angular.y angular.z];
        fprintf(fileID,'%6.2f %12.2f %18.2f %24.2f %30.2f %36.2f %42.2f\r\n', line);

        while (abs((slope) - angle(3))) > 0.04
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,vel_left,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,vel_right,vrep.simx_opmode_blocking);
            [returnCode,angle]=vrep.simxGetObjectOrientation(clientID,Robot,-1,vrep.simx_opmode_blocking);
        end
        
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,0,vrep.simx_opmode_blocking);

        time = 0.05;

        [returnCode,position]=vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_blocking);
        current_pos = [position(1),position(2)]; 

        vel_right = (norm(next_pos -current_pos))/time;
        vel_left=vel_right;
        
        timeStamp = toc
        [x_dot y_dot theta_dot] = extract_velocity(vel_right, vel_left, slope)
        line = [timeStamp x_dot y_dot linear.z theta_dot angular.y angular.z];
        fprintf(fileID,'%6.2f %12.2f %18.2f %24.2f %30.2f %36.2f %42.2f\r\n', line);
        
        while abs(vel_right) > 8 
            time = time + 0.01 ;
            vel_right = (norm(next_pos -current_pos))/time;
            vel_left = vel_right;
        end
        
        [returnCode,position]=vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_blocking);
        current_pos = [position(1),position(2)];    

        while ((norm(next_pos -current_pos)) > 0.09)
            norm(next_pos -current_pos)
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,vel_left,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,vel_right,vrep.simx_opmode_blocking);
            [returnCode,position]=vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_blocking);
            current_pos = [position(1),position(2)]
        end

        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,0,vrep.simx_opmode_blocking);

        % for position update
        [returnCode,position]=vrep.simxGetObjectPosition(clientID,Robot,-1,vrep.simx_opmode_blocking);

        current_pos = [position(1),position(2)];

    end
    vrep.simxFinish(-1);
end

vrep.delete();


vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if clientID > -1
    disp("Connected")
end

[~,left_motor]=vrep.simxGetObjectHandle(clientID,'wheel_left_joint',vrep.simx_opmode_blocking);
[~,right_motor]=vrep.simxGetObjectHandle(clientID,'wheel_right_joint',vrep.simx_opmode_blocking);

x = 0;
y = 0;

    


[~]=vrep.simxSetJointTargetVelocity(clientID,left_motor,,vrep.simx_opmode_blocking);
[~]=vrep.simxSetJointTargetVelocity(clientID,right_motor,5,vrep.simx_opmode_blocking);

vrep.delete();



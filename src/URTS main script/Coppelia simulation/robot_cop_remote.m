clc; clear; close all
sim = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1',19999,true,true,5000,5);

% disp('simu start')
% sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot);

if (clientID>-1)
    disp('Connected to remote API server');
    qr = load('Robot/data/Robot.mat').qr;
    [len1, len2] = size(qr);
    h = zeros(1, len1);
    r = zeros(1, len1);
    for i = 1 : len1
        objectPath = ['q' num2str(i)];
        [r(i), h(i)] = sim.simxGetObjectHandle(clientID, objectPath, sim.simx_opmode_blocking);
    end
    
    while 1
%         position = input(prompt);
        for i = 1 : len2
%             if mod(i, 1000) == 0
                disp(['i = ' num2str(i)])
%             end
            for j = 1 : len1
                position = qr(j, i);
%                 sim.simxSetJointTargetPosition(clientID, h(j), position, sim.simx_opmode_streaming);
                sim.simxSetJointPosition(clientID, h(j), position, sim.simx_opmode_streaming);
            end
        end        
    end
%     sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot)
%     sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot);
else
    disp('Failed connecting to remote API server');
end
% sim.delete(); % call the destructor!
disp('Program ended');
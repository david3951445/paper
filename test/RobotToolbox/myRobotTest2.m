% full robot
clc; clear; close all
rb = Robot();
robot = rb.rbtree;
%% FK, IK test
% config = homeConfiguration(robot);
% theta = [10 -20 -10 10 10 10]/180*pi;
% for i = 1 : length(theta) 
%     config(i) = theta(i);
% end
% tform = getTransform(robot,config,'body8','base') % forward kinemic
% 
% ik = inverseKinematics('RigidBodyTree',robot);
% initialguess = [0 -0 -0 0 0 0];
% weights = [0.25 0.25 0.25 1 1 1];
% [configSol,solInfo] = ik('body8',tform,weights,initialguess);
% disp('origin joints:')
% disp(theta)
% disp('IK joints')
% disp(configSol)
% show(robot, configSol)

%% [x y z phi theta psi] -> config
ik = inverseKinematics('RigidBodyTree', robot);
weights = [1 1 1 1 1 1];

% frame rotation between joint 1 and sole
eul = [-pi/2 -pi/2 0];
tform_r = eul2tform(eul);
% frame translation between joint 1 and sole
z = sum(rb.L(3:6))-.05;
n = 100;
x = linspace(-.1, .1, n);
theta = zeros(6, n);
pos = [x; .05*ones(1, n); -z*ones(1, n)];

tform = trvec2tform(pos(:, 1)')*tform_r;
initialguess = [0 -0 -.2 .2 0 -.2];
[configSol,~] = ik('body7',tform,weights,initialguess);
% configSol/pi*180
% tform = getTransform(robot,configSol,'body7','base')
theta(:, 1) = configSol;
% show(robot, configSol);

for i = 2 : n
    if mod(i, 10) == 0
        disp(['i = ', num2str(i)])
    end
    tform = trvec2tform(pos(:, i)')*tform_r;
    initialguess = configSol;
    [configSol,~] = ik('body7',tform,weights,initialguess);
%     configSol/pi*180
    % tform = getTransform(robot,configSol,'body7','base')
    theta(:, i) = configSol;
end
% show(robot, configSol);

t = linspace(0,1,n);
figure; hold on
for i = 1 : 6
    plot(t, theta(i, :), 'DisplayName', ['theta' num2str(i)])
end
legend

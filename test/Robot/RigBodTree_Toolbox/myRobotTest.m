% joint 7 to 3 
clc; clear; close all
JointLimit = [
    -90    90
    -90    90
    -90    90
]/180*pi;
robot = rigidBodyTree;
robot.DataFormat = 'row';

body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','fixed');
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','fixed');
body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
body5 = rigidBody('body5');
jnt5 = rigidBodyJoint('jnt5','fixed');
body6 = rigidBody('body6');
jnt6 = rigidBodyJoint('jnt6','revolute');

jnt2.PositionLimits = JointLimit(1, :);
jnt4.PositionLimits = JointLimit(2, :);
jnt6.PositionLimits = JointLimit(3, :);

rb = Robot();
dhparams = rb.DH_f;
setFixedTransform(jnt1,dhparams(1,:),'dh');
setFixedTransform(jnt2,dhparams(2,:),'dh');
setFixedTransform(jnt3,dhparams(3,:),'dh');
setFixedTransform(jnt4,dhparams(4,:),'dh');
setFixedTransform(jnt5,dhparams(5,:),'dh');
setFixedTransform(jnt6,dhparams(6,:),'dh');

body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;

addBody(robot,body1,'base') 
addBody(robot,body2,'body1')
addBody(robot,body3,'body2')
addBody(robot,body4,'body3')
addBody(robot,body5,'body4')
addBody(robot,body6,'body5')

%% FK, IK test
% config = homeConfiguration(robot);
% theta = [10 -10 -10]/180*pi;
% for i = 1 : 3 
%     config(i) = theta(i);
% end
% tform = getTransform(robot,config,'body6','base') % forward kinemic
% 
% ik = inverseKinematics('RigidBodyTree',robot);
% initialguess = [0 -0 -0];
% weights = [0.25 0.25 0.25 1 1 1];
% [configSol,solInfo] = ik('body6',tform,weights,initialguess);
% disp('origin joints:')
% disp(theta)
% disp('IK joints')
% disp(configSol)

%% [phi theta psi x y z] -> config

%% animation
% len = 10;
% z = rb.height_CoM*ones(1, len);
% y = zeros(1, len);
% x = linspace(-.1, .1, len);
% trvec = [x; y; z];
% trvec = timeseries(trvec');
% 
% ik = inverseKinematics('RigidBodyTree',robot);
% weights = [0.25 0.25 0.25 1 1 1];
% initialguess = [0 -0 -0];
% tform = cell(1, len);
% configSol = cell(1, len);
% for i = 1 : 1%len
%     disp(['i=' num2str(i)])
%     pos = [.1 0 .3]
%     pos = trvec(:, i)';
%     tform{i} = trvec2tform(pos);
%     [configSol{i},solInfo] = ik('body6',tform{i},weights,initialguess);
%     show(robot, configSol{i})
%     title(['i=' num2str(i)])
%     drawnow
%     initialguess = configSol{i};
% end

% full robot
clc; clear; close all
JointLimit = [
    -180 180
    -90 90
]/180*pi;
robot = rigidBodyTree;
robot.DataFormat = 'row';

body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','fixed');
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
body5 = rigidBody('body5');
jnt5 = rigidBodyJoint('jnt5','revolute');
body6 = rigidBody('body6');
jnt6 = rigidBodyJoint('jnt6','revolute');
body7 = rigidBody('body7');
jnt7 = rigidBodyJoint('jnt7','revolute');
% body8 = rigidBody('body8');
% jnt8 = rigidBodyJoint('jnt8','fixed');

% jnt1.PositionLimits = JointLimit(1, :);
% jnt2.PositionLimits = JointLimit(2, :);
% jnt3.PositionLimits = JointLimit(2, :);
% jnt4.PositionLimits = JointLimit(2, :);
% jnt5.PositionLimits = JointLimit(2, :);
% jnt6.PositionLimits = JointLimit(2, :);

rb = Robot();
dhparams = rb.DH;
setFixedTransform(jnt1,dhparams(1,:),'dh');
setFixedTransform(jnt2,dhparams(2,:),'dh');
setFixedTransform(jnt3,dhparams(3,:),'dh');
setFixedTransform(jnt4,dhparams(4,:),'dh');
setFixedTransform(jnt5,dhparams(5,:),'dh');
setFixedTransform(jnt6,dhparams(6,:),'dh');
setFixedTransform(jnt7,dhparams(7,:),'dh');
% setFixedTransform(jnt8,dhparams(8,:),'dh');

body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;
body7.Joint = jnt7;
% body8.Joint = jnt8;

addBody(robot,body1,'base') 
addBody(robot,body2,'body1')
addBody(robot,body3,'body2')
addBody(robot,body4,'body3')
addBody(robot,body5,'body4')
addBody(robot,body6,'body5')
addBody(robot,body7,'body6')
% addBody(robot,body8,'body7')

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

%% [phi theta psi x y z] -> config
eul = [pi/2 -pi/2 0];
tform = eul2tform(eul); % rotation
z = sum(rb.L(3:6))-.05;
pos = [.05 .05 -z];
tform = trvec2tform(pos)*tform % translation

% theta = [0 -0 .4 -.8 .0 .4];
% show(robot, theta);
% for i = 1 : length(theta) 
%     config(i) = theta(i);
% end
% tform = getTransform(robot,config,'body7','base')

ik = inverseKinematics('RigidBodyTree',robot);
initialguess = [0 -0 -.2 .2 0 -.2];
weights = [1 1 1 1 1 1];
[configSol,solInfo] = ik('body7',tform,weights,initialguess);
configSol
tform = getTransform(robot,configSol,'body7','base')
% disp('IK joints')
% disp(configSol)
show(robot, configSol);
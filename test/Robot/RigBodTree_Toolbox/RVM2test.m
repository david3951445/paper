clc; clear; close all
% a, alpha, d, theta
% PUMA560
% dhparams = [0   	pi/2	0   	0;
%             0.4318	0       0       0
%             0.0203	-pi/2	0.15005	0;
%             0   	pi/2	0.4318	0;
%             0       -pi/2	0   	0;
%             0       0       0       0];
% RV-M2 robot manipulator
dhparams = [
    .12 -pi/2  0  0
    .25  0     0  0
    .26  0     0  0
    0    -pi/2 0  0
    0    pi/2  0  0
    0    0     0  0
];
JointLimit = [-150   150
                 -30    100
                 -120   0
                 -110   110
                 -180   180
                 -180   180]/180*pi;
robot = rigidBodyTree;
robot.DataFormat = 'row';

body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
body5 = rigidBody('body5');
jnt5 = rigidBodyJoint('jnt5','revolute');
body6 = rigidBody('body6');
jnt6 = rigidBodyJoint('jnt6','revolute');

jnt = {jnt1, jnt2, jnt3, jnt4, jnt5, jnt6};
for i = 1 : 6
    jnt{i}.PositionLimits = JointLimit(i, :);
end

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

show(robot)
config = homeConfiguration(robot);
theta = [10 -10 -10 -10 10 -10]/180*pi;
for i = 1 : 6  
    config(i) = theta(i);
end
tform = getTransform(robot,config,'body6','base') % forward kinemic

ik = inverseKinematics('RigidBodyTree',robot);
initialguess = [0 -0 -0 -0 0 -0];
weights = [0.25 0.25 0.25 1 1 1];
[configSol,solInfo] = ik('body6',tform,weights,initialguess);
disp('origin joints:')
disp(theta)
disp('IK joints')
disp(configSol)
function rb = get_rbtree(rb)
%Construct robot rigidbody tree

robot = rigidBodyTree;  
robot.DataFormat = 'row';

%%-1 base
% construct body and it's joint
body = rigidBody('body0');
jnt = rigidBodyJoint('f0', 'fixed');
% construct body and it's joint
setFixedTransform(jnt, eye(4));
body.Joint = jnt;
body.Mass = rb.MASS(1);
body.Inertia = rb.INERTIA(1, :);
% add body to rigidbodytree
addBody(robot, body, 'base')

%%-2 base to q1 (left)
body = rigidBody('body_f1');
jnt = rigidBodyJoint('f1', 'fixed');
setFixedTransform(jnt, [rb.L(1) 0 -rb.L(2) pi/2], 'dh')
body.Joint = jnt;
body.Mass = 0;
body.Inertia = zeros(1, 6);
addBody(robot, body, 'body0')

%%-3 base to q2 (right)
body = rigidBody('body_f2');
jnt = rigidBodyJoint('f2', 'fixed');
% construct body and it's joint
setFixedTransform(jnt, [-rb.L(1) 0 -rb.L(2) pi/2], 'dh')
body.Joint = jnt;
body.Mass = 0;
body.Inertia = zeros(1, 6);
addBody(robot, body, 'body0')

%%-4 two legs
JOINT_LIMIT = [
    rb.JNT_LIMIT(1, :)
    0 0
    rb.JNT_LIMIT(2, :)
    rb.JNT_LIMIT(3, :)
    rb.JNT_LIMIT(4, :)
    rb.JNT_LIMIT(5, :)
    rb.JNT_LIMIT(6, :)
];
JOINT_TYPE = {'revolute', 'fixed', 'revolute', 'revolute', 'revolute', 'revolute', 'revolute'};
BODY_MASS = [rb.MASS(2), 0, rb.MASS(3), rb.MASS(4), rb.MASS(5), rb.MASS(6), rb.MASS(7)];

%%-4-1 left foot
BODY_NAME = {'body1', 'body_f3', 'body3', 'body5', 'body7', 'body9', 'body11'};
JOINT_NAME = {'jnt1', 'f3', 'jnt3', 'jnt5', 'jnt7', 'jnt9', 'jnt11'};
BODY_INERTIA = {rb.INERTIA(2, :), zeros(1, 6), rb.INERTIA(3, :), rb.INERTIA(4, :), rb.INERTIA(6, :), rb.INERTIA(8, :), rb.INERTIA(9, :)};
for i = 1 : 7
    body = rigidBody(BODY_NAME{i});
    jnt = rigidBodyJoint(JOINT_NAME{i}, JOINT_TYPE{i});
    if strcmp(JOINT_TYPE{i}, 'revolute')
        jnt.PositionLimits = JOINT_LIMIT(i, :);
    end
    setFixedTransform(jnt, rb.DH(i, :), 'dh')
    body.Joint = jnt;
    body.Mass = BODY_MASS(i);
    body.Inertia = BODY_INERTIA{i};
    if i == 1
        addBody(robot, body, 'body_f1');
    else
        addBody(robot, body, BODY_NAME{i-1});
    end
end
% showdetails(robot)

%%-4-1 right foot
BODY_NAME = {'body2', 'body_f4', 'body4', 'body6', 'body8', 'body10', 'body12'};
JOINT_NAME = {'jnt2', 'f4', 'jnt4', 'jnt6', 'jnt8', 'jnt10', 'jnt12'};
BODY_INERTIA = {rb.INERTIA(2, :), zeros(1, 6), rb.INERTIA(3, :), rb.INERTIA(5, :), rb.INERTIA(7, :), rb.INERTIA(8, :), rb.INERTIA(9, :)};
for i = 1 : 7
    body = rigidBody(BODY_NAME{i});
    jnt = rigidBodyJoint(JOINT_NAME{i}, JOINT_TYPE{i});
    if strcmp(JOINT_TYPE{i}, 'revolute')
        jnt.PositionLimits = JOINT_LIMIT(i, :);
    end
    setFixedTransform(jnt, rb.DH(i, :), 'dh')
    body.Joint = jnt;
    body.Mass = BODY_MASS(i);
    body.Inertia = BODY_INERTIA{i};
    if i == 1
        addBody(robot, body, 'body_f2');
    else
        addBody(robot, body, BODY_NAME{i-1});
    end
end

%%-5 others
robot.Gravity = [0 0 -9.8];
c = centerOfMass(robot);

rb.height_CoM        = rb.height_CoM0_stand + c(3);     
rb.rbtree            = robot;
end
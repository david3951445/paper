classdef Robot
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        % MASS = [6.869 .243 .243 1.045 1.045 3.095 3.095 2.401 2.401 1.045 1.045 .223 .223];
        MASS = [6.869 .243 1.045 3.095 2.401 1.045 .223];
        L = [.035 .0907 .0285 .11 .11 .0305];
        JNT_LIMIT = [
            -60 60
            -90 90
            -90 90
            0 160
            -90 90
            -90 90
        ]/180*pi;
        OFFSER_COM_STAND_WALK = .03; % remain an offset between CoM of standing and walking to let feet reach end point possibly
        height_feet = .01; % The highest point of a feet step
        INERTIA = [
            10^(-2)*[3.603 3.31 3.83 0 0 0]
            10^(-4)*[2 10 9 0 0 0]
            10^(-4)*[6 17 17 0 0 0]
            10^(-4)*[433 404 56 -20 29 3]
            10^(-4)*[433 404 56 20 29 -3]
            10^(-4)*[197 196 57 -14 -29 3]
            10^(-4)*[197 196 57 14 29 -3]
            10^(-4)*[6 17 17 0 0 0]
            10^(-5)*[22 99 91 0 -.1 0]
        ];
    end
    properties
        DH % DH table of leg (joint 1 -> 11 (2 -> 12))
        DH_f2 % DH table of leg (joint 7 -> 3 (8 -> 4))
        
        height_CoM % height of CoM0
        height_CoM0_stand % height of CoM when standing
        height_CoM0_walk % height of CoM when walking

        rbtree % rigidbodytree
        rbtree_f % full robot
    end
    methods
        function rb = Robot()
            % rb.DH = [
            %     0 0 0 0 % q1
            %     0 pi/2 -rb.L(3) 0 % q3
            %     0 0 0 pi/2
            %     0 pi/2 0 0 % q5
            %     -rb.L(4) 0 0 0 % q7
            %     -rb.L(5) -pi/2 0 0 % q9
            %     0 pi/2 0 0 % q11
            %     -rb.L(6) 0 0 0 % fixed
            % ];
            rb.DH = [
                0 pi/2 -rb.L(3) 0 % q1
                0 0 0 pi/2 % fixed
                0 pi/2 0 0 % q3
                -rb.L(4) 0 0 0 % q5
                -rb.L(5) -pi/2 0 0 % q7
                0 pi/2 0 0 % q9
                -rb.L(6) 0 0 0 % q11
            ];
            rb.DH_f2 = [
                0 -pi/2 rb.L(5) 0 % fixed
                0 0 0 0 % q7
                0 pi/2 0 0 % fixed
                0 -pi/2 rb.L(4) 0 % q5
                0 pi/2 0 -pi/2 % fixed
                0 0 0 0 % q3
            ];         
            
            %% construct robot rigidbody tree
            robot = rigidBodyTree;  
            robot.DataFormat = 'row';
            
            %% base
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

            %% base to q1 (left)
            body = rigidBody('body_f1');
            jnt = rigidBodyJoint('f1', 'fixed');
            setFixedTransform(jnt, [rb.L(1) 0 -rb.L(2) pi/2], 'dh')
            body.Joint = jnt;
            body.Mass = 0;
            body.Inertia = zeros(1, 6);
            addBody(robot, body, 'body0')

            %% base to q2 (right)
            body = rigidBody('body_f2');
            jnt = rigidBodyJoint('f2', 'fixed');
            % construct body and it's joint
            setFixedTransform(jnt, [-rb.L(1) 0 -rb.L(2) pi/2], 'dh')
            body.Joint = jnt;
            body.Mass = 0;
            body.Inertia = zeros(1, 6);
            addBody(robot, body, 'body0')

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

            %% left foot
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

            %% right foot
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
            robot.Gravity = [0 0 -9.8];
            c = centerOfMass(robot);

            rb.rbtree            = robot;
            rb.height_CoM0_stand = sum(rb.L(2:6));
            rb.height_CoM0_walk  = rb.height_CoM0_stand - rb.OFFSER_COM_STAND_WALK;
            rb.height_CoM        = rb.height_CoM0_stand + c(3);     
         
        end
        
        function rb = get_rbtree(rb) 
        end

        function y = m(rb, i)
            switch i
                case 0
                    y = rb.MASS(1);
                case {1, 2}
                    y = rb.MASS(2);
                case {3, 4}
                    y = rb.MASS(3);
                case {5, 6}
                    y = rb.MASS(4);
                case {7, 8}
                    y = rb.MASS(5);
                case {9, 10}
                    y = rb.MASS(6);
                case {11, 12}
                    y = rb.MASS(7);
            end
        end
    end
end


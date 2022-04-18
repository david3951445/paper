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
    end
    properties
        DH % DH table of leg (joint 1 -> 11 (2 -> 12))
        DH_f2 % DH table of leg (joint 7 -> 3 (8 -> 4))
        height_CoM % height of CoM of LIPM
        height_feet = .01; % The highest point of a feet step
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
            rb.height_CoM = sum(rb.L(2:6)) - .05; % remain .05 to let feet reach end point possibly
            
            % construct robot rigidbody tree
            % JointLimit = [
            %     -60 60
            %     0 0
            %     -90 90
            %     -90 90
            %     0 160
            %     -90 90
            %     -90 90
            % ]/180*pi;
            % robot = rigidBodyTree;  
            % robot.DataFormat = 'row';

            % JOINT_TYPE = {'revolute', 'fixed', 'revolute', 'revolute', 'revolute', 'revolute', 'revolute'};
            % BODY_MASS = [rb.MASS(2), 0, rb.MASS(3), rb.MASS(4), rb.MASS(5), rb.MASS(6), rb.MASS(7)];
            % for i = 1 : 7
            %     bodyName = ['body' num2str(i)];
            %     jointName = ['jnt' num2str(i)];

            %     % construct body and it's joint
            %     body = rigidBody(bodyName);
            %     jnt = rigidBodyJoint(jointName, JOINT_TYPE{i});
            %     if strcmp(JOINT_TYPE{i}, 'revolute')
            %         jnt.PositionLimits = JointLimit(i, :);
            %     end
            %     setFixedTransform(jnt,rb.DH(i, :),'dh')
            %     body.Joint = jnt;
            %     body.Mass = BODY_MASS(i);

            %     % add body to rigidbodytree
            %     if i == 1
            %         addBody(robot, body, 'base')
            %     else
            %         addBody(robot, body, ['body' num2str(i-1)])
            %     end
            % end
            % rb.rbtree = robot;

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
            % add body to rigidbodytree
            addBody(robot, body, 'base')

            %% base to q1 (left)
            body = rigidBody('body_f1');
            jnt = rigidBodyJoint('f1', 'fixed');
            setFixedTransform(jnt, [rb.L(1) 0 -rb.L(2) pi/2], 'dh')
            body.Joint = jnt;
            body.Mass = 0;
            addBody(robot, body, 'body0')

            %% base to q2 (right)
            body = rigidBody('body_f2');
            jnt = rigidBodyJoint('f2', 'fixed');
            % construct body and it's joint
            setFixedTransform(jnt, [-rb.L(1) 0 -rb.L(2) pi/2], 'dh')
            body.Joint = jnt;
            body.Mass = 0;
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
            for i = 1 : 7
                body = rigidBody(BODY_NAME{i});
                jnt = rigidBodyJoint(JOINT_NAME{i}, JOINT_TYPE{i});
                if strcmp(JOINT_TYPE{i}, 'revolute')
                    jnt.PositionLimits = JOINT_LIMIT(i, :);
                end
                setFixedTransform(jnt, rb.DH(i, :), 'dh')
                body.Joint = jnt;
                body.Mass = BODY_MASS(i);
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
            for i = 1 : 7
                body = rigidBody(BODY_NAME{i});
                jnt = rigidBodyJoint(JOINT_NAME{i}, JOINT_TYPE{i});
                if strcmp(JOINT_TYPE{i}, 'revolute')
                    jnt.PositionLimits = JOINT_LIMIT(i, :);
                end
                setFixedTransform(jnt, rb.DH(i, :), 'dh')
                body.Joint = jnt;
                body.Mass = BODY_MASS(i);
                if i == 1
                    addBody(robot, body, 'body_f2');
                else
                    addBody(robot, body, BODY_NAME{i-1});
                end
            end

            rb.rbtree = robot;

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


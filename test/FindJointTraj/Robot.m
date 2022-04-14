classdef Robot
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        % m = [6.869 .243 .243 1.045 1.045 3.095 3.095 2.401 2.401 1.045 1.045 .223 .223];
        m = [6.869 .243 1.045 3.095 2.401 1.045 .223];
        L = [.035 .0907 .0285 .11 .11 .0305];
    end
    properties
        DH % DH table of leg (joint 1 -> 11 (2 -> 12))
        DH_f2 % DH table of leg (joint 7 -> 3 (8 -> 4))
        height_CoM % height of CoM of LIPM
        height_feet = .01; % The highest point of a feet step
        rbtree % rigidbodytree
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
                0 pi/2 0 pi/2 % fixed
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
            JointLimit = [
                -60 60
                0 0
                -90 90
                -90 90
                0 160
                -90 90
                -90 90
            ]/180*pi;
            robot = rigidBodyTree;  
            robot.DataFormat = 'row';

            JOINT_TYPE = {'revolute', 'fixed', 'revolute', 'revolute', 'revolute', 'revolute', 'revolute'};
            for i = 1 : 7
                bodyName = ['body' num2str(i)];
                jointName = ['jnt' num2str(i)];

                % construct body and it's joint
                body = rigidBody(bodyName);
                jnt = rigidBodyJoint(jointName, JOINT_TYPE{i});
                if strcmp(JOINT_TYPE{i}, 'revolute')
                    jnt.PositionLimits = JointLimit(i, :);
                end
                setFixedTransform(jnt,rb.DH(i, :),'dh')
                body.Joint = jnt;

                % add body to rigidbodytree
                if i == 1
                    addBody(robot, body, 'base')
                else
                    addBody(robot, body, ['body' num2str(i-1)])
                end
            end
            rb.rbtree = robot;
        end
        
        function rb = get_rbtree(rb) 
        end
    end
end


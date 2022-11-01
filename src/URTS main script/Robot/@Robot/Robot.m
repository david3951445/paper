classdef Robot < Agent
    % 12 link robot model
    
    properties (Constant)
        % MASS = [6.869 .243 .243 1.045 1.045 3.095 3.095 2.401 2.401 1.045 1.045 .223 .223];
        MASS = [6.869 .243 1.045 3.095 2.401 1.045 .223]
        L = [.035 .0907 .0285 .11 .11 .0305]
        JNT_LIMIT = [
            -60 60
            -90 90
            -90 90
            0 160
            -90 90
            -90 90
        ]/180*pi
        OFFSER_COM_STAND_WALK = .03 % remain an offset between CoM of standing and walking to let feet reach end point possibly
        height_feet = .01 % The highest point of a feet step
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
        ] % inertia of CoM1~12

        INTERP_DENSITY = 1500 % interp density of zmp
        UNIT = {'rad', 'rad', 'rad', 'rad', 'rad', 'rad', 'rad', 'rad', 'rad', 'rad', 'rad', 'rad'}
    end
    properties
        %% rigidbodytree
        DH % DH table of leg (joint 1 -> 11 (2 -> 12))
        DH_f2 % DH table of leg (joint 7 -> 3 (8 -> 4))
        
        height_CoM % height of CoM0
        height_CoM0_stand % height of CoM when standing
        height_CoM0_walk % height of CoM when walking

        rbtree % rigidbodytree of robot

        %% reference design
        r % task space ref traj
        r_lr
        zmp
        CoM
        qr % joint space ref traj

        %% flow control of code
        EXE_Z2C = 1 % ZMP to CoM converter
        EXE_IK = 1 % inverse dynamic
    end

    methods
        function rb = Robot()
            rb@Agent();
            rb.PATH  = ['data/' mfilename]; % path of saved data
            rb = rb.Load(); % load old data
            rb.DIM_F = 12;
            
            %% D-H table
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
  
            rb.height_CoM0_stand = sum(rb.L(2:6));
            rb.height_CoM0_walk  = rb.height_CoM0_stand - rb.OFFSER_COM_STAND_WALK;
            rb = get_rbtree(rb); % Construct robot rbtree
            % rb = GRF(rb); % Ground Reaction Force
        end
        
        function y = M(rb, x)
            y = rb.rbtree.massMatrix(x');
        end

        function y = H(rb, x, dx)
            CG = rb.rbtree.velocityProduct(x', dx') + rb.rbtree.gravityTorque(x');
            y = CG';
        end

        rb = get_rbtree(rb) % rigidBodyTree setting
        y = m(rb, i) % Just a mapping between m and MASS
        rb = Ref2Config(rb, r) % Task space ref to joint space configuration ref
        y = GRF(rb)
    end
end


classdef Robot
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
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
        dt = .001

        DIM_F = 12 % dimension of state (pos)
        WINDOW = 3 % looking forward window of unknown signal

        INTERP_DENSITY = 2500 % interp density of zmp

        PATH = ['data/' mfilename]
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

        %% Control design
        sys
        sys_a
        sys_s
        sys_aug

        K  % Control gain matrix
        KL  % Observer gain matrix
        DIM

        %% trajectories
        tr 
    end

    methods
        function rb = Robot()
            %% Load old data
            rb = rb.Load(); % load old data

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
        end
        
        function y = M(rb, x)
            y = rb.rbtree.massMatrix(x');
        end

        function y = H(rb, x, dx)
            CG = rb.rbtree.velocityProduct(x', dx') + rb.rbtree.gravityTorque(x');
            y = CG';
        end

        function y = f_aug(rb, t, xb)
            DIM_X3 = rb.sys_aug.DIM_X;
            x = xb(1 : DIM_X3);
            xh = xb(DIM_X3 + (1:DIM_X3));
            u = rb.u_PID(xh);
            y = [
                rb.sys_aug.A*x + rb.sys_aug.B*u
                rb.sys_aug.A*xh + rb.sys_aug.B*u - rb.KL*rb.sys_aug.C*(x-xh)
            ];
        end

        function y = u_PID(rb, xh)
            % range = 1 : rb.DIM_F*3; 
            % y = rb.K(:, range)*xh(range); % without unknown sigal info
            y = rb.K*xh; 
        end

        rb = get_rbtree(rb) % rigidBodyTree setting
        y = m(rb, i) % Just a mapping between m and MASS
        rb = Ref2Config(rb) % Task space ref to joint space configuration ref
        rb = trajectory(rb)
        Save(rb, filename, whichVar) % Save property
        rb = Load(rb, filename) % Load property
    end
end


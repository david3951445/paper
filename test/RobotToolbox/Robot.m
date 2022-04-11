classdef Robot
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        % m = [6.869 .243 .243 1.045 1.045 3.095 3.095 2.401 2.401 1.045 1.045 .223 .223];
        m = [6.869 .243 1.045 3.095 2.401 1.045 .223];
        L = [.035 .0907 .0285 .11 .11 .0305];
    end
    properties
        DH % base to joint 11 (12)
        DH_f % DH table of leg (joint 1 -> 11 (2 -> 12))
        DH_f2 % DH table of leg (joint 7 -> 3 (8 -> 4))
        height_CoM % height of CoM of LIPM
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
                0 -pi/2 0 pi/2 % fixed
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
        end
        
        function rb = get_rbtree(rb) 
        end
    end
end


% reference model
classdef REF
    properties (Constant)
        c = 1*[10, 5, 10, 5, 10, 5]
        g = 9.81
        %         radius = 10;
        %         freq = 0.5;
    end
    
    properties
        A
        B

        m % mass of UAV
    end
    
    methods
        function obj = REF(uav)
            obj.A = -10*eye(uav.dim);
            obj.B = -obj.A;

            obj.m = uav.m;
        end
        
        function y = r(obj, x, F, t)
            amp   = 1; % 1
            amp_z = 1; % 0.8
            freg     = 0.5;

            y = [
                amp*sin(freg*t)
                freg*amp*cos(freg*t)
                amp*cos(freg*t)
                -freg*amp*sin(freg*t)
                amp_z*t
                amp_z
                0
                0
                0
                0
                0
                0
            ];
            
            % method 1-1
            ux = obj.c(1)*(x(1) - y(1)) + obj.c(2)*(x(2) - y(2));
            uy = obj.c(3)*(x(3) - y(3)) + obj.c(4)*(x(4) - y(4));
            uz = obj.c(5)*(x(5) - y(5)) + obj.c(6)*(x(6) - y(6));        
            % a = 1/(uz + obj.g);
            % phi_d = x(11);
            % % y(7) = asin(obj.m/F*(ux*sin(x(11)) - uy*cos(x(11))));
            % y(7) = atan(a*(ux*sin(phi_d) - uy*cos(phi_d)));
            % y(9) = atan(a*(ux*cos(phi_d) + uy*sin(phi_d))/cos(x(7)));

            % method 1-2 (good)
            y(7) = atan(-freg*cos(freg*t)/obj.g);
            y(9) = atan(freg*sin(freg*t)/obj.g*cos(y(7)));

            % method 1-3
            % y(7) = 0.4*atan(obj.m*(-uy)/(ux + uy + uz));
            % y(9) = 0.4*atan(ux/(uz + obj.g));
            
            % method 1-4 (李博's method)         
            % y(7) = atan(ux*sin(y(11))-1*uy*cos(y(11))); % roll
            % % y(8) = (y(7)-f1(7))/h;
            % y(9) = atan((ux*cos(y(11))+uy*sin(y(11)))/cos(y(11))); % pitch
            % % y(10) = (y(9)-f1(9))/h;


        end
    end
end


% reference model
classdef REF
    properties
        c = 1*[10, 5, 10, 5, 10, 5];
    end
    
    properties
        A, B, m, g
    end
    
    methods
        function obj = REF(uav)
            I = eye(uav.dim);
            obj.A = -10*I;
            obj.B = -obj.A;
            obj.m = uav.m;
            obj.g = uav.G; 
        end
        
        function y = r(obj, x, F, t)
            y = [sin(0.5*t);
                 0.5*cos(0.5*t);
                 cos(0.5*t);
                 -0.5*sin(0.5*t);
                 0.8*t;
                 0.8;
                 0;
                 0;
                 0;
                 0;
                 0;
                 0];
            
            ux = obj.c(1)*(x(1) - y(1)) + obj.c(2)*(x(2) - y(2));
            uy = obj.c(3)*(x(3) - y(3)) + obj.c(4)*(x(4) - y(4));
            uz = obj.c(5)*(x(5) - y(5)) + obj.c(6)*(x(6) - y(6));
            
%             y(7) = asin(obj.m/F*(ux*sin(x(11)) - uy*cos(x(11))));
            a = 1/(uz + obj.g);
            phi_d = x(11);
%             y(7) = atan(a*(ux*sin(phi_d) - uy*cos(phi_d)));
%             y(9) = atan(a*(ux*cos(phi_d) + uy*sin(phi_d))/cos(x(7)));
            y(7) = atan(-0.25*cos(0.5*t)/obj.g);
            y(9) = atan(0.25*sin(0.5*t)/obj.g*cos(y(7)));
        end
    end
end


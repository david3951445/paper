% reference model
classdef REF
    properties
        c = 1*[10, 5, 10, 5, 10, 5];
    end
    
    properties
        A, B, m, g
        xd, yd, zd % desired trajectory
    end
    
    methods
        function obj = REF(uav)
            I = eye(uav.dim);
            obj.A = -10*I;
            obj.B = -obj.A;
            obj.m = uav.m;
            obj.g = uav.G;
            
            syms t
            obj.xd = sin(0.5*t);
            obj.yd = cos(0.5*t);
            obj.zd = 0.5*t;
        end
        
        function y = r(obj, t)
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
            
            y(9) = atan(-0.25*sin(0.5*t)/obj.g);
            y(7) = atan(0.25*cos(0.5*t)*cos(y(9))/obj.g);
            
        end
    end
end


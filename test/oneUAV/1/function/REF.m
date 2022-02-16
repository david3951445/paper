% reference model
classdef REF
    properties (Constant)
        c = 1*[10, 5, 10, 5, 10, 5];
        %         radius = 10;
        %         freq = 0.5;
    end
    
    properties
        A, B, m, g
    end
    
    methods
        function obj = REF(uav)
            I = eye(uav.dim);
            obj.A = -2*I;
            obj.B = -obj.A;
            obj.m = uav.m;
            obj.g = uav.G;
        end
        
        function y = r(obj, x, F, t)
            radius = 10; % 1
            radius_z = 1; % 0.8
            freg = 0.5;
            y = [radius*sin(freg*t)
                freg*radius*cos(freg*t)
                radius*cos(0.5*t);
                -0.5*radius*sin(0.5*t);
                radius_z*t;
                radius_z;
                0;
                0;
                0;
                0;
                0;
                0];
            
            % method 1-1
            % ux = obj.c(1)*(x(1) - y(1)) + obj.c(2)*(x(2) - y(2));
            % uy = obj.c(3)*(x(3) - y(3)) + obj.c(4)*(x(4) - y(4));
            % uz = obj.c(5)*(x(5) - y(5)) + obj.c(6)*(x(6) - y(6));        
            % % a = 1/(uz + obj.g);
            % phi_d = x(11);
            % y(7) = asin(obj.m/F*(ux*sin(x(11)) - uy*cos(x(11))));
            % y(7) = atan(a*(ux*sin(phi_d) - uy*cos(phi_d)));
            % y(9) = atan(a*(ux*cos(phi_d) + uy*sin(phi_d))/cos(x(7)));

            % method 1-2
            % y(7) = atan(-0.25*cos(0.5*t)/obj.g);
            % y(9) = atan(0.25*sin(0.5*t)/obj.g*cos(y(7)));
            
            % method 2 (李博's method)
            d1(1) = x(1)-y(1);  %% x-xd
            d1(2) = x(2)-y(2);    %% x_dot-x_dotd
            d1(3) = x(3)-y(3);  %% y-yd
            d1(4) = x(4)-y(4);    %% y_dot-y_dotd
            ux = 5*d1(1)+1.5*d1(2);
            uy = 5*d1(3)+1.5*d1(4);
            
            y(7) = 20*atan(ux*sin(y(11))-1*uy*cos(y(11))); % roll
            %   y(8) = (y(7)-f1(7))/h;
            y(9) = -30*atan((ux*cos(y(11))+uy*sin(y(11)))/cos(y(11))); % pitch
            %   y(10) = (y(9)-f1(9))/h;
        end
    end
end


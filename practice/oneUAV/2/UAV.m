% it's a simplify version of UAV model
classdef UAV  
    properties (Constant)
        % system parameters
        m = 2, l = 0.2, b = 2, d = 5, G = 9.81
        Jx = 1.25, Jy = 1.25, Jz = 2.2
        Kx = 0.01, Ky = 0.01, Kz = 0.01
        Kph = 0.012, Kth = 0.012, Kps = 0.012
        
        dim = 12  % dimension of state
        dim_u = 4 % dimension of control       
    end   
    
    properties
        x % trajectory
        A, B, K
        uo % feedforward
%         c % for calculation speed
    end
    
    % system functions
    methods
        function obj = UAV()
        end
        
        function y = f(obj, x)  
%             y = [x(2) ; obj.Kx*x(2)/obj.m;
%                  x(4) ; obj.Ky*x(4)/obj.m;
%                  x(6) ; obj.Kz*x(6)/obj.m - obj.G;
%                  x(8) ; ((obj.Jy - obj.Jz)*x(9)*x(11) - obj.Kph*x(8))/obj.Jx;
%                  x(10); ((obj.Jz - obj.Jx)*x(7)*x(11) - obj.Kth*x(10))/obj.Jy;
%                  x(12); ((obj.Jx - obj.Jy)*x(7)*x(9) - obj.Kps*x(12))/obj.Jz;];
            y = [   x(2) ; 0;
                    x(4) ; 0;
                    x(6) ; -obj.G;
                    x(8) ; (obj.Jy - obj.Jz)*x(9)*x(11)/obj.Jx;
                    x(10); (obj.Jz - obj.Jx)*x(7)*x(11)/obj.Jy;
                    x(12); (obj.Jx - obj.Jy)*x(7)*x(9)/obj.Jz;  ];
        end
        
        function y = g(obj, x)
%             y = zeros(obj.dim, obj.dim_u);
            y(2, 1)  = (cos(x(7))*sin(x(9))*cos(x(11)) + sin(x(7))*sin(x(11)))/obj.m;
            y(4, 1)  = (cos(x(7))*sin(x(9))*cos(x(11)) - sin(x(7))*cos(x(11)))/obj.m;
            y(6, 1)  = cos(x(7))*cos(x(9))/obj.m;
            y(8, 2)  = 1/obj.Jx;
            y(10, 3) = 1/obj.Jy;
            y(12, 4) = 1/obj.Jz;
        end
    end
end


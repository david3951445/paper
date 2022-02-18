classdef UAV  
%it's a simplify model for UAV
% sys
%       dx/dt = f(x) + g(x)*u + Ev
    properties (Constant)
        % system parameters
        m = 2, l = 0.2, b = 2, d = 5, G = 9.81
        Jx = 1.25, Jy = 1.25, Jz = 2.2
        Kx = 0.01, Ky = 0.01, Kz = 0.01
        Kph = 0.012, Kth = 0.012, Kps = 0.012
        
        dim = 12  % dimension of state
        dim_u = 4 % dimension of control 

        E = eye(12) % disturbance matrix
        O = zeros(12);    
    end   
    
    properties
        A
        B
        K
    end
    
    % system functions
    methods
        function obj = UAV(fz)
            % load old data
            UAV_MAT = load('data/uav.mat');
            obj.A = UAV_MAT.A;
            obj.B = UAV_MAT.B;
            obj.K = UAV_MAT.K;

            %% find A, B (linearize)
            if EXE.A_B
                obj = obj.getAB(fz);
                obj.save('data/uav.mat', 'A')
                obj.save('data/uav.mat', 'B')  
            end        
        end
        
        function y = f(obj, x)  
            y = [x(2) ; -obj.Kx*x(2)/obj.m;
                 x(4) ; -obj.Ky*x(4)/obj.m;
                 x(6) ; -obj.Kz*x(6)/obj.m - obj.G;
                 x(8) ; ((obj.Jy - obj.Jz)*x(9)*x(11) - obj.Kph*x(8))/obj.Jx;
                 x(10); ((obj.Jz - obj.Jx)*x(7)*x(11) - obj.Kth*x(10))/obj.Jy;
                 x(12); ((obj.Jx - obj.Jy)*x(7)*x(9) - obj.Kps*x(12))/obj.Jz;];
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

        % Save property
        function save(obj, fname, whichVar)
            switch whichVar
                case 'A'
                    A = obj.A;
                case 'B'
                    B = obj.B;
                case 'K'
                    K = obj.K;
                otherwise
                    disp('No such property in UAV')
            end

            if isfile(fname)
                save(fname, whichVar, '-append');
            else
                save(fname, whichVar);
            end
            
        end
    end

    methods (Access = private)
        function obj = getAB(obj, fz)  
            s = sym('s', [obj.dim, 1]);
            
            % find A
            obj.A = cell(1, fz.num);
            for k = 1 : fz.num
                obj.A{k} = zeros(obj.dim);   

                y = subs(obj.f(s), s(fz.PV), fz.set(:, k));               
                for i = 1 : obj.dim
                    [cf, u] = coeffs(y(i));
                    u = subs(u, s, (1:obj.dim)');

                    obj.A{k}(i, u) = cf;
                end
            end
        
            % find B
            obj.B = cell(1, fz.num);
            for k = 1 : fz.num
                obj.B{k} = zeros(obj.dim, obj.dim_u);

                y = subs(obj.g(s), s(fz.PV), fz.set(:, k));
                obj.B{k}(:,:) = subs(y, s(11), 0); % let phi = 0
            end
        end
    end
end
classdef UAV  
%it's a simplify model for UAV
% - System form
%       - dx/dt = f(x) + g(x)*u + Ev
% - The model is refered to 1.pdf
%       - Euler angle (extrinsic rotation)
%           - x(7)  : phi, x-axis
%           - x(9)  : theta, y-axis
%           - x(11) : psi, z-axis
%       - Typo in 1.pdf.
%           - x(7), x(9), x(11) are not roll, pitch, yaw respectively. 
%               - More detail : https://en.wikipedia.org/wiki/Rotation_matrix
%           - Coriolis term in dx(7)dt
%               ((obj.Jy - obj.Jz)*x(9)*x(11) - obj.Kph*x(8))/obj.Jx
%             need to be corrected to
%               ((obj.Jy - obj.Jz)*x(10)*x(12) - obj.Kph*x(8))/obj.Jx
%             dx(9)dt, dx(11)t as well.
%               - More detail
%                   Quadrotor control: modeling, nonlinear control design, and simulation (2.19)
%                   [https://www.diva-portal.org/smash/get/diva2:860649/FULLTEXT01.pdf]

    properties (Constant)
        % system parameters
        m = 2, l = 0.2, b = 2, d = 5, G = 9.81
        Jx = 1.25, Jy = 1.25, Jz = 2.2
        Kx = 0.01, Ky = 0.01, Kz = 0.01
        Kph = 0.012, Kth = 0.012, Kps = 0.012
        
        DIM_X = 12  % dimension of state
        DIM_U = 4 % dimension of control
    end   
    
    properties
        rho = 10^(1);
        Q   = 10^(-1)*diag([1, 0.001, 1.5, 0.002, 1, 0.001, 0.1, 0, 0.1, 0, 1, 0.001]); % correspond to x - xr
        E   = 10^(-1)*diag([0 1 0 1 0 1 0 1 0 1 0 1]); % disturbance matrix

        A  % System matrix
        B  % Input matrix
        K  % Control gain matrix

        tr % trajectories (x, xr, t, dt, ...)
    end

    properties (Access = protected)
        DATA_FOLDER_PATH = 'data/'
        PATH % data path
    end
    
    methods
        function obj = UAV()
            obj.tr = Trajectory();  
        end
        
        % system functions
        function y = f(obj, x)  
            y = [
                x(2)
                -obj.Kx*x(2)/obj.m
                x(4)
                -obj.Ky*x(4)/obj.m
                x(6)
                -obj.Kz*x(6)/obj.m + obj.G
                x(8)
                % ((obj.Jy - obj.Jz)*x(9)*x(11) - obj.Kph*x(8))/obj.Jx
                % ((obj.Jy - obj.Jz)*x(10)*x(12) - obj.Kph*x(8))/obj.Jx
                -obj.Kph*x(8)/obj.Jx
                x(10)
                % ((obj.Jz - obj.Jx)*x(7)*x(11) - obj.Kth*x(10))/obj.Jy
                % ((obj.Jz - obj.Jx)*x(8)*x(12) - obj.Kth*x(10))/obj.Jy
                -obj.Kth*x(10)/obj.Jy
                x(12)
                % ((obj.Jx - obj.Jy)*x(7)*x(9) - obj.Kps*x(12))/obj.Jz
                % ((obj.Jx - obj.Jy)*x(8)*x(10) - obj.Kps*x(12))/obj.Jz
                -obj.Kps*x(12)/obj.Jz
            ];
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

        Save(obj, filename, whichVar) % save property
        obj = load(obj, filename)
    end

    methods (Access = private)
    end
end
%#ok<*PROPLC>
%#ok<*NASGU>
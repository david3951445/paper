classdef UAV_RM_L  
% UAV model, Reference Model, Linearized (e.g., fuzzy, global linearized, tensor product, ...)
% - System form
%       - dx/dt = f(x) + g(x)*u + Ev
% - The model is refered to 1.pdf
%       - Euler angle (intrinsic rotation)
%           - x(7)  : phi, x-axis, roll
%           - x(9)  : theta, y-axis, pitch
%           - x(11) : psi, z-axis, yaw
%       - Typo in 1.pdf.
%           - Coriolis term in dx(7)dt
%               ((uav.Jy - uav.Jz)*x(9)*x(11) - uav.Kph*x(8))/uav.Jx
%             need to be corrected to
%               ((uav.Jy - uav.Jz)*x(10)*x(12) - uav.Kph*x(8))/uav.Jx
%             dx(9)/dt, dx(11)/dt as well.
%               - More detail
%                   Quadrotor control: modeling, nonlinear control design, and simulation (2.19)
%                   [https://www.diva-portal.org/smash/get/diva2:860649/FULLTEXT01.pdf]
% - Reference model
%       - dxr/dt = Ar*x + Br*r

    properties (Constant)  
        DIM_X = 12  % dimension of state
        DIM_U = 4 % dimension of control
    end   
    
    properties
        % Designed parameters
        rho = 10^(1);
        Q   = 10^(-1)*diag([1, 0.001, 1.5, 0.002, 1, 0.001, 0.1, 0, 0.1, 0, 1, 0.001]); % correspond to x - xr
        E   = 10^(-1)*diag([0 1 0 1 0 1 0 1 0 1 0 1]); % disturbance matrix

        % Linearized system
        A  % System matrix
        B  % Input matrix
        C  % output matrix
        K  % Control gain matrix
        L  % Observer gain matrix

        % Reference model
        Ar
        Br
        TIMES_AR = 10;

        % trajectories (x, xr, t, dt, ...)
        tr 

        tau
        
        debug
    end

    properties (Access = protected)
        % system parameters
        m = 2, G = 9.81 %, l = 0.2, b = 2, d = 5, 
        Jx = 1.25, Jy = 1.25, Jz = 2.2
        Kx = 0.01, Ky = 0.01, Kz = 0.01
        Kph = 0.012, Kth = 0.012, Kps = 0.012

        % for saving data
        DATA_FOLDER_PATH = 'data/'
        PATH % data path
    end
    
    methods
        function uav = UAV_RM_L()
            uav.tr = Trajectory();
            uav.Ar = -uav.TIMES_AR*eye(uav.DIM_X);
            uav.Br = -uav.Ar;                         
        end
        
        % system functions
        function y = f(uav, x)  
            y = [
                x(2)
                -uav.Kx*x(2)/uav.m
                x(4)
                -uav.Ky*x(4)/uav.m
                x(6)
                -uav.Kz*x(6)/uav.m - uav.G
                x(8)
                % ((uav.Jy - uav.Jz)*x(9)*x(11) - uav.Kph*x(8))/uav.Jx
                ((uav.Jy - uav.Jz)*x(10)*x(12) - uav.Kph*x(8))/uav.Jx
                % -uav.Kph*x(8)/uav.Jx
                x(10)
                % ((uav.Jz - uav.Jx)*x(7)*x(11) - uav.Kth*x(10))/uav.Jy
                ((uav.Jz - uav.Jx)*x(8)*x(12) - uav.Kth*x(10))/uav.Jy
                % -uav.Kth*x(10)/uav.Jy
                x(12)
                % ((uav.Jx - uav.Jy)*x(7)*x(9) - uav.Kps*x(12))/uav.Jz
                ((uav.Jx - uav.Jy)*x(8)*x(10) - uav.Kps*x(12))/uav.Jz
                % -uav.Kps*x(12)/uav.Jz
            ];
        end
        
        function y = g(uav, x)
%             y = zeros(uav.dim, uav.dim_u);
            y(2, 1)  = (cos(x(7))*sin(x(9))*cos(x(11)) + sin(x(7))*sin(x(11)))/uav.m;
            y(4, 1)  = (cos(x(7))*sin(x(9))*sin(x(11)) - sin(x(7))*cos(x(11)))/uav.m;
            y(6, 1)  = cos(x(7))*cos(x(9))/uav.m;
            y(8, 2)  = 1/uav.Jx;
            y(10, 3) = 1/uav.Jy;
            y(12, 4) = 1/uav.Jz;
        end

        [r, F] = r_F(uav, x, t)
        Save(uav, filename, whichVar) % save property
        uav = load(uav, filename)
    end

    methods (Access = private)
    end
end
%#ok<*PROPLC>
%#ok<*NASGU>
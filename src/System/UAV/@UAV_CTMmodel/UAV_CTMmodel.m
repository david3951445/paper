classdef UAV_CTMmodel < UAV
%UAV CTM-based model
% - System form
%       - tau = M(x)*x'' + C(x, x') + G(x) + F(x')
% - Control law
%       - tau = M0(x)(Kp*e + Ki*intergral{e} + Kd*e') + C0x, x') + G0(x)
%
    properties (Constant)
        DIM_F = 6;
        % system parameters
    end   
    
    properties
        DIM_X3
        Dt % torque drag coefficient matrix
        Df % force drag coefficient matrix
        WINDOW % looking forward window of unknown signal
    end
    
    properties (Access = private)

    end

    % system functions
    methods
        function uav = UAV_CTMmodel(fz)
            uav@UAV();
            uav.PATH = [uav.DATA_FOLDER_PATH mfilename]; % Path of data
            uav = uav.load(); % load old data

            uav.Dt = diag([uav.Kph, uav.Kth, uav.Kps]);
            uav.Df = diag([uav.Kx, uav.Ky, uav.Kz]);
        end

        function y = M(uav, x)
            % y = [
            %     uav.R_inv(x(4:6))*uav.m zeros(3)
            %     zeros(3) diag([uav.Jx, uav.Jy, uav.Jz])
            % ];
            y = [
                uav.m*eye(3) zeros(3)
                zeros(3) diag([uav.Jx, uav.Jy, uav.Jz])
            ];
        end

        function y = H(uav, x, dx)
            % CG = [uav.R_inv(x(4:6))*[0;0;uav.G]; cross(dx(4:6), diag([uav.Jx, uav.Jy, uav.Jz])*dx(4:6))];
            CG = [[0;0;uav.G]; cross(dx(4:6), diag([uav.Jx, uav.Jy, uav.Jz])*dx(4:6))];
            % F = [uav.R_inv(x(4:6))*diag([uav.Kx, uav.Ky, uav.Kz]) zeros(3);zeros(3) diag([uav.Kph, uav.Kth, uav.Kps])]*dx;
            F = [uav.Df zeros(3);zeros(3) uav.Dt]*dx;

            y = CG + F;
        end
        
        function y = R_inv(uav, x)
            y = rotx(-x(1))*roty(-x(2))*rotz(-x(3));
        end

        uav = trajectory(uav) % get trajectory (x, u, ...)      
    end

    methods (Access = private)

    end
end
%#ok<*PROPLC>
%#ok<*NASGU>
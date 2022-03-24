classdef UAV_CLMmodel < UAV
%UAV CTM-based model
% - System form
%       - tau = M(x)*x'' + C(x, x') + G(x) + F(x')
% - Control law
%       - tau = M0(x)(Kp*e + Ki*intergral{e} + Kd*e') + C0x, x') + G0(x)
%
    properties (Constant)
        % system parameters
    end   
    
    properties

    end
    
    properties (Access = private)

    end

    % system functions
    methods
        function uav = UAV_FZmodel(fz)
            uav@UAV();
            uav.PATH = [uav.DATA_FOLDER_PATH mfilename]; % Path of data
            uav = uav.load(); % load old data
        end

        function y = M(x)
            y = [
                uav.R_inv(x(4:6))*uav.m zeros(3)
                zeros(x) diag([uav.Jx, uav.Jy, uav.Jz])
            ];
            % 
        end
        function y = H(x, dx)
            CG = [uav.R_inv(x(4:6))*uav.G; cross(dx(4:6), diag([uav.Jx, uav.Jy, uav.Jz])*dx(4:6))];
            F = [diag([uav.Kx, uav.Ky, uav.Kz]); diag([uav.Kph, uav.Kth, uav.Kps])]*dx
            y = CG + F
        end
        
        function y = R_inv(x)
            rotx(-x(1))*roty(-x(2))*rotz(-x(3));
        end
        % uav = trajectory(uav) % get trajectory (x, xr, u, ...)      
    end

    methods (Access = private)

    end
end
%#ok<*PROPLC>
%#ok<*NASGU>
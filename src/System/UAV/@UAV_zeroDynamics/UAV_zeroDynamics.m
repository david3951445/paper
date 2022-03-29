classdef UAV_zeroDynamics < UAV
%UAV zero dynamics control
% reference book(Section 3.2.2):
%       Quadrotor control: modeling, nonlinear control design, and simulation
%       source: https://www.diva-portal.org/smash/get/diva2:860649/FULLTEXT01.pdf

    properties (Constant)
        DIM_F = 4;
        % system parameters
    end   
    
    properties
    end
    
    properties (Access = private)

    end

    % system functions
    methods
        function uav = UAV_zeroDynamics(fz)
            uav@UAV();
            uav.PATH = [uav.DATA_FOLDER_PATH mfilename]; % Path of data
            uav = uav.load(); % load old data
        end

        function y = M(uav, x)
            y = [
                uav.m/(cos(x(2))*cos(x(3))) zeros(1)
                zeros(3) diag([uav.Jx, uav.Jy, uav.Jz])
            ];
        end

        function y = H(uav, x, dx)
            y = [
                uav.m/(cos(x(2))*cos(x(3)))*(uav.Kz/uav.m*dx(1) + uav.G)
                cross(dx(2:4), diag([uav.Jx, uav.Jy, uav.Jz])*dx(2:4)) + diag([uav.Kph, uav.Kth, uav.Kps])*dx
            ];
        end

        % uav = trajectory(uav) % get trajectory (x, u, ...)      
    end

    methods (Access = private)

    end
end
%#ok<*PROPLC>
%#ok<*NASGU>
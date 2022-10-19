classdef UAV_FZmodel < UAV_RM_L
%UAV fuzzy model
% - System form
%       - dx/dt = Ai*x + Bi*u + Ev
% - Control law
%       - u = Ki*x;
%
    properties (Constant)
        % system parameters
    end   
    
    properties
        % fz may be moved to here
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
        
        uav = getAB(uav, fz)
        uav = getKL(uav, fz)
        uav = trajectory(uav, fz) % get trajectory (x, xr, u, ...)
        
        % function Save(uav, whichVar) % pass "mfilename" to save@UAV()
        %     Save@UAV(uav, whichVar);
        % end
    end

    methods (Access = private)

    end
end
%#ok<*PROPLC>
%#ok<*NASGU>
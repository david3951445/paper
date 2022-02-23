classdef UAV_FZmodel < UAV
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
        % fz
    end
    
    properties (Access = private)

    end

    % system functions
    methods
        function uav = UAV_FZmodel(fz)
            uav@UAV();
            uav = uav.load(mfilename); % load old data
        end
        
        uav = getAB(uav, fz)
        uav = getKL(uav, fz, ref)
        uav = trajectory(uav, ref, fz) % get trajectory (x, xr, u, ...)
        
        function save(uav, whichVar) % pass "mfilename" to save@UAV()
            save@UAV(uav, mfilename, whichVar);
        end
    end

    methods (Access = private)

    end
end
%#ok<*PROPLC>
%#ok<*NASGU>
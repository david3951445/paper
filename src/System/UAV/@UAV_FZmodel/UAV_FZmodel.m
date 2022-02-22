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
        A
        B
        K
    end
    
    properties (Access = private)
        DATA_PATH = ['data/' mfilename];
    end

    % system functions
    methods
        function uav = UAV_FZmodel(fz)
            uav@UAV();
            % load old data
            file = load(uav.DATA_PATH);
            uav.A = file.A;
            uav.B = file.B;
            uav.K = file.K;
            uav.tr = file.tr;
        end
        
        obj = getAB(obj, fz)
        obj = getKL(obj, fz, ref)
        obj = trajectory(obj, ref, fz) % get trajectory (x, xr, u, ...)
        save(obj, whichVar) % save property
    end

    methods (Access = private)

    end
end
%#ok<*PROPLC>
%#ok<*NASGU>
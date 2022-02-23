classdef UAV_TPmodel < UAV
%UAV TP model
% - System form
%       - dx/dt = Ai*x + Bi*u + Ev
% - Control law
%       - u = Ki*x;
%
    properties (Constant)
        % system parameters
    end   
    
    properties
        ABl
        AB
    end
    
    % system functions
    methods
        function uav = UAV_TPmodel()
            uav@UAV();
            uav = uav.load(mfilename); % load old data b

            uav = uav.getABl();
            uav.ABl.domain = 0.5*[-1 1; -1 1; -1 1];
            uav.ABl.gridsize = 20*[1 1 1];
            uav.ABl.SV_TOLERANCE = 0.001;
            uav.ABl.num_p = length(uav.ABl.gridsize); % length of parameter vector of lpv system
            uav.ABl.dep = zeros([size(uav.ABl.val) uav.ABl.num_p]);
            % A
            % B
            uav.ABl.dep(2, 13, :) = [1 1 1];
            uav.ABl.dep(4, 13, :) = [1 1 1];
            uav.ABl.dep(6, 13, :) = [1 1 0];
        end
        
        function save(obj, whichVar) % pass "mfilename" to save@UAV()
            save@UAV(obj, mfilename, whichVar);
        end
    end

    methods (Access = private)

    end
end
%#ok<*PROPLC>
%#ok<*NASGU>
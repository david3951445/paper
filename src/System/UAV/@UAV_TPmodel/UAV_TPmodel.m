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
classdef UAV_TPmodel < UAV_RM_L
%UAV TP model
% - System form
%       - dx/dt = Ai*x + Bi*u + Ev
% - Control law
%       - u = Ki*x;
    properties (Constant)
        
    end   
    
    properties
        AB % [A B] matrix
        ABl % AB's TPmodel transformation parameters
    end

    properties (Access = private)
    end
    
    % system functions
    methods
        function uav = UAV_TPmodel()
            uav@UAV();
            uav.PATH = [uav.DATA_FOLDER_PATH mfilename]; % Path of data
            uav = uav.load(); % load old data
        end
        
        uav = getAB(uav) % TPmodel parameter setting and execute
        uav = getKL(uav) % LMI
        uav = trajectory(uav) % calculate trajectory
        % Save(uav, whichVar)
        % uav = load(uav)
    end

    methods (Access = private)

    end
end
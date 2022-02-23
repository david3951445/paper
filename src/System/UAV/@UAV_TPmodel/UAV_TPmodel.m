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
        AB
        ABl
        test = 1
    end
    
    % system functions
    methods
        function uav = UAV_TPmodel()
            uav@UAV();
            uav = uav.load(mfilename); % load old data b

        end
        
        function Save(uav, whichVar)
            PATH = [uav.DATA_FOLDER_PATH mfilename];

            switch whichVar
                case 'AB'
                    AB = uav.AB;
                otherwise
                    Save@UAV(uav, mfilename, whichVar); % The properties in superclass UAV()
            end

            if isfile([PATH '.mat'])
                save(PATH, whichVar, '-append')
            else
                save(PATH, whichVar)
            end           
        end
    end

    methods (Access = private)

    end
end
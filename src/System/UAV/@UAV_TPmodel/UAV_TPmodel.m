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
        
        uav = getAB(uav)
        uav = getKL(uav, ref)
        uav = trajectory(uav, ref)
        
        function Save(uav, whichVar)
            switch whichVar
                case 'AB'
                    AB = uav.AB;
                otherwise
                    Save@UAV(uav, mfilename, whichVar); % The properties in superclass UAV()
                    return
            end
            
            if isfile([uav.PATH '.mat'])
                save(uav.PATH, whichVar, '-append')
            else
                save(uav.PATH, whichVar)
            end           
        end

        function uav = load(uav)
            if isfile([uav.PATH '.mat'])
                data = load(uav.PATH);
                
                if isfield(data, 'AB')
                    uav.AB = data.AB;
                end
            end

            uav = load@UAV(uav);
        end
    end

    methods (Access = private)

    end
end
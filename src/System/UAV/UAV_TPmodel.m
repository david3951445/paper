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
        
        A
        B
        K
    end
    
    % system functions
    methods
        function obj = UAV_TPmodel(fz)
            obj@UAV();
            % load old data
            UAV_MAT = load('data/uav.mat');
            obj.A = UAV_MAT.A;
            obj.B = UAV_MAT.B;
            obj.K = UAV_MAT.K;

            %% find A, B (linearize)
            if EXE.A_B
                obj.getAB(fz);
                obj.save('data/uav.mat', 'A')
                obj.save('data/uav.mat', 'B')  
            end 
        end
        
        function obj = test(obj, a)
            disp(a)
        end
        % Save property
        function save(obj, fname, whichVar)
            switch whichVar
                case 'A'
                    A = obj.A;
                case 'B'
                    B = obj.B;
                case 'K'
                    K = obj.K;
                otherwise
                    disp('No such property in UAV')
            end

            if isfile(fname)
                save(fname, whichVar, '-append');
            else
                save(fname, whichVar);
            end
            
        end
    end

    methods (Access = private)

    end
end
%#ok<*PROPLC>
%#ok<*NASGU>
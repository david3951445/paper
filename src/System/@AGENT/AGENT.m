classdef AGENT  
% agent model

    properties (Constant)  
        DIM_X = 12  % dimension of state
    end   
    
    properties
        % trajectories (x, xr, t, dt, ...)
        tr 

        tau
        
    end

    properties (Access = protected)
        % system parameters
        m = 2, l = 0.2, b = 2, d = 5, G = 9.81
        Jx = 1.25, Jy = 1.25, Jz = 2.2
        Kx = 0.01, Ky = 0.01, Kz = 0.01
        Kph = 0.012, Kth = 0.012, Kps = 0.012

        % for saving data
        DATA_FOLDER_PATH = 'data/'
        PATH % data path
    end
    
    methods
        function uav = AGENT()
            uav.tr = Trajectory();
            uav.Ar = -uav.TIMES_AR*eye(uav.DIM_X);
            uav.Br = -uav.Ar;                         
        end

        Save(uav, filename, whichVar) % save property
        uav = load(uav, filename)
    end

    methods (Access = private)
    end
end
%#ok<*PROPLC>
%#ok<*NASGU>
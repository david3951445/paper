classdef Agent  
% agent model

    properties (Constant)  

    end   

    properties (Access = protected)
        PATH % path of saved data
    end

    properties
        DIM_F % dimension of state (pos)
        % INTERP_DENSITY = 2500 % interp density of zmp

        %% Control design
        sys
        sys_a
        sys_s
        sys_aug

        K  % Control gain matrix
        KL  % Observer gain matrix
        % DIM

        % trajectories (x, xr, t, dt, ...)
        tr

        %% flow control of code
        EXE_LMI = 1 % solving LMI
        EXE_TRAJ = 1 % trajectory
        EXE_PLOT = 1 % plot results
    end
    
    methods
        function ag = AGENT()
        end
        
        y = M(ag, x) % inertial matrix
        y = H(ag, x, dx) % non-inertial matrix
        ag = get_K_L(ag, sys_aug1) % solve LMI to obtain control gain K and observer gain L
        ag = trajectory(ag) % get trajectory (x, u, ...)
        is_saved = Save(ag, whichVar) % save properties
        ag = Load(ag) % load properties
    end
end
%#ok<*PROPLC>
%#ok<*NASGU>
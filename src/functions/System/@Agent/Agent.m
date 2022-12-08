classdef Agent  
% agent model

    properties (Constant)  

    end   

    properties (Access = protected)
    end

    properties
        DIM_F % dimension of state (pos)
        % INTERP_DENSITY = 2500 % interp density of zmp

        %% Control design
        sys % agent system
        sys_a % actuator smoothed model
        sys_s % sensor smoothed model
        sys_aug % augment system

        K  % Control gain matrix
        KL  % Observer gain matrix
        % DIM

        % trajectories (x, xr, t, dt, ...)
        tr

        %% flow control of code
        EXE_LMI = 1 % solving LMI
        EXE_TRAJ = 1 % trajectory
        EXE_PLOT = 1 % plot results

        FILE_NAME % file name of class
        PATH % path of saved data
    end
    
    methods
        function ag = AGENT()
        end

        function y = u_PID(ag, xh)
            % range = 1 : rb.DIM_F*3; 
            % y = rb.K(:, range)*xh(range); % without unknown sigal info
            y = ag.K*xh; 
        end

        function y = f_aug(rb, t, xb) % A_bar*x_bar
            DIM_X3 = rb.sys_aug.DIM_X;
            x = xb(1 : DIM_X3);
            xh = xb(DIM_X3 + (1:DIM_X3));
            u = rb.u_PID(xh);
            y = [
                rb.sys_aug.A*x + rb.sys_aug.B*u
                rb.sys_aug.A*xh + rb.sys_aug.B*u - rb.KL*rb.sys_aug.C*(x-xh)
            ];
        end

        y = M(ag, x) % inertial matrix
        y = H(ag, x, dx) % non-inertial matrix
        ag = get_K_L(ag, sys1, sys_aug1) % solve LMI to obtain control gain K and observer gain L
        ag = trajectory(ag) % get trajectory (x, u, ...)
        is_saved = Save(ag, whichVar) % save properties
        ag = Load(ag) % load properties
        PlotTC(ag) % Plot the results of tracking control
        [ag, xb] = CalculateNextState(ag, xb, d1, r, dr, ddr, i) % Calculate xb(i+1). Temporary method, the coupling between subclass (UAV_AGENTmodel and Robot) must be reduced
        PlotFault(ag, TITLE) % Plot fault estimation
        y = GetHinfPerformance(ag)

    end
end
%#ok<*PROPLC>
%#ok<*NASGU>
classdef Trajectory 
%trajectory for a system

    properties (Constant)

    end   
    
    properties
        dt  = 0.001 % time step
        t % time
        T   = 10 % final time
        LEN
        
        % initial value
        x0
        xr0
        xh0

        IS_LINEAR = 1 % run fuzzy linear system or origin nonlinear system
        IS_RK4 = 0 % run RK4 or Euler

        % trajectory
        x % system state
        r % reference trajectory
        xr % reference model state
        xh % estimated state

        e % x - r
        er % x - xr
        eh % x - xh

        u % control
        v % distrubance
    end
    
    methods
        function o = Trajectory()
            o.t = 0 : o.dt : o.T;
        end
        
        function plot(o)
            % figure('units','normalized','outerposition',[0 0 1 1])
            % x(t), xr(t)
            SIZE_X = size(o.x, 1);
            for i = 1 : SIZE_X
                figure
                plot(o.t, o.x(i, :), o.t, o.xr(i, :))
                title(['x_{' num2str(i) '}'])
                legend("x", "x_r")
                xlabel("t")
                % ylim([-2 2])
            end 
            
            % u(t)
            % for j = 1 : o.SIZE_U
            %     figure(i+j)
            %     plot(o.t, o.u(j, :));
            %     title(['u_{' num2str(j) '}'])
            % end
        end
    end

    methods (Access = private)
        
    end
end


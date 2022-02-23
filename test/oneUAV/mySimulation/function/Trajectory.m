classdef Trajectory 
%trajectory for a system

    properties (Constant)

    end   
    
    properties
        dt  = 0.001 % time step
        t % time
        T   = 10 % final time

        % initial value
        x0  = [0.1 0 0.1 0.5 0.1 0.5 0.51 0.59 0.52 0.52 0.55 0.52]';
        xr0 = [0 1 0.5 0 0 0.8 0 0 0 0 0 0]';

        IS_LINEAR = 1 % run fuzzy linear system or origin nonlinear system
        IS_RK4 = 0 % run RK4 or Euler

        % trajectory
        x
        xr
        u
        v
        r
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
                figure(i)
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


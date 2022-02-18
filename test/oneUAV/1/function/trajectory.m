classdef Trajectory 
%Generate trajectory for a multi-local linear system with reference model

    properties (Constant)
        dt  = 0.002 % time step
        T   = 10 % final time

        % initial value
        x0  = [0.54 0.55 0.57 0.57 2 0.5 0.51 0.59 0.52 0.52 0.55 0.52]';
        xr0 = [0 1 0.5 0 0 0.8 0 0 0 0 0 0]';
    end   
    
    properties
        t % time
        LEN
        SIZE_X
        SIZE_U

        x
        xr
        v
        r
    end
    
    methods
        function o = Trajectory(uav, ref, fz)
            dt = o.dt;

            [o.SIZE_X, o.SIZE_U] = size(uav.B{1});

            o.t         = 0 : dt : o.T;
            o.LEN       = length(o.t);
        
            v           = @(t) 0.5*randn(o.SIZE_X, 1) + 0;
            vb          = @(x, t) [v(t); ref.r(x, [], t)];

            xb = zeros(2*o.SIZE_X, o.LEN);
            xb(:, 1) = [o.x0; o.xr0];

            for i = 1 : o.LEN - 1       
                if mod(i, 1/o.dt) == 0
                    disp(['t = ' num2str(i*dt)])
                end
            
                k1 = o.RK4(uav, fz, ref, xb(:, i),         vb, i*dt);
                k2 = o.RK4(uav, fz, ref, xb(:, i)+k1*dt/2, vb, i*dt + dt/2);
                k3 = o.RK4(uav, fz, ref, xb(:, i)+k2*dt/2, vb, i*dt + dt/2);
                k4 = o.RK4(uav, fz, ref, xb(:, i)+k3*dt,   vb, i*dt + dt); 

                xb(:, i+1)  = xb(:, i)  + (k1 + 2*k2 + 2*k3 + k4)*dt/6;
                % xb(:, i+1)  = xb(:, i) +  k1*dt;
            end

            o.x     = xb(1 : o.SIZE_X, :);
            o.xr    = xb(o.SIZE_X+1 : 2*o.SIZE_X, :);
        end
        
        function plot(o)
            % figure('units','normalized','outerposition',[0 0 1 1])
            for i = 1 : o.SIZE_X
                figure(i)
            %     subplot(1, 2, i);
                plot(o.t, o.x(i, :), o.t, o.xr(i, :));
                title(['x_{' num2str(i) '}']);
                legend("x", "x_r");
                xlabel("t"); ylim([-15 15])
            end               
        end

    end

    methods (Access = private)
        function k = RK4(o, uav, fz, ref, x, vb, t)
            O = zeros((o.SIZE_X));
            % A = O;
            % B = zeros(o.SIZE_X, o.SIZE_U);
            % K = B';
            Ab = zeros(o.SIZE_X*2);
            for i = 1 : fz.num
                A = uav.A{i};
                B = uav.B{i};
                K = uav.K{i};
                Ab = Ab + fz.mbfun(i, x)*[A+B*K -B*K; O ref.A];
            end
            Eb = [uav.E uav.O; uav.O ref.B];
        
            k = Ab*x + Eb*vb(x, t);
        end
    end
end


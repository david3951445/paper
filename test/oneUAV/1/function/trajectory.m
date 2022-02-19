classdef Trajectory 
%Generate trajectory for a system with reference model

    properties (Constant)
        dt  = 0.002 % time step
        T   = 20 % final time

        % initial value
        x0  = [0.1 0 0.1 0.5 0.1 0.5 0.51 0.59 0.52 0.52 0.55 0.52]';
        xr0 = [0 1 0.5 0 0 0.8 0 0 0 0 0 0]';

        IS_LINEAR = 1 % run fuzzy linear system or origin nonlinear system
        IS_RK4 = 0 % run RK4 or Euler
    end   
    
    properties
        t % time
        LEN % length of t
        SIZE_X
        SIZE_U

        x
        xr
        u
        v
        r
    end
    
    methods
        function o = Trajectory(uav, ref, fz)
            dt = o.dt;

            [o.SIZE_X, o.SIZE_U] = size(uav.B{1});

            o.t         = 0 : dt : o.T;
            o.LEN       = length(o.t);
        
            xb = zeros(2*o.SIZE_X, o.LEN);
            u = zeros(o.SIZE_U, o.LEN);
            xb(:, 1) = [o.x0; o.xr0];

            disp('Ploting trajectory ...')
            for i = 1 : o.LEN - 1       
                if mod(i, 1/o.dt) == 0
                    disp(['t = ' num2str(i*dt)])
                end
                
                k1 = o.RK4(uav, fz, ref, xb(:, i), i*dt);
                if o.IS_RK4
                    k2 = o.RK4(uav, fz, ref, xb(:, i)+k1*dt/2, i*dt + dt/2);
                    k3 = o.RK4(uav, fz, ref, xb(:, i)+k2*dt/2, i*dt + dt/2);
                    k4 = o.RK4(uav, fz, ref, xb(:, i)+k3*dt, i*dt + dt); 

                    xb(:, i+1)  = xb(:, i)  + (k1 + 2*k2 + 2*k3 + k4)*dt/6;
                end
                xb(:, i+1)  = xb(:, i) +  k1*dt;

                o.x(:, i+1) = xb(1 : o.SIZE_X, i+1);
                o.xr(:, i+1) = xb(o.SIZE_X+1 : 2*o.SIZE_X, i+1);

                total_K = zeros(o.SIZE_U, o.SIZE_X);
                for j = 1 : fz.num
                    total_K = total_K + fz.mbfun(j, o.x(:, i+1))*uav.K{j};
                end
                o.u(:, i+1) = total_K*(o.x(:, i+1) - o.xr(:, i+1));
            end
        end
        
        function plot(o)
            % figure('units','normalized','outerposition',[0 0 1 1])
            for i = 1 : o.SIZE_X
                figure(i)
                plot(o.t, o.x(i, :), o.t, o.xr(i, :))
                title(['x_{' num2str(i) '}'])
                legend("x", "x_r")
                xlabel("t")
                % ylim([-2 2])
            end 
            
            for j = 1 : o.SIZE_U
                figure(i+j)
                plot(o.t, o.u(j, :));
                title(['u_{' num2str(j) '}'])
            end
        end

    end

    methods (Access = private)
        function k = RK4(o, uav, fz, ref, xb, t)
            O       = zeros((o.SIZE_X));
            v       = @(t) 0.1*randn(o.SIZE_X, 1) + 0;
            [r, F]  = ref.r_F(xb(1 : o.SIZE_X), t);
            x       = xb(1 : o.SIZE_X);
            xr      = xb(o.SIZE_X+1 : o.SIZE_X*2);

            if o.IS_LINEAR % linear
                Ab = zeros(o.SIZE_X*2);
                total_B = zeros(o.SIZE_X, o.SIZE_U);
                for i = 1 : fz.num
                    A = uav.A{i};
                    B = uav.B{i};
                    K = uav.K{i};

                    total_B = total_B + fz.mbfun(i, x)*B;
                    Ab = Ab + fz.mbfun(i, x)*[A+B*K -B*K; O ref.A];
                end
                Eb = [uav.E uav.O; uav.O ref.B];

                feed = [total_B*[F; 0; 0; 0]; zeros(12, 1)]; % feedforward
                k = Ab*xb + Eb*[v(t); r];
            else % nonlinear
                % control gain
                K = zeros(size(uav.K{1}));
                for i = 1 : fz.num
                    K = K + fz.mbfun(i, x)*uav.K{i};
                end
                % fine-tune
                % K = K;
                
                U = K*(x - xr);

                k = [
                    uav.f(x) + uav.g(x)*U + uav.E*v(t)
                    ref.A*xr + ref.B*r
                ];
            end
        end
    end
end


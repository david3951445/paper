classdef Trajectory 
%Generate trajectory for a system with reference model

    properties (Constant)
        dt  = 0.002 % time step
        T   = 20 % final time

        % initial value
        x0  = [0.54 0.55 0.57 0.57 2 0.5 0.51 0.59 0.52 0.52 0.55 0.52]';
        xr0 = [0 1 0.5 0 0 0.8 0 0 0 0 0 0]';

        LINEAR = 1 % determine run fuzzy linear system or origin nonlinear system
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
        
            v           = @(t) 0.1*randn(o.SIZE_X, 1) + 0;
            vb          = @(x, t) [v(t); ref.r(x, [], t)];

            xb = zeros(2*o.SIZE_X, o.LEN);
            u = zeros(o.SIZE_U, o.LEN);
            xb(:, 1) = [o.x0; o.xr0];

            disp('Ploting trajectory ...')
            for i = 1 : o.LEN - 1       
                if mod(i, 1/o.dt) == 0
                    disp(['t = ' num2str(i*dt)])
                end
            
                k1 = o.RK4(uav, fz, ref, xb(:, i), vb, i*dt);
                % k2 = o.RK4(uav, fz, ref, xb(:, i)+k1*dt/2, vb, i*dt + dt/2);
                % k3 = o.RK4(uav, fz, ref, xb(:, i)+k2*dt/2, vb, i*dt + dt/2);
                % k4 = o.RK4(uav, fz, ref, xb(:, i)+k3*dt, vb, i*dt + dt); 

                % xb(:, i+1)  = xb(:, i)  + (k1 + 2*k2 + 2*k3 + k4)*dt/6;
                xb(:, i+1)  = xb(:, i) +  k1*dt;

                o.x(:, i+1)     = xb(1 : o.SIZE_X, i+1);
                o.xr(:, i+1)    = xb(o.SIZE_X+1 : 2*o.SIZE_X, i+1);
                % o.u(:, i+1)     = o.xr(:, i+1) - o.x(:, i+1)
            end
        end
        
        function plot(o)
            % figure('units','normalized','outerposition',[0 0 1 1])
            for i = 1 : o.SIZE_X
                figure(i)
            %     subplot(1, 2, i);
                plot(o.t, o.x(i, :), o.t, o.xr(i, :))
                title(['x_{' num2str(i) '}'])
                legend("x", "x_r")
                xlabel("t")
                ylim([-2 2])
            end               
        end

    end

    methods (Access = private)
        function k = RK4(o, uav, fz, ref, xb, vb, t)
            O = zeros((o.SIZE_X));
            if o.LINEAR % linear
                Ab = zeros(o.SIZE_X*2);
                for i = 1 : fz.num
                    A = uav.A{i};
                    B = uav.B{i};
                    K = uav.K{i};
                    Ab = Ab + fz.mbfun(i, xb)*[A+B*K -B*K; O ref.A];
                end
                Eb = [uav.E uav.O; uav.O ref.B];
            
                k = Ab*xb + Eb*vb(xb, t);
            else % nonlinear
                x = xb(1 : o.SIZE_X);
                xr = xb(o.SIZE_X+1 : o.SIZE_X*2);
                vb_ = vb(xb, t);
                v = vb_(1 : o.SIZE_X);
                r = vb_(o.SIZE_X+1 : o.SIZE_X*2);

                % control gain
                K = zeros(size(uav.K{1}));
                for i = 1 : fz.num
                    K = K + fz.mbfun(i, x)*uav.K{i};
                end
                % fine-tune
                % K = K;
                
                U = K*(x - xr);
                k = [
                    uav.f(x) + uav.g(x)*U + v
                    ref.A*xr + ref.B*ref.r(zeros(12,1), U(1), t)
                ];
            end
        end
    end
end


function tr = trajectory(uav, fz, ref, p)
    dt = p.dt; dt2 = p.dt/2;
    t = 0 : dt : p.tf; t2 = 0 : dt2 : p.tf;

    v = zeros(uav.dim, length(t2));
    v(2:2:12, :) = 0.1*wgn(uav.dim/2, length(t2), 0);
    x = zeros(uav.dim, length(t)); x(1:6, 1) = [0; 1; 0.5; 0; 0; 0.8];
    xr = zeros(uav.dim, length(t)); xr(1:6, 1) = [0; 1; 0.5; 0; 0; 0.8];

    for i = 1 : length(t) - 1
        [k1, kr1] = RK4(uav, fz, ref, x(:, i),           xr(:, i),            v(2*i-1), t2(2*i-1));
        [k2, kr2] = RK4(uav, fz, ref, x(:, i)+0.5*k1*dt, xr(:, i)+0.5*kr1.*dt, v(2*i),   t2(2*i));
        [k3, kr3] = RK4(uav, fz, ref, x(:, i)+0.5*k2*dt, xr(:, i)+0.5*kr2.*dt, v(2*i),   t2(2*i));
        [k4, kr4] = RK4(uav, fz, ref, x(:, i)+k3*dt,     xr(:, i)+kr3.*dt,     v(2*i+1), t2(2*i+1));

        x(:, i+1)  = x(:, i)  + (k1 + 2*k2 + 2*k3 + k4)*dt/6; 
        xr(:, i+1) = xr(:, i) + (kr1 + 2*kr2 + 2*kr3 + kr4)*dt/6;
    end
    
    tr.x = x; tr.xr = xr; tr.t = t;
    
    %% function
    function [k, kr] = RK4(uav, fz, ref, x, xr, v, t)
        k = zeros(uav.dim, 1); kr = zeros(uav.dim, 1);
        for j = 1 : fz.num
            K = uav.K(:, :, j);
            U = K*(x - xr);
        %     U = K*[x; xr]; % method 2
            k = k + fz.mbfun(j, x)*(uav.f(x) + uav.g(x)*U + v);
%             kr = kr + fz.mbfun(j, x)*(ref.A*xr + ref.B*ref.r(x, U(1), t));
        %     kr = kr + ref.r(x, U(1), t) - x; % method 2
        end
        kr = ref.A*xr + ref.B*ref.r(x, U(1), t);
    end
end

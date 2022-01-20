function tr = trajectory(uav, fz, ref, p, method)
    dt = p.dt; dt2 = p.dt/2;
    t = p.t; t2 = 0 : dt2 : p.tf;

    v = zeros(uav.dim, length(t2));
    v(2:2:12, :) = p.ampv*wgn(uav.dim/2, length(t2), 0);
    x = zeros(uav.dim, length(t));
    xr = zeros(uav.dim, length(t));
    x(:, 1)  = [0; 1; 0.5; 0; 0; 0; -0.0025; 0; 0; 0; 0; 0];
    xr(:, 1) = [0; 1; 0.5; 0; 0; 0; -0.0025; 0; 0; 0; 0; 0];
    
    switch method
        case 'Euler'
            for i = 1 : length(t) - 1
                [k, kr] = dxdt(uav, fz, ref, x(:, i), xr(:, i), v(2*i-1), t2(2*i-1), i);
                
                x(:, i+1)  = x(:, i) +  p.dt*k;
                xr(:, i+1) = xr(:, i) + p.dt*kr;
            end
            
        case 'RK4'
            for i = 1 : length(t) - 1
                [k1, kr1] = dxdt(uav, fz, ref, x(:, i),           xr(:, i),             v(2*i-1),  t2(2*i-1), i);
                [k2, kr2] = dxdt(uav, fz, ref, x(:, i)+0.5*k1*dt, xr(:, i)+0.5*kr1.*dt, v(2*i),   t2(2*i)  , i);
                [k3, kr3] = dxdt(uav, fz, ref, x(:, i)+0.5*k2*dt, xr(:, i)+0.5*kr2.*dt, v(2*i),   t2(2*i)  , i);
                [k4, kr4] = dxdt(uav, fz, ref, x(:, i)+k3*dt,     xr(:, i)+kr3.*dt,     v(2*i+1), t2(2*i+1), i);

                x(:, i+1)  = x(:, i)  + (k1 + 2*k2 + 2*k3 + k4)*dt/6; 
                xr(:, i+1) = xr(:, i) + (kr1 + 2*kr2 + 2*kr3 + kr4)*dt/6;
            end
    end
    
    tr.x = x; tr.xr = xr; tr.t = t;
    
    %% function
    function [k, kr] = dxdt(uav, fz, ref, x, xr, v, t, i)
        k = zeros(uav.dim, 1);
        for j1 = 1 : fz.num
            U1 = uav.uo(:, i) + uav.K(:, :, j1)*(x - xr);
            k = k + fz.mbfun(j1, x)*(uav.f(x) + uav.g(x)*U1 + v);
        end
        kr = ref.A*xr + ref.B*ref.r(t);
    end
end

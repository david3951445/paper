function tr = trajectory2(uav, fz, ref, p)
dt = p.dt; dt2 = p.dt/2;
t = 0 : dt : p.tf; t2 = 0 : dt2 : p.tf;

v = zeros(uav.dim, length(t2));
%     v(2:2:12, :) = 0.1*wgn(uav.dim/2, length(t2), 0);
x = zeros(uav.dim, length(t));
%     x(1:6, 1) = [0; 1; 0.5; 0; 0; 0.8];
x(:, 1) = [0.54 0.55 0.57 0.57 2 0.5 0.51 0.59 0.52 0.52 0.55 0.52]';
xr = zeros(uav.dim, length(t));
xr(1:6, 1) = [0; 1; 0.5; 0; 0; 0.8];

for i = 1 : length(t) - 1
%         if isnan(norm(x))
%             error('diverge')
%         end
    
    [k1, kr1] = RK4(uav, fz, ref, x(:, i),           xr(:, i),            v(2*i-1), t2(2*i-1));
    [k2, kr2] = RK4(uav, fz, ref, x(:, i)+0.5*k1*dt, xr(:, i)+0.5*kr1.*dt, v(2*i),   t2(2*i));
    [k3, kr3] = RK4(uav, fz, ref, x(:, i)+0.5*k2*dt, xr(:, i)+0.5*kr2.*dt, v(2*i),   t2(2*i));
    [k4, kr4] = RK4(uav, fz, ref, x(:, i)+k3*dt,     xr(:, i)+kr3.*dt,     v(2*i+1), t2(2*i+1));

    x(:, i+1)  = x(:, i)  + (k1 + 2*k2 + 2*k3 + k4)*dt/6; 
    xr(:, i+1) = xr(:, i) + (kr1 + 2*kr2 + 2*kr3 + kr4)*dt/6;
    r(:, i+1) = ref.r(x(:, i), 0, t(i));
end

tr.x = x;
tr.xr = xr;
tr.r = r;
tr.t = t;
end

%% Local function
function [k, kr] = RK4(uav, fz, ref, x, xr, v, t)
    % Calculate sum of Kj
    K_total = zeros(size(uav.K(:, :, 1)));
    for j = 1 : fz.num
        % U = K*[x; xr]; % method 2
        K_total = K_total + fz.mbfun(j, x)*uav.K(:, :, j);
        % kr = kr + ref.r(x, U(1), t) - x; % method 2
    end
    
    U = K_total*(x - xr);
    k = uav.f(x) + uav.g(x)*U + v;
    kr = ref.A*xr + ref.B*ref.r(x, U(1), t);
end

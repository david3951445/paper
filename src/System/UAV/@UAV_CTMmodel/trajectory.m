function uav = trajectory(uav, fz)
%calculate trajectory

dt          = uav.tr.dt;
t           = 0 : dt : uav.tr.T;
LEN         = length(t);

xb          = zeros(2*uav.DIM_X, LEN);
xb(:, 1)    = [uav.tr.x0; uav.tr.xr0];
u           = zeros(uav.DIM_U, LEN);

disp('Ploting trajectory ...')
for i = 1 : LEN - 1       
    if mod(i, 1/dt) == 0
        disp(['t = ' num2str(i*dt)])
    end
    
    k1 = RK4(uav, xb(:, i), i*dt);
    if uav.tr.IS_RK4
        k2 = RK4(uav, fz, xb(:, i)+k1*dt/2, i*dt + dt/2);
        k3 = RK4(uav, fz, xb(:, i)+k2*dt/2, i*dt + dt/2);
        k4 = RK4(uav, fz, xb(:, i)+k3*dt, i*dt + dt); 

        xb(:, i+1)  = xb(:, i)  + (k1 + 2*k2 + 2*k3 + k4)*dt/6;
    end
    xb(:, i+1)  = xb(:, i) +  k1*dt;

    uav.tr.x(:, i+1) = xb(1 : uav.DIM_X, i+1);
    uav.tr.xr(:, i+1) = xb(uav.DIM_X+1 : 2*uav.DIM_X, i+1);
    % uav.tr.r(:, i+1) = 

    % total_K = zeros(o.DIM_U, o.DIM_X);
    % for j = 1 : fz.num
    %     total_K = total_K + fz.mbfun(j, o.x(:, i+1))*uav.K{j};
    % end
    % o.u(:, i+1) = total_K*(o.x(:, i+1) - o.xr(:, i+1));
end
uav.tr.t = t;
end

%% Local function
function k = RK4(uav, xb, t)
    O       = zeros(uav.DIM_X);
    v       = @(t) 0.01*randn(uav.DIM_X, 1) + 0;
    [r, ~]  = uav.r_F(xb(1 : uav.DIM_X), t);
    x       = xb([1 : uav.DIM_X]);
    X       = x([1 3 5 7 9 11]);
    dX      = x([2 4 6 8 10 12]);

    if uav.tr.IS_LINEAR % linear                 
        Eb = [eye(uav.DIM_X) O; O uav.Br];
        u = K*x + uav.M(X) + uav.H(X, dX) % control law
        k = uav.A*xb + uav.B*u + Eb*[v(t); r];

    else % nonlinear        
        u = K*x + uav.M(X) + uav.H(X, dX) % control law

        k = [
            uav.f(x) + uav.g(x)*U + eye(uav.DIM_X)*v(t)
            uav.Ar*xr + uav.Br*r
        ];
    end
end
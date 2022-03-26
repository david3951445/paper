function uav = trajectory(uav, fz)
%calculate trajectory

dt          = uav.tr.dt;
t           = 0 : dt : uav.tr.T;
LEN         = length(t);

x           = zeros(uav.DIM_X3, LEN);
x(uav.DIM_F+(1:2*uav.DIM_F)) = uav.tr.x0;
xh          = zeros(uav.DIM_X3, LEN);
xb          = [x; xh];
u           = zeros(uav.DIM_U, LEN);

%% calculate r, dr, ddr
amp_z   = 1; % 0.8
amp     = 1; % 1
freg    = 0.5;
% referecne trajectory of x, y, z
r1 = zeros(3, LEN);
for i = 1 : LEN
    r1(:, i) = [
        amp*sin(freg*t(i))
        amp*cos(freg*t(i))
        amp_z*t(i)
    ];      
end
dr1 = my_diff(r1);
ddr1 = my_diff(dr1);

% referecne trajectory of phi, theta, psi
r2 = zeros(3, LEN);
for i = 1 : LEN - 1
    c1 = uav.m*ddr1(1, i) + uav.Kx*dr1(1, i);
    c2 = uav.m*ddr1(2, i) + uav.Ky*dr1(2, i);
    c3 = uav.m*(ddr1(3, i) + uav.G) + uav.Kz*dr1(3, i);
    theta = atan2(c1, c3);
    r2(:, i) = [
        atan2(-c2*cos(theta), c3)
        theta
        0
    ];      
end
dr2 = my_diff(r2);
ddr2 = my_diff(dr2);
uav.tr.r = {[r1; r2], [dr1; dr2], [ddr1; ddr2]};

%% disturbance
v = 0.01*randn(uav.DIM_X, LEN) + 0;
uav.tr.v = [zeros(uav.DIM_X, LEN); zeros(uav.DIM_X, LEN); v];

%% plot
disp('Ploting trajectory ...')
for i = 1 : LEN - 1       
    if mod(i, 1/dt) == 0
        disp(['t = ' num2str(i*dt)])
    end
    if norm(xb(:, i)) == NaN

    end

    k1 = RK4(uav, xb(:, i), i);
    if uav.tr.IS_RK4
        % k2 = RK4(uav, fz, xb(:, i)+k1*dt/2, i*dt + dt/2);
        % k3 = RK4(uav, fz, xb(:, i)+k2*dt/2, i*dt + dt/2);
        % k4 = RK4(uav, fz, xb(:, i)+k3*dt, i*dt + dt); 

        % xb(:, i+1)  = xb(:, i)  + (k1 + 2*k2 + 2*k3 + k4)*dt/6;
    end
    xb(:, i+1) = xb(:, i) + k1*dt;
    uav.tr.xb(:, i+1) = xb(:, i+1);
    % uav.tr.r(:, i+1) = 

    % total_K = zeros(o.DIM_U, o.DIM_X);
    % for j = 1 : fz.num
    %     total_K = total_K + fz.mbfun(j, o.x(:, i+1))*uav.K{j};
    % end
    % o.u(:, i+1) = total_K*(o.x(:, i+1) - o.xr(:, i+1));
end
% uav.tr.xb = xb;
uav.tr.t = t;
end

%% Local function
function k = RK4(uav, xb, i) % calculate dxdt
    % O       = zeros(uav.DIM_X);
    ddr     = uav.tr.r{3}(:, i);
    x       = xb(1 : uav.DIM_X3);
    xh      = xb(uav.DIM_X3 + (1 : uav.DIM_X3));
    X       = x(6+[1 3 5 7 9 11]);
    dX      = x(6+[2 4 6 8 10 12]);
    Xh      = xh(6+[1 3 5 7 9 11]);
    dXh     = xh(6+[2 4 6 8 10 12]);

    if uav.tr.IS_LINEAR % linear                 
        % Eb = [eye(uav.DIM_X) O; O uav.Br];
        % u = K*x + uav.M(X) + uav.H(X, dX) % control law
        % k = uav.A*xb + uav.B*u + Eb*[v(t); r];
    else % nonlinear
        M = uav.M(X);
        Mh = uav.M(Xh);
        H = uav.H(X, dX);
        Hh = uav.H(Xh, dXh);
        u = Mh*(ddr + uav.K*xh) + Hh; % control law
        f = -M\((M-Mh)*(ddr + uav.K*xh) + H-Hh);
        k = [
            uav.A*x + uav.B*(u + f)
            uav.A*xh + uav.B*u - uav.L*uav.C*(x-xh)
        ];
    end
end
function uav = trajectory(uav, fz)
%calculate trajectory

dt          = uav.tr.dt;
t           = 0 : dt : uav.tr.T;
t2          = 0 : dt : uav.tr.T + dt*4; % for numercial differention
LEN         = length(t);

%% calculate r, dr, ddr
% referecne trajectory of x, y, z
amp_z   = 1; % 0.8
amp     = 1; % 1
freg    = 0.5;

r1 = zeros(3, LEN+4);
for i = 1 : LEN+4
    r1(:, i) = [
        amp*sin(freg*t2(i))
        amp*cos(freg*t2(i))
        amp_z*t2(i)
    ];      
end
% dr1 = my_diff(r1)/dt;
% ddr1 = my_diff(dr1)/dt;

% ir1 = cumsum(t1)
dr1 = diff(r1,1,2)/dt;
ddr1 = diff(r1,2,2)/dt;

% referecne trajectory of phi, theta, psi
% note: Since UAV is underactuated (Systrm DoF:6, Control DoF:4), phi,
% theta can be calculated by inverse dynamic. UAV no need to spin -> psi = 0
r2 = zeros(3, LEN+2);
for i = 1 : LEN+2
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
% dr2 = my_diff(r2)/dt;
% ddr2 = my_diff(dr2)/dt;
dr2 = diff(r2,1,2)/dt; 
ddr2 = diff(r2,2,2)/dt;

r1       = r1(:, 1:LEN);
dr1      = dr1(:, 1:LEN);
ddr1     = ddr1(:, 1:LEN);
r2       = r2(:, 1:LEN);
dr2      = dr2(:, 1:LEN);
% ddr2   = ddr2(:, 1:LEN);
uav.tr.r = {[r1; r2], [dr1; dr2], [ddr1; ddr2]};

% Debug, see r(t) trajectory
% plot(t, uav.tr.r{1}(4:6,:))
% figure
% plot(t,r1)
% figure
% plot(t,dr1)
% figure
% plot(t,ddr1)
% legend

%% calculate disturbance
v           = 0.01*randn(uav.DIM_X, LEN) + 0;
uav.tr.v    = [zeros(uav.DIM_X, LEN); zeros(uav.DIM_X, LEN); v];

%% calculate x, xh
x           = zeros(uav.DIM_X3, LEN);
r           = [uav.tr.r{1}; uav.tr.r{2}];
x(uav.DIM_F+(1:2*uav.DIM_F)) = uav.tr.x0 - r(:, 1);
xh          = zeros(uav.DIM_X3, LEN);
xb          = [x; xh];
u           = zeros(uav.DIM_U, LEN);
disp('Ploting trajectory ...')
for i = 1 : LEN - 1       
    if mod(i, 1/dt) == 0
        disp(['t = ' num2str(i*dt)])
    end
    if isnan(xb(:, i))
        disp(['traj has NaN, i = ' num2str(i)])
    end
    if norm(xb(:, i)) == Inf
        disp(['traj has Inf, i = ' num2str(i)])
    end

    k1 = RK4(uav, xb(:, i), i);
    if uav.tr.IS_RK4
        % k2 = RK4(uav, fz, xb(:, i)+k1*dt/2, i*dt + dt/2);
        % k3 = RK4(uav, fz, xb(:, i)+k2*dt/2, i*dt + dt/2);
        % k4 = RK4(uav, fz, xb(:, i)+k3*dt, i*dt + dt); 

        % xb(:, i+1)  = xb(:, i)  + (k1 + 2*k2 + 2*k3 + k4)*dt/6;
    end
    xb(:, i+1) = xb(:, i) + k1*dt;

    % total_K = zeros(o.DIM_U, o.DIM_X);
    % for j = 1 : fz.num
    %     total_K = total_K + fz.mbfun(j, o.x(:, i+1))*uav.K{j};
    % end
    % o.u(:, i+1) = total_K*(o.x(:, i+1) - o.xr(:, i+1));
end
uav.tr.x = xb(1:uav.DIM_X3, :);
uav.tr.xh = xb(uav.DIM_X3 + (1:uav.DIM_X3), :);
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
        u_PID = uav.K*xh;
        u = Mh*(ddr + u_PID) + Hh; % control law
        f = -M\((M-Mh)*(ddr + uav.K*xh) + H-Hh);
        k = [
            uav.A*x + uav.B*(u + f)
            % uav.A*x + uav.B*u_PID
            uav.A*xh + uav.B*u_PID - uav.L*uav.C*(x-xh)
        ];
    end
end
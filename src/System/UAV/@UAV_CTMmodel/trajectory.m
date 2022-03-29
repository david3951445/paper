function uav = trajectory(uav, fz)
%calculate trajectory

dt          = uav.tr.dt;
t           = 0 : dt : uav.tr.T;
t2          = 0 : dt : uav.tr.T + dt*4; % for numercial differention
LEN         = length(t);
DIM_F       = uav.DIM_F;
DIM_X       = uav.DIM_X;
DIM_X3      = uav.DIM_X3;

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
v           = 0.01*randn(DIM_X, LEN) + 0;
uav.tr.v    = [zeros(DIM_X, LEN); zeros(DIM_X, LEN); v];

%% calculate x, xh
x           = zeros(DIM_X3, LEN);
r           = [uav.tr.r{1}; uav.tr.r{2}];
xh          = zeros(DIM_X3, LEN);

% u           = zeros(uav.DIM_U, LEN);
% initial value of x
x(DIM_F+(1:2*DIM_F), 1) = uav.tr.x0;
X       = x(DIM_F + (1:DIM_F), 1);
dX      = x(2*DIM_F + (1:DIM_F), 1);
Xh      = xh(DIM_F + (1:DIM_F), 1);
dXh     = xh(2*DIM_F + (1:DIM_F), 1);
ddr0    = uav.tr.r{3}(:, 1);
M       = uav.M(X);
Mh      = uav.M(Xh);
H       = uav.H(X, dX);
Hh      = uav.H(Xh, dXh);
f       = -M\((M-Mh)*(ddr0 + uav.K*xh(:, 1)) + H-Hh);
x(3*DIM_F + (1:DIM_F), 1) = f;

disp('Ploting trajectory ...')
xb = [x; xh];
for i = 1 : LEN - 1
    if mod(i, 1/dt) == 0
        disp(['t = ' num2str(i*dt)])
    end
    if isnan(xb(:, i))
        disp(['traj has NaN, t = ' num2str(i*dt)])
        uav.tr.LEN = i;
        break
    end
    if norm(xb(:, i)) == Inf
        disp(['traj has Inf, t = ' num2str(i*dt)])
        uav.tr.LEN = i;
        break
    end

    [k1, f] = RK4(uav, xb(:, i), i);
    if uav.tr.IS_RK4
        % k2 = RK4(uav, fz, xb(:, i)+k1*dt/2, i*dt + dt/2);
        % k3 = RK4(uav, fz, xb(:, i)+k2*dt/2, i*dt + dt/2);
        % k4 = RK4(uav, fz, xb(:, i)+k3*dt, i*dt + dt); 

        % xb(:, i+1)  = xb(:, i)  + (k1 + 2*k2 + 2*k3 + k4)*dt/6;
    end
    xb(:, i+1) = xb(:, i) + k1*dt;
    xb(3*DIM_F + (1:DIM_F), i+1) = f;
    % disp(['xb', num2str(xb(:, i)')])
end

uav.tr.x = xb(1:DIM_X3, :);
uav.tr.xh = xb(DIM_X3 + (1:DIM_X3), :);
uav.tr.t = t;
uav.tr.LEN = i  ;
end

%% Local function
function [k, f] = RK4(uav, xb, i) % calculate dxdt
    % O       = zeros(uav.DIM_X);
    DIM_F   = uav.DIM_F;

    r       = uav.tr.r{1}(:, i);
    dr      = uav.tr.r{2}(:, i);
    ddr     = uav.tr.r{3}(:, i);
    x       = xb(1 : uav.DIM_X3);
    xh      = xb(uav.DIM_X3 + (1:uav.DIM_X3));
    X       = x(DIM_F + (1:DIM_F));
    dX      = x(2*DIM_F + (1:DIM_F));
    Xh      = xh(DIM_F + (1:DIM_F));
    dXh     = xh(2*DIM_F + (1:DIM_F));

    if uav.tr.IS_LINEAR % linear                 
        % Eb = [eye(uav.DIM_X) O; O uav.Br];
        % u = K*x + uav.M(X) + uav.H(X, dX) % control law
        % k = uav.A*xb + uav.B*u + Eb*[v(t); r];
    else % nonlinear
        M = uav.M(X+r);
        Mh = uav.M(Xh+r);
        H = uav.H(X+r, dX+dr);
        Hh = uav.H(Xh+r, dXh+dr);
        u_PID = uav.K*xh;
        % u = Mh*(ddr + u_PID) + Hh; % control law
        f = -M\((M-Mh)*(ddr + uav.K*xh) + H-Hh);
        k = [
%             uav.A*x + uav.B*(u + f)
            uav.A*x + uav.B*u_PID
            uav.A*xh + uav.B*u_PID - uav.L*uav.C*(x-xh)
        ];
    end
end
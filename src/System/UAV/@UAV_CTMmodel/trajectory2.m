function uav = trajectory2(uav, fz)
%calculate trajectory

dt          = uav.tr.dt;
t           = 0 : dt : uav.tr.T;
LEN         = length(t);
DIM_F       = uav.DIM_F;
DIM_X       = uav.DIM_X;
DIM_X3      = uav.DIM_X3;

%% calculate disturbance
v           = 0.01*randn(DIM_X, LEN) + 0;
uav.tr.v    = [zeros(DIM_X, LEN); zeros(DIM_X, LEN); v];

%% calculate x, xh
x           = zeros(DIM_X3, LEN);
xh          = zeros(DIM_X3, LEN);
r           = zeros(DIM_F, LEN);
uav.tr.r    = {r, r, r};

%% initial value of x
X       = uav.tr.x0(1:DIM_F, 1);
dX      = uav.tr.x0(DIM_F + (1:DIM_F), 1);
Xh      = xh(DIM_F + (1:DIM_F), 1);
dXh     = xh(2*DIM_F + (1:DIM_F), 1);
ddr0    = uav.tr.r{3}(:, 1);
M       = uav.M(X);
Mh      = uav.M(Xh);
H       = uav.H(X, dX);
Hh      = uav.H(Xh, dXh);
f       = -M\((M-Mh)*(ddr0 + uav.K*xh(:, 1)) + H-Hh);
x(3*DIM_F + (1:DIM_F), 1) = f;
x(:, 1) = [
    zeros(DIM_F, 1)
    X
    dX
    f
    f
];

%% initial value of r
% Since only have information of r at i = 1, dr and ddr need to be zero, i.e. r[-1] = r[0] = r[1]
% i = 2, we can obtain dr. i = 3, we can obtain ddr.
amp_z   = 1; % 0.8
amp     = 1; % 1
freg    = 1;
% r = [
%     amp*sin(freg*t(1))
%     amp*cos(freg*t(1))
%     amp_z*t(1) + 1
%     0
%     0
%     0
% ];
% r_old = repmat(r,[1 3]); % for numercial differentiation of r[k]
% uav.tr.r{1}(:, 1) = r;
% uav.tr.r{2}(:, 1) = 0;
% uav.tr.r{3}(:, 1) = 0;

%% control gain test
A = [0 1 0; 0 0 1; 0 0 0];
B = [0; 0; 1];
A = kron(A, eye(DIM_F)); 
B = kron(B, eye(DIM_F)); 
Q = diag(1*[.1 100 10]); % I, P, D
Q = kron(Q, diag([1 1 1 .01 .01 .01])); % x, y, z, phi, theta, psi
R = [];
rho = 100;
K = solveLMI6(A,B,Q,R,rho);
uav.debug.K = K;

%% trajectory
disp('Calculating trajectory ...')
xb = [x; xh];
uav.tr.t = t;
for i = 1 : LEN - 1
    %% debug message
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

    %% extract x, xh, X, dX form xb
    x       = xb(1 : uav.DIM_X3, i);
    % xh      = xb(uav.DIM_X3 + (1:uav.DIM_X3));

    X       = x(DIM_F + (1:DIM_F));
    dX      = x(2*DIM_F + (1:DIM_F));
    % Xh      = xh(DIM_F + (1:DIM_F));
    % dXh     = xh(2*DIM_F + (1:DIM_F));
    %% control 
    u = K*x(1:3*DIM_F);

    %% In practice, r[k] get from path planning algorithm
    r = [
        amp*sin(freg*t(i))
        amp*cos(freg*t(i))
        amp_z*t(i) + 1
        0
        0
        0
    ];
    if i == 1 % Since only have information of r(4:6) at i = 1
        r_old(1:3, :) = repmat(r(1:3), [1 3]);
    end

    %% In practice, dr and ddr are obtained from numercial differentiation
    % d and dd of x, y, z
    dr = zeros(6, 1); ddr = dr;
    dr(1:3) = [r(1:3) r_old(1:3, 1)]*[1 -1]'/dt; % finite different, precision: o(h)
    ddr(1:3) = [r(1:3) r_old(1:3, 1:2)]*[1 -2 1]'/dt^2;
    % 0, d and dd of phi, theta, psi
    c = uav.m*eye(3)*ddr(1:3) + [0; 0; -uav.m*uav.G] + u(1:3) + uav.Df*dX(1:3);
    % uav.debug.c(:, i) = c;
    r(5) = atan(c(1)/c(3));
    % r(5) = c(1)/c(3);
    r(4) = atan(-c(2)*cos(r(5))/c(3));
    % r(4) = -c(2)*cos(r(5))/c(3);
    if i == 1 % Since only have information of r(4:6) at i = 1
        r_old(4:6, :) = repmat(r(4:6), [1 3]);
    end
    dr(4:6) = [r(4:6) r_old(4:6, 1)]*[1 -1]'/dt; % finite different, precision: o(h)
    ddr(4:6) = [r(4:6) r_old(4:6, 1:2)]*[1 -2 1]'/dt^2;
    if i < 3
        [r(1:3) r_old(1:3, 1:2)]
    end
    uav.tr.r{1}(:, i) = r;
    uav.tr.r{2}(:, i) = dr;
    uav.tr.r{3}(:, i) = ddr;
    F = sqrt(c(1)^2 + c(2)^2 + c(3)^2);

    M = uav.M(X+r);
    % Mh = uav.M(Xh+r);
    f = randn(DIM_F, 1);
    k = [
        A*x(1:3*DIM_F) + B*(u + f)
        zeros(DIM_X3-DIM_F*3, 1)
        zeros(DIM_X3, 1)
    ];
    % Hh = uav.H(Xh+r, dXh+dr);
    % u_PID = uav.K*xh;
    % u = Mh*(ddr + u_PID) + Hh; % control law
    % f = -M\((M-Mh)*(ddr + uav.K*xh) + H-Hh);
    % k = [
    %     uav.A*x + uav.B*u_PID
    %     uav.A*xh + uav.B*u_PID - uav.L*uav.C*(x-xh)
    % ];

    xb(:, i+1) = xb(:, i) + k*dt;
    xb(3*DIM_F + (1:DIM_F), i+1) = f;
    r_old = [r r_old(:, 1:size(r_old, 2)-1)]; % update old r
end

uav.tr.x = xb(1:DIM_X3, :);
uav.tr.xh = xb(DIM_X3 + (1:DIM_X3), :);
uav.tr.LEN = i;
end

%% Local function
function [k, f] = RK4(uav, xb, i) % calculate dxdt
    % O       = zeros(uav.DIM_X);
          
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
function uav = trajectory2(uav)
%calculate trajectory

dt          = uav.tr.dt;
t           = 0 : dt : uav.tr.T;
LEN         = length(t);
DIM_F       = uav.DIM_F;
DIM_X       = uav.DIM_X;
DIM_X3      = uav.DIM_X3;
startTime   = 5;

%% control gain test
% A = [0 1 0; 0 0 1; 0 0 0];
% B = [0; 0; 1];
% A = kron(A, eye(DIM_F)); 
% B = kron(B, eye(DIM_F)); 
% Q = diag(1*[.1 100 10]); % I, P, D
% Q = kron(Q, diag([1 1 1 .01 .01 .01])); % x, y, z, phi, theta, psi
% R = [];
% rho = 100;
% K = solveLMI6(A,B,Q,R,rho);
% uav.K = K;

%% set xd, yd, zd, phid
r4 = zeros(4, LEN);
amp_z   = 0.9;
amp     = 0.8;
freg    = 1;
for i = 1 : LEN - 1
    r4(:, i) = [
        amp*sin(freg*t(i))
        amp*cos(freg*t(i))
        amp_z*t(i) + 1
        0
    ];
end

%% calculate disturbance
v           = 0.01*randn(DIM_F, LEN) + 5;
uav.tr.v    = v;

%% initialize x, xh, r
x           = zeros(DIM_X3, LEN);
x2          = zeros(DIM_F*2, LEN);
xh          = zeros(DIM_X3, LEN);
r           = zeros(DIM_F, LEN);
uav.tr.r    = {r, r, r};
uav.tr.f    = zeros(DIM_X3, LEN);

%% initial value of x
% Hence UAV will stop in place when i < 5
% X       = uav.tr.x0(1:DIM_F, 1);
% dX      = uav.tr.x0(DIM_F + (1:DIM_F), 1);
% Xh      = uav.tr.xh0(1:DIM_F, 1);
% dXh     = uav.tr.xh0(DIM_F + (1:DIM_F), 1);
% M       = uav.M(X);
% Mh      = uav.M(Xh);
% H       = uav.H(X, dX);
% Hh      = uav.H(Xh, dXh);
% f       = -M\((M-Mh)*(ddr0 + uav.K*xh(:, 1)) + H-Hh);
% x(3*DIM_F + (1:DIM_F), 1) = f;
x(:, 1:startTime) = repmat(uav.tr.x0, [1 startTime]);
x2(:, 1:startTime) = repmat(uav.tr.x0(DIM_F + (1:DIM_F*2)), [1 startTime]);

%% initial value of r
% Let r = [x y z phi theta psi]'
% Since only have information of x, y, z and phi, i.e. r([1 2 3 6]) at i = 1, The UAV will statrt at i = 4.
%   i > 2, we can obtain dr([1 2 3 6])
%   i > 3, we can obtain ddr([1 2 3 6]) and r([4 5])
%   i > 4, we can obtain dr([4 5])
% r_old is used to save previous two r, i.e. r(:, i-1) and r(:, i-2) 
%   i > 5, with current r(:, i), the r, dr, drr can all obtain form finite different method

% construct r_old
r_old = zeros(DIM_F, 2); % store r[i-1], r[i-2]
for i = 3 : startTime-1
    [phi, theta, ~] = uav.pos_controller(x(:, i), r4(1:3, i-2:i), dt);
    r = [
        r4(1, i)
        r4(2, i)
        r4(3, i)
        phi
        theta
        r4(4, i)
    ];

    % ddr(4:6) = [r(4:6) r_old(4:6, 1:2)]*coeff'/dt^2;
    % uav.tr.r{1}(:, i) = r;
    % uav.tr.r{2}(:, i) = dr;
    % uav.tr.r{3}(:, i) = ddr;
    % F = sqrt(c(1)^2 + c(2)^2 + c(3)^2);
    r_old = [r r_old(:, 1 : size(r_old, 2)-1)]; % update old r
end

%% trajectory
disp('Calculating trajectory ...')
xb = [x; xh];
uav.tr.t = t;
for i = startTime : LEN - 1
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
    xh      = xb(uav.DIM_X3 + (1:uav.DIM_X3), i);
    X       = x(DIM_F + (1:DIM_F));
    dX      = x(2*DIM_F + (1:DIM_F));
    Xh      = xh(DIM_F + (1:DIM_F));
    dXh     = xh(2*DIM_F + (1:DIM_F));

    %% reference
    [phi, theta, F] = uav.pos_controller(xh, r4(1:3, i-2:i), dt);
    r = [
        r4(1, i)
        r4(2, i)
        r4(3, i)
        phi
        theta
        r4(4, i)
    ];
    % In practice, dr and ddr are obtained from numercial differentiation
    dr = [r r_old(:, 1)]*[1 -1]'/dt; % finite different, precision: o(h)
    ddr = [r r_old(:, 1:2)]*[1 -2 1]'/dt^2;
    r_old = [r r_old(:, 1 : size(r_old, 2)-1)]; % update old r
    uav.tr.r{1}(:, i) = r;
    uav.tr.r{2}(:, i) = dr;
    uav.tr.r{3}(:, i) = ddr;
    
    u = uav.K*xh;
    M = uav.M(X+r);
    Mh = uav.M(Xh+r);
    H = uav.H(X+r, dX+dr);
    Hh = uav.H(Xh+r, dXh+dr);
    uav.R(X(4:6))*[0; 0; F]-u(1:3)
    u4 = [u(1:3); u(4:6)];
    % u = Mh*(ddr + u_PID) + Hh; % control law
    f = -eye(DIM_F)/M*((M-Mh)*(ddr + u4) + H-Hh + v(:, i));
    % uav.tr.f(:, i) = f;

    k = [
        uav.A*x + uav.B*u4
        uav.A*xh + uav.B*u4 - uav.L*uav.C*(x-xh)
    ];

    k2 = uav.f(x) + uav.g(x)*[F; u(4:6)];
    x2(:, i+1) = x2(:, i) + k2*dt;
    xb(:, i+1) = xb(:, i) + k*dt;
    xb(3*DIM_F + (1:DIM_F), i+1) = f;
end

uav.tr.x = xb(1:DIM_X3, :);
uav.tr.xh = xb(DIM_X3 + (1:DIM_X3), :);
% uav.tr.LEN = i;
uav.tr.f = f;
uav.tr.x2 = x2;
end

%% Local function
% function [k, f] = RK4(uav, xb, i) % calculate dxdt
%     % O       = zeros(uav.DIM_X);
          
%     dr      = uav.tr.r{2}(:, i);
%     ddr     = uav.tr.r{3}(:, i);
%     x       = xb(1 : uav.DIM_X3);
%     xh      = xb(uav.DIM_X3 + (1:uav.DIM_X3));
%     X       = x(DIM_F + (1:DIM_F));
%     dX      = x(2*DIM_F + (1:DIM_F));
%     Xh      = xh(DIM_F + (1:DIM_F));
%     dXh     = xh(2*DIM_F + (1:DIM_F));

%     if uav.tr.IS_LINEAR % linear                 
%         % Eb = [eye(uav.DIM_X) O; O uav.Br];
%         % u = K*x + uav.M(X) + uav.H(X, dX) % control law
%         % k = uav.A*xb + uav.B*u + Eb*[v(t); r];
%     else % nonlinear
%         M = uav.M(X+r);
%         Mh = uav.M(Xh+r);
%         H = uav.H(X+r, dX+dr);
%         Hh = uav.H(Xh+r, dXh+dr);
%         u_PID = uav.K*xh;
%         % u = Mh*(ddr + u_PID) + Hh; % control law
%         norm(M-Mh)
%         norm(H-Hh)
%         f = -M\((M-Mh)*(ddr + uav.K*xh) + H-Hh);
%         k = [
% %             uav.A*x + uav.B*(u + f)
%             uav.A*x + uav.B*u_PID
%             uav.A*xh + uav.B*u_PID - uav.L*uav.C*(x-xh)
%         ];
%     end
% end
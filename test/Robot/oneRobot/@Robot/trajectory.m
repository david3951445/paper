function rb = trajectory(rb)
%calculate trajectory

dt          = rb.tr.dt;
t           = 0 : dt : rb.tr.T;
LEN         = length(t);
DIM_F       = rb.DIM_F;
DIM_X       = rb.DIM_X;
DIM_X3      = rb.DIM_X3;
startTime   = 3;

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
% rb.K = K;

%% calculate disturbance
v           = 0.01*randn(DIM_F, LEN) + 5;
rb.tr.v    = v;

%% initialize x, xh, r
x           = zeros(DIM_X3, LEN);
x2          = zeros(DIM_F*2, LEN);
xh          = zeros(DIM_X3, LEN);
r           = zeros(DIM_F, LEN);
rb.tr.r    = {r, r, r};
rb.tr.f     = zeros(DIM_X3, LEN);

%% initial value of x
x(:, 1:startTime) = repmat(rb.tr.x0, [1 startTime]);
x2(:, 1:startTime) = repmat(rb.tr.x0(DIM_F + (1:DIM_F*2)), [1 startTime]);

%% set r from as joint ref
qr = rb.qr;

% construct r_old
r_old = zeros(DIM_F, 2); % store r[i-1], r[i-2]
for i = 1 : startTime-1
    r = qr(:, i);
    % ddr(4:6) = [r(4:6) r_old(4:6, 1:2)]*coeff'/dt^2;
    % rb.tr.r{1}(:, i) = r;
    % rb.tr.r{2}(:, i) = dr;
    % rb.tr.r{3}(:, i) = ddr;
    % F = sqrt(c(1)^2 + c(2)^2 + c(3)^2);
    r_old = [r r_old(:, 1 : size(r_old, 2)-1)]; % update old r
end

%% trajectory
disp('Calculating trajectory ...')
xb = [x; xh];
rb.tr.t = t;
for i = startTime : LEN - 1
    %% debug message
    if mod(i, 1/dt) == 0
        disp(['t = ' num2str(i*dt)])
    end
    if isnan(xb(:, i))
        disp(['traj has NaN, t = ' num2str(i*dt)])
        rb.tr.LEN = i;
        break
    end
    if norm(xb(:, i)) == Inf
        disp(['traj has Inf, t = ' num2str(i*dt)])
        rb.tr.LEN = i;
        break
    end

    %% extract x, xh, X, dX form xb
    x       = xb(1 : rb.DIM_X3, i);
    xh      = xb(rb.DIM_X3 + (1:rb.DIM_X3), i);
    X       = x(DIM_F + (1:DIM_F));
    dX      = x(2*DIM_F + (1:DIM_F));
    Xh      = xh(DIM_F + (1:DIM_F));
    dXh     = xh(2*DIM_F + (1:DIM_F));
    r       = qr(:, i);

    %% reference
    % In practice, dr and ddr are obtained from numercial differentiation
    dr = [r r_old(:, 1)]*[1 -1]'/dt; % finite different, precision: o(h)
    ddr = [r r_old(:, 1:2)]*[1 -2 1]'/dt^2;
    r_old = [r r_old(:, 1 : size(r_old, 2)-1)]; % update old r
    rb.tr.r{1}(:, i) = r;
    rb.tr.r{2}(:, i) = dr;
    rb.tr.r{3}(:, i) = ddr;

    u = rb.K*xh; % PID control law
    M = rb.M(X+r);
    Mh = rb.M(Xh+r);
    H = rb.H(X+r, dX+dr);
    Hh = rb.H(Xh+r, dXh+dr);
    f = -eye(DIM_F)/M*((M-Mh)*(ddr + u) + H-Hh + v(:, i));
    % rb.tr.f(:, i) = f;

    k = [
        rb.A*x + rb.B*u
        rb.A*xh + rb.B*u - rb.KL*rb.C*(x-xh)
    ];

    xb(:, i+1) = xb(:, i) + k*dt;
    xb(3*DIM_F + (1:DIM_F), i+1) = f;
end

rb.tr.x = xb(1:DIM_X3, :);
rb.tr.xh = xb(DIM_X3 + (1:DIM_X3), :);
% rb.tr.LEN = i;
rb.tr.f = f;
rb.tr.x2 = x2;
end

%% Local function
% function [k, f] = RK4(rb, xb, i) % calculate dxdt
%     % O       = zeros(rb.DIM_X);
          
%     dr      = rb.tr.r{2}(:, i);
%     ddr     = rb.tr.r{3}(:, i);
%     x       = xb(1 : rb.DIM_X3);
%     xh      = xb(rb.DIM_X3 + (1:rb.DIM_X3));
%     X       = x(DIM_F + (1:DIM_F));
%     dX      = x(2*DIM_F + (1:DIM_F));
%     Xh      = xh(DIM_F + (1:DIM_F));
%     dXh     = xh(2*DIM_F + (1:DIM_F));

%     if rb.tr.IS_LINEAR % linear                 
%         % Eb = [eye(rb.DIM_X) O; O rb.Br];
%         % u = K*x + rb.M(X) + rb.H(X, dX) % control law
%         % k = rb.A*xb + rb.B*u + Eb*[v(t); r];
%     else % nonlinear
%         M = rb.M(X+r);
%         Mh = rb.M(Xh+r);
%         H = rb.H(X+r, dX+dr);
%         Hh = rb.H(Xh+r, dXh+dr);
%         u_PID = rb.K*xh;
%         % u = Mh*(ddr + u_PID) + Hh; % control law
%         norm(M-Mh)
%         norm(H-Hh)
%         f = -M\((M-Mh)*(ddr + rb.K*xh) + H-Hh);
%         k = [
% %             rb.A*x + rb.B*(u + f)
%             rb.A*x + rb.B*u_PID
%             rb.A*xh + rb.B*u_PID - rb.KL*rb.C*(x-xh)
%         ];
%     end
% end
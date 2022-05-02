function rb = trajectory(rb)
%calculate trajectory

sys         = rb.sys;
sys_a       = rb.sys_a;
sys_s       = rb.sys_s;
dt          = rb.tr.dt;
LEN         = length(rb.qr);
t           = 0 : dt : dt*(LEN-1);
DIM_F       = 1;
DIM_X       = rb.sys.DIM_X;
DIM_X3      = rb.sys_aug.DIM_X;
startTime   = 3; % For calculate ddr(t), start at 3-rd step (k = 3)

%% set disturbance
v1 = repmat(0.2*cos(1*t), sys_a.DIM, 1);
v1_init = [repmat(v1(:, 1), 1, sys_a.WINDOW) v1];
v2 = repmat(0.1*sin(1*t), sys_s.DIM, 1);
v2_init = [repmat(v2(:, 1), 1, sys_s.WINDOW) v2];
rb.tr.v1    = v1;
rb.tr.v2    = v2;

%% initialize x, xh, r, f
x           = zeros(DIM_X3, LEN);
% x2          = zeros(DIM_F*2, LEN);
xh          = zeros(DIM_X3, LEN);
r           = zeros(DIM_F, LEN);
rb.tr.r     = {r, r, r};
rb.tr.f1    = zeros(sys_a.DIM, LEN);
rb.tr.f2    = zeros(sys_s.DIM, LEN);

%% initial value of x
x(:, 1:startTime) = repmat(rb.tr.x0, [1 startTime]);
% x2(:, 1:startTime) = repmat(rb.tr.x0(DIM_F + (1:DIM_F*2)), [1 startTime]);

%% Construct previous state of r(t)
% To calculate dr, ddr, we need perious state
r_old = zeros(DIM_F, 2); % store r[i-1], r[i-2]
for i = 1 : startTime-1 % i = 2, we have dr. i = 3, we have ddr.
    r = rb.qr(:, i);
    r_old = [r r_old(:, 1 : size(r_old, 2)-1)]; % update old r
end

%% trajectory
disp(['Calculating trajectory ..., t = 0 ~ ' num2str(dt*(LEN-1))])
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
    x       = xb(1 : DIM_X3, i);
    xh      = xb(DIM_X3 + (1:DIM_X3), i);
    X       = x(DIM_F + (1:DIM_F)); % position
    dX      = x(2*DIM_F + (1:DIM_F)); % velocity
    Xh      = xh(DIM_F + (1:DIM_F));
    dXh     = xh(2*DIM_F + (1:DIM_F));
    
    %% reference
    % In practice, dr and ddr are obtained from numercial differentiation
    r = rb.qr(:, i);
    dr = [r r_old(:, 1)]*[1 -1]'/dt; % finite different, precision: o(h)
    ddr = [r r_old(:, 1:2)]*[1 -2 1]'/dt^2;
    r_old = [r r_old(:, 1 : size(r_old, 2)-1)]; % update old r
    rb.tr.r{1}(:, i) = r;
    rb.tr.r{2}(:, i) = dr;
    rb.tr.r{3}(:, i) = ddr;

    %% unknown signal
    u = rb.u_PID(xh); % PID control
    % M = rb.M(X+r);
    % Mh = rb.M(Xh+r);
    % H = rb.H(X+r, dX+dr);
    % Hh = rb.H(Xh+r, dXh+dr);
    % f = -eye(DIM_F)/M*((M-Mh)*(ddr + u) + H-Hh + v(:, i));
    rb.tr.f1(:, i) = v1(:, i);
    rb.tr.f2(:, i) = v2(:, i);

    xb(:, i+1) = ODE_solver(@rb.f_aug, dt, [x; xh], t(i), 'RK4');

    % xb(:, i+1) = xb(:, i) + fun(t(i), xb(:, i))*dt;
    range = i : i + sys_a.WINDOW-1;
    v1_ = flip(reshape(v1_init(:, range)', [], 1));
    xb(sys.DIM_X+(1:sys_a.DIM_X), i+1) = v1_;
    range = i : i + sys_s.WINDOW-1;
    v2_ = flip(reshape(v2_init(:, range)', [], 1));
    xb(sys.DIM_X+sys_a.DIM_X+(1:sys_s.DIM_X), i+1) = v2_;
end

rb.tr.x = xb(1:DIM_X3, :);
rb.tr.xh = xb(DIM_X3 + (1:DIM_X3), :);
% rb.tr.LEN = i;
end 
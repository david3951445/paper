function rb = trajectory(rb)
%calculate trajectory

sys         = rb.sys;
sys_a       = rb.sys_a;
sys_s       = rb.sys_s;
dt          = rb.tr.dt;
LEN         = length(rb.qr);
t           = 0 : dt : dt*(LEN-1);
DIM_F       = rb.DIM_F;
DIM_X       = rb.sys.DIM_X;
DIM_X3      = rb.sys_aug.DIM_X;
startTime   = 3; % For calculate ddr(t), start at 3-rd step (k = 3)

%% set disturbance
v1 = repmat(.2*sin(t), sys_a.DIM, 1) ;
% v2 = repmat(.05*sin(1*t), sys_s.DIM, 1);
v2 = 0.05*ones(sys_s.DIM, length(t));
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
        % rb.tr.LEN = i;
        break
    end
    if norm(xb(:, i)) == Inf
        disp(['traj has Inf, t = ' num2str(i*dt)])
        % rb.tr.LEN = i;
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
    rb.tr.r{1}(:, i+1) = r;
    rb.tr.r{2}(:, i+1) = dr;
    rb.tr.r{3}(:, i+1) = ddr;

    %% fault signal
    u = rb.u_PID(xh); % PID control
    M = rb.M(X+r);
    % Mh = rb.M(Xh+r);
    % H = rb.H(X+r, dX+dr);
    % Hh = rb.H(Xh+r, dXh+dr);
    % f = -eye(DIM_F)/M*((M-Mh)*(ddr + u) + H-Hh + v1(:, i));
    % f = -eye(DIM_F)/M*((M-Mh)*(ddr + u) + v1(:, i));
    f = -eye(DIM_F)/M*(v1(:, i));
    % if mod(i, 100) == 0
    %     disp(['norm of M-Mh: ' num2str(norm(M-Mh))])
    % end
    % f = v1(:, i);
    
    xb(:, i+1) = ODE_solver(@rb.f_aug, dt, [x; xh], t(i), 'RK4');
    % xb(:, i+1) = xb(:, i) + fun(t(i), xb(:, i))*dt;

    %% assign real fault signal
    % Since the real fault signal is not produced by smooth model, we need reassign it.
    % actuator fault
    j0 = sys.DIM_X;
    range = 1 : sys_a.DIM;
    for j = 1 : sys_a.WINDOW-1
        j1 = j0 + j*sys_a.DIM;
        xb(j1+range, i+1) = xb(j1-sys_a.DIM+range, i);
    end
    xb(j0+range, i+1) = f;
    % sensor fault
    j0 = sys.DIM_X + sys_a.DIM_X;
    range = 1 : sys_s.DIM;
    for j = 1 : sys_s.WINDOW-1
        j1 = j0 + j*sys_s.DIM;
        xb(j1+range, i+1) = xb(j1-sys_s.DIM+range, i);
    end
    xb(j0+range, i+1) = v2(:, i);

    %% save data
    rb.tr.f1(:, i) = f;
    rb.tr.f2(:, i) = v2(:, i);
end

rb.tr.x = xb(1:DIM_X3, :);
rb.tr.xh = xb(DIM_X3 + (1:DIM_X3), :);
% rb.tr.LEN = i;
end 
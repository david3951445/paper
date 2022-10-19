function uav = trajectory(uav)
%calculate trajectory

sys_a       = uav.sys_a;
sys_s       = uav.sys_s;
dt          = uav.tr.dt;
t           = uav.tr.t;
LEN         = uav.tr.LEN;
DIM_F       = uav.DIM_F;
DIM_X       = uav.sys.DIM_X;
DIM_X3      = uav.sys_aug.DIM_X;
startTime   = 5; % For calculate ddr(t), start at 3-rd step (k = 3)

%% set disturbance
%-1 actuator fault
d1 = repmat(100*sin(3*t), sys_a.DIM, 1);
sys_a.fault = zeros(sys_a.DIM, LEN);

%-2 sensor fault
% square wave
% uav.tr.f2    = 0.1*ones(sys_s.DIM, uav.tr.LEN);
% b = [0.1 -0.05 0.05]; n = length(b)+1;
% a = round(linspace(1,uav.tr.LEN,n));
% for i = 1 : n-1
%     sys_s.fault(:, a(i):a(i+1)) = b(i);
% end

% smoothed square wave
t_ = linspace(0, uav.tr.T, 50);
x_ = 1*square(.5*t_, 60);
fx = fit(t_', x_', 'SmoothingSpline');
x3 = feval(fx, uav.tr.t)';
sys_s.fault = repmat(x3, sys_s.DIM, 1);

% r = uav.qr;
% dr = diff(r, 1, 2)/dt;
% dr = filloutliers(dr, 'pchip', 'movmedian', 10); % since numercial differenciation will produce outliers if it's not smooth.
% ddr = diff(dr, 1, 2)/dt;
% ddr = filloutliers(ddr, 'pchip', 'movmedian', 10);
% dr = [zeros(DIM_F, 1) dr]; % diff() will result in 1 missing data, resize it
% ddr = [zeros(DIM_F, 2) ddr];
% uav.tr.r     = cell(1,3);
% uav.tr.r{1} = [r(1:3,:); zeros(DIM_F,LEM); r(4,:)];
% uav.tr.r{2} = [dr(1:3,:); zeros(DIM_F,LEM); dr(4,:)];
% uav.tr.r{3} = [ddr(1:3,:); zeros(DIM_F,LEM); ddr(4,:)];

%% initialize x, xh, u
x           = zeros(DIM_X3, LEN);
xh          = zeros(DIM_X3, LEN);
x(:, 1:startTime) = repmat(uav.tr.x0, [1 startTime]);
u           = zeros(uav.sys.DIM_U, LEN);

% construct r_old
r_old = zeros(DIM_F, 2); % store r[i-1], r[i-2]
r4 = uav.qr;
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

    r_old = [r r_old(:, 1 : size(r_old, 2)-1)]; % update old r
end

%% trajectory
disp(['Calculating trajectory ..., t = 0 ~ ' num2str(dt*(LEN-1))])
xb = [x; xh];
uav.tr.r = cell(1,3);
uav.tr.u = zeros(uav.sys.DIM_U, LEN);
for i = startTime : LEN - 1
    %% debug message
    if mod(i, 1/dt) == 0
        disp(['t = ' num2str(i*dt)])
    end
    if isnan(xb(:, i))
        disp(['traj has NaN, t = ' num2str(i*dt)])
        % uav.tr.LEN = i;
        break
    end
    if norm(xb(:, i)) == Inf
        disp(['traj has Inf, t = ' num2str(i*dt)])
        % uav.tr.LEN = i;
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
    uav.tr.r{1}(:, i+1) = r;
    uav.tr.r{2}(:, i+1) = dr;
    uav.tr.r{3}(:, i+1) = ddr;

    %% fault signal
    % feedback linearized term: Mh(), Hh(). By testing, using feedforward only ( Mh(r(t)) ) is more stable then feedback + feedforward ( Mh(xh(t)+r(t)) ).
    u = uav.u_fb(xh);
    uav.tr.u(:,i+1) = u;
    M = uav.M(X+r);
    Mh = uav.M(r);
    % Mh = uav.M(Xh+r);
    % Mh = 0;
    
    H = uav.H(X+r, dX+dr);
    Hh = uav.H(r, dr);
    % Hh = uav.H(Xh+r, dXh+dr); % diverge
    % Hh = uav.H(Xh+r, dr);
    % Hh = uav.H(X+r, dXh+dr);
    % Hh = 0;
    
    sys_a.fault(:, i) = -eye(DIM_F)/M*((M-Mh)*(ddr + u) + H-Hh - d1(:, i));
    % uav.sys_a.fault = -eye(DIM_F)/M*((M-Mh)*(ddr + u) + uav.tr.f1(:, i));
    % uav.sys_a.fault = -eye(DIM_F)/M*(uav.tr.f1(:, i));

    % Show norm of error terms
    if mod(i, 100) == 0 
        % disp(['norm of f1: ' num2str(norm(sys_a.fault(:, i)))])
        % disp(['norm of M-Mh: ' num2str(norm(M-Mh))])
        % disp(['norm of H-Hh: ' num2str(norm(H-Hh))])
    end
    
    xb(:, i+1) = ODE_solver(@uav.f_aug, dt, [x; xh], t(i), 'RK4');
    % xb(:, i+1) = xb(:, i) + fun(t(i), xb(:, i))*dt;

    %% assign real fault signal
    xb = sys_a.set_real_signal(xb, i);
    xb = sys_s.set_real_signal(xb, i);
end

% uav.Save('tr');

%% some mapping
uav.tr.x = xb(1:DIM_X3, :);
uav.tr.xh = xb(DIM_X3 + (1:DIM_X3), :);
uav.sys_a = sys_a;
uav.sys_s = sys_s;
end
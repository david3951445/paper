function uav = trajectory(uav)
%calculate trajectory

dt          = uav.tr.dt;
t           = uav.tr.t;
LEN         = uav.tr.LEN;
DIM_F       = uav.DIM_F;
DIM_X       = uav.sys.DIM_X;
DIM_X3      = uav.sys_aug.DIM_X;
startTime   = 5; % For calculate ddr(t), start at 3-rd step (k = 3)

%% set disturbance
%-1 actuator disturbance
d1 = repmat(100*sin(3*t), uav.sys_a.DIM, 1);
uav.sys_a.fault = zeros(uav.sys_a.DIM, LEN);

%-2 sensor fault
% square wave
% uav.tr.f2    = 0.1*ones(uav.sys_s.DIM, uav.tr.LEN);
% b = [0.1 -0.05 0.05]; n = length(b)+1;
% a = round(linspace(1,uav.tr.LEN,n));
% for i = 1 : n-1
%     uav.sys_s.fault(:, a(i):a(i+1)) = b(i);
% end

% smoothed square wave
t_ = linspace(0, uav.tr.T, 50);
x_ = 1*square(.5*t_, 60);
fx = fit(t_', x_', 'SmoothingSpline');
x3 = feval(fx, uav.tr.t)';
uav.sys_s.fault = repmat(x3, uav.sys_s.DIM, 1);

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
uav.tr.F = zeros(1, LEN);
for i = startTime : LEN - 1
    %% reference
    [phi, theta, F] = uav.pos_controller(xb(1 : DIM_X3, i), r4(1:3, i-2:i), dt);
    r = [
        r4(1, i)
        r4(2, i)
        r4(3, i)
        phi
        theta
        r4(4, i)
    ];
    % Calculate at every time step. More practical but it can't cancel the outliers
    dr = [r r_old(:, 1)]*[1 -1]'/dt; % finite different, precision: o(h)
    ddr = [r r_old(:, 1:2)]*[1 -2 1]'/dt^2;
    r_old = [r r_old(:, 1 : size(r_old, 2)-1)]; % update old r
    uav.tr.r{1}(:, i+1) = r;
    uav.tr.r{2}(:, i+1) = dr;
    uav.tr.r{3}(:, i+1) = ddr;

    %% debug message
    if mod(i, 1/dt) == 0
        disp(['t = ' num2str(i*dt)])
    end
    if isnan(xb(:, i))
        disp(['traj has NaN, t = ' num2str(i*dt)])
        break
    end
    if norm(xb(:, i)) == Inf
        disp(['traj has Inf, t = ' num2str(i*dt)])
        break
    end
    
    [uav, xb] = uav.CalculateNextState(xb, d1, r, dr, ddr, i);
    uav.tr.F(i) = F;
end

% uav.Save('tr');

%% some mapping
uav.tr.x = xb(1:DIM_X3, :);
uav.tr.xh = xb(DIM_X3 + (1:DIM_X3), :);
end
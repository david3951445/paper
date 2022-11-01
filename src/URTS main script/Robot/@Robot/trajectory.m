function rb = trajectory(rb)
%calculate trajectory

dt          = rb.tr.dt;
t           = rb.tr.t;
LEN         = rb.tr.LEN;
DIM_F       = rb.DIM_F;
DIM_X       = rb.sys.DIM_X;
DIM_X3      = rb.sys_aug.DIM_X;
startTime   = 3; % For calculate ddr(t), start at 3-rd step (k = 3)

%% set disturbance
%-1 actuator fault
d1 = 1*repmat(10*sin(3*t), rb.sys_a.DIM, 1);
rb.sys_a.fault = zeros(rb.sys_a.DIM, LEN);

%-2 sensor fault
% square wave
% rb.sys_s.fault = 0.1*ones(rb.sys_s.DIM, rb.tr.LEN);
% b = [0.1 -0.05 0.05]; n = length(b)+1;
% a = round(linspace(1,rb.tr.LEN,n));
% for i = 1 : n-1
%     rb.sys_s.fault(:, a(i):a(i+1)) = b(i);
% end

% smoothed square wave
t_ = linspace(0, rb.tr.T, 100);
x_ = .1*square(.5*t_, 60);
fx = fit(t_', x_', 'SmoothingSpline');
x3 = feval(fx, rb.tr.t)';
rb.sys_s.fault = repmat(x3, rb.sys_s.DIM, 1);

% sin wave
% x = 1*sin(1*rb.tr.t);
% x = x + sqrt(.01)*randn(1, rb.tr.LEN);
% rb.sys_s.fault    = repmat(x, rb.sys_s.DIM, 1);

% constant 
% rb.tr.f2    = 0.5*ones(rb.sys_s.DIM, rb.tr.LEN);

%% Construct r(t)
% Method 1
% To calculate dr, ddr, we need perious state
% r_old = zeros(DIM_F, 2); % store r[i-1], r[i-2]
% for i = 1 : startTime-1 % i = 2, we have dr. i = 3, we have ddr.
%     r = rb.qr(:, i);
%     r_old = [r r_old(:, 1 : size(r_old, 2)-1)]; % update old r
% end
% Method 2
rb.tr.r = cell(1,3);
for i = 1 : DIM_F
    r = rb.qr(i,1:LEN);
    % r = smoothdata(r, "movmean", 10);
    
    dr = diff(r, 1, 2)/dt;
    dr = filloutliers(dr,'clip','movmedian', 10, 'ThresholdFactor', 1); % numercial differenciation will produce outliers 
    dr = smoothdata(dr);
    
    ddr = diff(dr, 1, 2)/dt;
    ddr = smoothdata(ddr);
    
    dr = [zeros(1, 1) dr]; % diff() will result in 1 missing data, resize it
    ddr = [zeros(1, 2) ddr]; % diff() will result in 1 missing data, resize it

    rb.tr.r{1}(i,:) = r;
    rb.tr.r{2}(i,:) = dr;
    rb.tr.r{3}(i,:) = ddr;
end

%% initialize x, xh, u
x           = zeros(DIM_X3, LEN);
xh          = zeros(DIM_X3, LEN);
x(:, 1:startTime) = repmat(rb.tr.x0, [1 startTime]);
rb.tr.u     = zeros(rb.sys.DIM_U, LEN);

%% trajectory
disp(['Calculating trajectory ..., t = 0 ~ ' num2str(dt*(LEN-1))])
xb = [x; xh];
for i = startTime : LEN - 1
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
        % rb.tr.LEN = i;
        break
    end
    
    %% reference
    % Method 1, calculate at every time step. More practical but it can't cancel the outliers
    % In practice, dr and ddr are obtained from numercial differentiation
    % r = rb.qr(:, i);
    % dr = [r r_old(:, 1)]*[1 -1]'/dt; % finite different, precision: o(h)
    % ddr = [r r_old(:, 1:2)]*[1 -2 1]'/dt^2;
    % r_old = [r r_old(:, 1 : size(r_old, 2)-1)]; % update old r
    % Method 2, one time calculate
    r = rb.tr.r{1}(:, i);
    dr = rb.tr.r{2}(:, i);
    ddr = rb.tr.r{3}(:, i);
    
    [rb, xb] = rb.CalculateNextState(xb, d1, r, dr, ddr, i);
end

%% some mapping
rb.tr.x = xb(1:DIM_X3, :);
rb.tr.xh = xb(DIM_X3 + (1:DIM_X3), :);
end 
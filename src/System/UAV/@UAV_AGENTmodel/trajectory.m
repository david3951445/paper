function uav = trajectory(uav)
%calculate trajectory

sys         = uav.sys;
sys_a       = uav.sys_a;
sys_s       = uav.sys_s;
dt          = uav.tr.dt;
t           = uav.tr.t;
LEN         = uav.tr.LEN;
DIM_F       = uav.DIM_F;
DIM_X       = uav.sys.DIM_X;
DIM_X3      = uav.sys_aug.DIM_X;
startTime   = 5; % For calculate ddr(t), start at 3-rd step (k = 3)

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
    
    f = -eye(DIM_F)/M*((M-Mh)*(ddr + u) + H-Hh + uav.tr.f1(:, i));
    % f = -eye(DIM_F)/M*((M-Mh)*(ddr + u) + uav.tr.f1(:, i));
    % f = -eye(DIM_F)/M*(uav.tr.f1(:, i));

    % Show norm of error terms
    if mod(i, 100) == 0 
        disp(['norm of M-Mh: ' num2str(norm(M-Mh))])
        disp(['norm of H-Hh: ' num2str(norm(H-Hh))])
    end
    uav.tr.f1(:, i) = f;
    
    xb(:, i+1) = ODE_solver(@uav.f_aug, dt, [x; xh], t(i), 'RK4');
    % xb(:, i+1) = xb(:, i) + fun(t(i), xb(:, i))*dt;

    %% assign real fault signal
    % Since the real fault signal is not produced from smooth model, we need reassign it.
    % actuator fault
    j0 = sys.DIM_X;
    range = 1 : sys_a.DIM;
    for j = 1 : sys_a.WINDOW-1
        j1 = j0 + j*sys_a.DIM;
        xb(j1+range, i+1) = xb(j1+range-sys_a.DIM, i); % Since Fa(i) = [fa(i), fa(i-1), fa(i-2), ...], so Fa(i+1) = ["new fa", fa(i), f(i-1)]
    end
    xb(j0+range, i+1) = uav.tr.f1(:, i); % assign "new fa"
    % sensor fault
    if ~isempty(sys_s)
        j0 = sys.DIM_X + sys_a.DIM_X;
        range = 1 : sys_s.DIM;
        for j = 1 : sys_s.WINDOW-1
            j1 = j0 + j*sys_s.DIM;
            xb(j1+range, i+1) = xb(j1-sys_s.DIM+range, i);
        end
        xb(j0+range, i+1) = uav.tr.f2(:, i);
    end
end

uav.tr.x = xb(1:DIM_X3, :);
uav.tr.xh = xb(DIM_X3 + (1:DIM_X3), :);
end 
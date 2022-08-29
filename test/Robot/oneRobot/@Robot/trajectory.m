function rb = trajectory(rb)
%calculate trajectory

sys         = rb.sys;
sys_a       = rb.sys_a;
sys_s       = rb.sys_s;
dt          = rb.tr.dt;
t           = rb.tr.t;
LEN         = rb.tr.LEN;
DIM_F       = rb.DIM_F;
DIM_X       = rb.sys.DIM_X;
DIM_X3      = rb.sys_aug.DIM_X;
startTime   = 3; % For calculate ddr(t), start at 3-rd step (k = 3)

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
        % rb.tr.LEN = i;
        break
    end
    if norm(xb(:, i)) == Inf
        disp(['traj has Inf, t = ' num2str(i*dt)])
        % rb.tr.LEN = i;
        break
    end
    
    %% reference
    % Method 1, calculate at every time step. More practical, more practical but it can't cancel the outliers
    % In practice, dr and ddr are obtained from numercial differentiation
    % r = rb.qr(:, i);
    % dr = [r r_old(:, 1)]*[1 -1]'/dt; % finite different, precision: o(h)
    % ddr = [r r_old(:, 1:2)]*[1 -2 1]'/dt^2;
    % r_old = [r r_old(:, 1 : size(r_old, 2)-1)]; % update old r
    % Method 2, one time calculate
    r = rb.tr.r{1}(:, i);
    dr = rb.tr.r{2}(:, i);
    ddr = rb.tr.r{3}(:, i);

    %% extract x, xh, X, dX form xb
    x       = xb(1 : DIM_X3, i);
    xh      = xb(DIM_X3 + (1:DIM_X3), i);
    X       = x(DIM_F + (1:DIM_F)); % position
    dX      = x(2*DIM_F + (1:DIM_F)); % velocity
    Xh      = xh(DIM_F + (1:DIM_F));
    dXh     = xh(2*DIM_F + (1:DIM_F));

    %% fault signal
    % feedback linearized term: Mh(), Hh(). By testing, using feedforward only ( Mh(r(t)) ) is more stable then feedback + feedforward ( Mh(xh(t)+r(t)) ).
    u = rb.u_PID(xh); % PID control
    rb.tr.u(:, i+1) = u;
    M = rb.M(X+r); 
    Mh = rb.M(r); % feedforward compensation
    % Mh = rb.M(Xh+r); 
    % Mh = 0;
    
    H = rb.H(X+r, dX+dr);
    Hh = rb.H(r, dr); % feedforward compensation
    % Hh = rb.H(Xh+r, dXh+dr); % feedback compensation
    % Hh = 0;

    % fext1 = externalForce(rb.rbtree, 'body11', [0 0 0 0 0 rb.tr.GRF{1}(i)], X');
    % tau = inverseDynamics(rb.rbtree, X', [], [], fext1);

    f = -eye(DIM_F)/M*((M-Mh)*(ddr + u) + H-Hh + rb.tr.f1(:, i));
    % f = -eye(DIM_F)/M*((M-Mh)*(ddr + u) + rb.tr.f1(:, i));
    % f = -eye(DIM_F)/M*(rb.tr.f1(:, i));
    % f = tau;

    % Show norm of error terms
    if mod(i, 100) == 0 
        % disp(['norm of M-Mh: ' num2str(norm(M-Mh))])
        % disp(['norm of H-Hh: ' num2str(norm(H-Hh))])
    end
    rb.tr.f1(:, i) = f;
    
    xb(:, i+1) = ODE_solver(@rb.f_aug, dt, [x; xh], t(i), 'RK4');
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
    xb(j0+range, i+1) = rb.tr.f1(:, i); % assign "new fa"
    % sensor fault
    if ~isempty(sys_s)
        j0 = sys.DIM_X + sys_a.DIM_X;
        range = 1 : sys_s.DIM;
        for j = 1 : sys_s.WINDOW-1
            j1 = j0 + j*sys_s.DIM;
            xb(j1+range, i+1) = xb(j1-sys_s.DIM+range, i);
        end
        xb(j0+range, i+1) = rb.tr.f2(:, i);
    end
end

rb.tr.x = xb(1:DIM_X3, :);
rb.tr.xh = xb(DIM_X3 + (1:DIM_X3), :);
end 
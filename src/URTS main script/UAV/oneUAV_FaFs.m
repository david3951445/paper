%main script
% one uav, observer-based tracking control, feedfoward linearization, FTC (smoothed model, actuator and sensor fault), K solved by DIM = 1 method
clc; clear; close all; tic;
addpath(genpath('../../../src'))
addpath(genpath('function'))

uav = UAV_AGENTmodel();
% flow control of code
uav.EXE_LMI     = 0; % solving LMI
uav.EXE_TRAJ    = 0; % trajectory
uav.EXE_PLOT    = 1; % plot results

% time
uav.tr.dt    = .001; % Time step
uav.tr.T     = 20; % Final time

% global variable
DIM_F       = uav.DIM_F; % Dimension of e
DIM_SOLVE_K = 1;
I           = eye(DIM_F);

%% system
% Error weighting. Tracking:1, Estimation:2
A0          = [0 1 0; 0 0 1; 0 0 0];
B0          = [0; 0; 1];
C0          = [1 0 0; 0 1 0; 0 0 1]; % Intergral{e}, e, de

% for solving control and observer gain
sys1        = LinearModel(A0, B0, C0);
sys1.Q1     = 10^(2)*diag([1 100 10]);
sys1.Q2     = 10^(2)*diag([1 100 10]); % corresponding to [Intergral{e}, e, de]

% for plot trajectories
sys = LinearModel(kron(A0, I), kron(B0, I), kron(C0, I));

%% smooth model (acuator)
WINDOW  = 3;
dt_     = 1*uav.tr.dt;
METHOD  = '2';

% for solving control and observer gain
sys_a1      = SmoothModel(WINDOW, DIM_SOLVE_K, dt_, METHOD);
sys_a1.B    = sys1.B;
sys_a1.Q1   = diag(zeros(1, WINDOW)); % Can't stablilze unknown signal
sys_a1.Q2   = 10^(2)*diag((.1.^(0 : WINDOW-1)));

% for plot trajectories
sys_a       = SmoothModel(WINDOW, DIM_F, dt_, METHOD);
sys_a.B     = kron(sys_a1.B, I);

%% smooth model (sensor)
WINDOW  = 4;
dt_     = 1000*uav.tr.dt; % multiply 1000 is better by testing
METHOD = '2';

% for solving control and observer gain
sys_s1      = SmoothModel(WINDOW, DIM_SOLVE_K, dt_, METHOD);
sys_s1.B    = [0; .1; 1];
sys_s1.Q1   = diag(zeros(1, WINDOW));  % Can't stablilze unknown signal
sys_s1.Q2   = 10^(2)*diag((.1.^(0 : WINDOW-1)));

% for plot trajectories
sys_s       = SmoothModel(WINDOW, DIM_F, dt_, METHOD);
sys_s.B     = kron(sys_s1.B, I);

%% augment system
% for solving control and observer gain
[A, B, C]       = AugmentSystem(sys1.A, sys1.B, sys1.C, sys_a1.A, sys_a1.B, sys_a1.C, sys_s1.A, sys_s1.B, sys_s1.C);
sys_aug1        = LinearModel(A, B, C);
sys_aug1.Q1     = 10^(0)/2*blkdiag(sys1.Q1, sys_a1.Q1, sys_s1.Q1); % weight of integral{e}, e, de, f1, f2
sys_aug1.Q2     = 10^(-1)/2*blkdiag(sys1.Q2, sys_a1.Q2, sys_s1.Q2); % weight of integral{e}, e, de, f1, f2
sys_aug1.E      = 1*eye(sys_aug1.DIM_X);
sys_aug1.R      = 2*10^(-3)*eye(sys_aug1.DIM_U);
sys_aug1.rho    = 30;

% for plot trajectories
[A, B, C]   = AugmentSystem(sys.A, sys.B, sys.C, sys_a.A, sys_a.B, sys_a.C, sys_s.A, sys_s.B, sys_s.C);
sys_aug     = LinearModel(A, B, C);

%% some mapping
sys_a.begin     = sys.DIM_X;
sys_s.begin     = sys.DIM_X + sys_a.DIM_X;
uav.sys         = sys;
uav.sys_a       = sys_a;
uav.sys_s       = sys_s;
uav.sys_aug     = sys_aug;

%% solve LMI
uav = uav.get_K_L(sys_aug1);

%% trajectory
% Construct reference r(t) = [xd, yd, zd, phid]^T
uav.tr.t    = 0 : uav.tr.dt : uav.tr.T;
uav.tr.LEN  = length(uav.tr.t);

%-1 circle up
% uav.qr  = zeros(4, uav.tr.LEN);
% amp_z   = 0.9;
% amp     = 0.8;
% freg    = 1;
% for i = 1 : uav.tr.LEN - 1
%     uav.qr(:, i) = [
%         amp*sin(freg*uav.tr.t(i))
%         amp*cos(freg*uav.tr.t(i))
%         amp_z*uav.tr.t(i) + 1
%         0
%     ];
% end

%-2 search task
r1 = [
    0 0 1 1 2 2 2 1 0
    0 1 1 0 0 1 2 2 2
]*5;
a = 20;

len1 = length(r1);
t1 = linspace(0,1,len1);

len2 = (len1-1)*a+1;
t2 = linspace(0,1,len2);
r2 = zeros(2, len2);
r2 = [
    interp1(t1, r1(1, :), t2);
    interp1(t1, r1(2, :), t2);
];

fx = fit(t2', r2(1,:)', 'SmoothingSpline');
fy = fit(t2', r2(2,:)', 'SmoothingSpline');
len3 = uav.tr.LEN;
t3 = linspace(0,1,len3);
r3 = [
    feval(fx, t3)';
    feval(fy, t3)';
];
r4 = cat(1, r3, 1*sin(uav.tr.t));
uav.qr = cat(1, r4, zeros(1, length(r4)));

if uav.EXE_TRAJ
    %% set initial
    x0_pos = zeros(1, sys.DIM_X);
    uav.tr.x0    = [x0_pos zeros(1, sys_a.DIM_X) zeros(1, sys_s.DIM_X)]';
    uav.tr.xh0   = zeros(uav.sys_aug.DIM_X, 1);

    uav = uav.trajectory();
    uav.Save('tr');
end

if uav.EXE_PLOT
    disp('Ploting trajectory ...')
    %% r(t)
    figure; hold on;    
    plot(r2(1, :), r2(2, :));
    plot(uav.qr(1, :), uav.qr(2, :), '-o', 'DisplayName', 'r(t)')

    %% state, error, estimated state
    fig = figure;
    DIM = DIM_F;
    div = divisors(DIM);
    i = ceil((length(div))/2);
    Layout = tiledlayout(DIM/div(i), div(i));
    
    r = uav.tr.r{1};
    % timeInterval = 1:length(uav.tr.t)-1;
    for i = 1 : DIM_F % position
        nexttile
        hold on
        index = DIM_F + i;
        plot(uav.tr.t, uav.tr.x(index, :)+r(i, :), 'DisplayName', 'state', 'LineWidth', 2)
        plot(uav.tr.t, uav.tr.xh(index, :)+r(i, :), 'DisplayName', 'estimated', 'LineWidth', 2)
        plot(uav.tr.t, r(i, :), 'DisplayName', 'reference', 'LineWidth', 2)
        
        title(['$q_' num2str(i) '$'], 'Interpreter','latex')
        legend
        xlabel("t")
        % ylim([-2 2])
    end
    
    FILE_NAME = ['results/fig' num2str(fig.Number) '.pdf'];
    saveas(fig, FILE_NAME)
    FILE_NAME = ['data/fig/fig' num2str(fig.Number) '.fig'];
    savefig(FILE_NAME)
    
    %% fault signals (Fa, Fs)
    Plot(uav.tr.t, uav.tr.x, uav.tr.xh, uav.sys_a.begin, sys_a.DIM, 'a')
    % Plot(uav.tr.t, uav.tr.x, uav.tr.xh, index, 1, 'a')
    Plot(uav.tr.t, uav.tr.x, uav.tr.xh, uav.sys_s.begin , sys_s.DIM, 's')
    % Plot(uav.tr.t, uav.tr.x, uav.tr.xh, index, 1, 's')
    
    %% control u(t)
    fig = figure;
    DIM = size(uav.tr.u, 1);
    div = divisors(DIM);
    i = ceil((length(div))/2);
    Layout = tiledlayout(DIM/div(i), div(i));
    
    % timeInterval = 1:length(uav.tr.t)-1;
    for i = 1 : DIM % position
        nexttile
        hold on
        index = i;
        plot(uav.tr.t, uav.tr.u(index, :), 'DisplayName', 'control', 'LineWidth', 2)    
        title(['$u_' num2str(i) '$'], 'Interpreter','latex')
        legend
        xlabel("t")
        % ylim([-2 2])
    end
    
%     FILE_NAME = ['results/fig' num2str(fig.Number) '.pdf'];
%     saveas(fig, FILE_NAME)
%     FILE_NAME = ['data/fig/fig' num2str(fig.Number) '.fig'];
%     savefig(FILE_NAME)
end

%% Execution time
toc
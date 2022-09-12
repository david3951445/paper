%main script
% one Robot, observer-based tracking control, feedfoward linearization, FTC (smoothed model, actuator and sensor fault), K solved by DIM = 1 method, path planning
clc; clear; close all; tic;
addpath(genpath('../../../src'))
addpath(genpath('function'))

rb = Robot();
% flow control of code
rb.EXE_LMI     = 0;
rb.EXE_Z2C     = 0; % ZMP to CoM converter
rb.EXE_IK      = 0; % inverse dynamic
rb.EXE_TRAJ    = 0; % trajectory
rb.EXE_PLOT    = 0; % plot results

% time
rb.tr.dt    = .001; % Time step
rb.tr.T     = 9; % Final time

% global variable
DIM_F       = rb.DIM_F; % Dimension of e
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
dt_     = 1*rb.tr.dt;
METHOD  = '2';

% for solving control and observer gain
sys_a1      = SmoothModel(WINDOW, DIM_SOLVE_K, dt_, METHOD);
sys_a1.B    = sys1.B;
sys_a1.Q1   = diag(zeros(1,WINDOW)); % Can't stablilze unknown signal
sys_a1.Q2   = 10^(2)*diag((.1.^(0 : WINDOW-1)));

% for plot trajectories
sys_a       = SmoothModel(WINDOW, DIM_F, dt_, METHOD);
sys_a.B     = kron(sys_a1.B, I);

%% smooth model (sensor)
WINDOW  = 4;
dt_     = 1000*rb.tr.dt; % multiply 1000 is better by testing
METHOD = '2';

% for solving control and observer gain
sys_s1      = SmoothModel(WINDOW, DIM_SOLVE_K, dt_, METHOD);
sys_s1.B    = [0; .1; 1];
sys_s1.Q1   = diag(zeros(1,WINDOW));  % Can't stablilze unknown signal
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
sys_a.begin = sys.DIM_X;
sys_s.begin = sys.DIM_X + sys_a.DIM_X;
rb.sys      = sys;
rb.sys_a    = sys_a;
rb.sys_s    = sys_s;
rb.sys_aug = sys_aug;

%% solve LMI
rb = rb.get_K_L(sys_aug1);

%% trajectory
% Find task space ref
pp = PathPlanning();

% find joint ref
rb = rb.Ref2Config(pp.r); % rb.r -> rb.qr

% testing ref
% t = 0 : dt : 10;
% ref = zeros(DIM_F, length(t));
% ref = repmat(.1*cos(t), DIM_F, 1);
% ref(1:DIM_F/2, :) = repmat(.1*cos(t), DIM_F/2, 1);
% ref(DIM_F/2+1:DIM_F, :) = repmat(.1*sin(t), DIM_F/2, 1);
% rb.qr = ref;

if rb.EXE_TRAJ
    rb.tr.t    = 0 : rb.tr.dt : rb.tr.T;
    rb.tr.LEN  = length(rb.tr.t);

    %% set initial
    % x0_pos = [0.2 0.2 0 0.1 0.1 0.5 0.2 0.2 0 0.1 0.1 0.5];
    % x0_pos = .1*[0.2 0.2 0 0.1 0.1 0.5 0.2 0.2 0 0.1 0.1 0.5];
    x0_pos      = zeros(1, sys.DIM_X);
%     x0_pos = [zeros(1,DIM_F) x0_pos zeros(DIM_F)]
    rb.tr.x0    = [x0_pos zeros(1, sys_a.DIM_X) 0*ones(1, sys_s.DIM_X)]';
    rb.tr.xh0   = zeros(rb.sys_aug.DIM_X, 1);

    rb = rb.trajectory();
    rb.Save('tr');
end

if rb.EXE_PLOT
    disp('Ploting trajectory ...')
    %% GRF
    
    %% map and path
    fig = figure;
    show(pp.map);
    hold on;
    plot(pp.tree(:,1), pp.tree(:,2),'.-', 'DisplayName','tree expansion'); % tree expansion
    % plot(pthObj.States(:,1), pthObj.States(:,2),'r-','LineWidth',2, 'DisplayName','path') % draw path  
    plot(pp.r(1, :), pp.r(2, :), '-o', 'DisplayName', 'r(t)')
    
    len2 = length(rb.r);
    plot(rb.r_lr(1, 1:2:len2), rb.r_lr(2, 1:2:len2), '-o', 'DisplayName', 'left foot')
    plot(rb.r_lr(1, 2:2:len2), rb.r_lr(2, 2:2:len2), '-o', 'DisplayName', 'right foot')
    plot(rb.zmp(1, :), rb.zmp(2, :), '-s', 'Displayname', 'ZMP trajectory')
    plot(rb.CoM(1, :), rb.CoM(2, :), 'Displayname', 'CoM trajectory')
    axis equal
    title('foot trajectory')
    xlabel('x'); ylabel('y')
    legend
%     
%     FILE_NAME = ['results/fig' num2str(fig.Number) '.pdf'];
%     saveas(fig, FILE_NAME)
%     FILE_NAME = ['data/fig/fig' num2str(fig.Number) '.fig'];
%     savefig(FILE_NAME)
    
    %% state, estimated state, reference
    fig = figure;
    DIM = 1;
    div = divisors(DIM);
    i = ceil((length(div))/2);
    Layout = tiledlayout(DIM, div(i));
    
    r = rb.tr.r{1};
    % timeInterval = 1:length(rb.tr.t)-1;
    for i = 1 : 1%DIM_F % position
        nexttile
        hold on
        index = DIM_F + i;
        plot(rb.tr.t, rb.tr.x(index, :)+r(i, :), 'DisplayName', 'state', 'LineWidth', 2)
        plot(rb.tr.t, rb.tr.xh(index, :)+r(i, :), 'DisplayName', 'estimated', 'LineWidth', 2)
        plot(rb.tr.t, r(i, :), 'DisplayName', 'reference', 'LineWidth', 2)
        
        title(['$q_' num2str(i) '$'], 'Interpreter','latex')
        legend
        xlabel("t")
        % ylim([-2 2])
    end
    
    FILE_NAME = ['results/fig' num2str(fig.Number) '.pdf'];
    saveas(fig, FILE_NAME)
    FILE_NAME = ['data/fig/fig' num2str(fig.Number) '.fig'];
    savefig(FILE_NAME)
    
    %% Fa and Fs
    Plot(rb.tr.t, rb.tr.x, rb.tr.xh, rb.sys_a.begin, sys_a.DIM, 'a')
    % Plot(rb.tr.t, rb.tr.x, rb.tr.xh, index, 1, 'a')
    Plot(rb.tr.t, rb.tr.x, rb.tr.xh, rb.sys_s.begin, sys_s.DIM, 's')
    % Plot(rb.tr.t, rb.tr.x, rb.tr.xh, index, 1, 's')

    %% control u(t)
    fig = figure;
    DIM = size(rb.tr.u, 1);
    div = divisors(DIM);
    i = ceil((length(div))/2);
    Layout = tiledlayout(DIM/div(i), div(i));
    
    % timeInterval = 1:length(rb.tr.t)-1;
    for i = 1 : DIM % position
        nexttile
        hold on
        index = i;
        plot(rb.tr.t, rb.tr.u(index, :), 'DisplayName', 'control', 'LineWidth', 2)    
        title(['$u_{' num2str(i) '}$'], 'Interpreter','latex')
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

%% Controlability
% rank(ctrb(rb.A, rb.B))    
%main script
% one Robot, observer-based tracking control, feedfoward linearization, FTC (smoothed model, actuator and sensor fault), K solved by DIM = 1 method, path planning
clc; clear; close all; tic;
addpath(genpath('../../../src'))
addpath(genpath('function'))

rb = Robot();
% flow control of code
rb.EXE_LMI     = 1;
rb.EXE_Z2C     = 0; % ZMP to CoM converter
rb.EXE_IK      = 0; % inverse dynamic
rb.EXE_TRAJ    = 1; % trajectory
rb.EXE_PLOT    = 1; % plot results

% time
rb.tr.dt    = .001; % Time step
rb.tr.T     = 2; % Final time

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
sys_a1.Q2   = 10^(1)*diag((.1.^(0 : WINDOW-1)));

% for plot trajectories
sys_a       = SmoothModel(WINDOW, DIM_F, dt_, METHOD);
sys_a.B     = kron(sys_a1.B, I);

%% smooth model (sensor)
WINDOW  = 5;
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
sys_aug1.rho    = 25;

% for plot trajectories
[A, B, C]   = AugmentSystem(sys.A, sys.B, sys.C, sys_a.A, sys_a.B, sys_a.C, sys_s.A, sys_s.B, sys_s.C);
sys_aug     = LinearModel(A, B, C);

%% some mapping
rb.sys      = sys;
rb.sys_a    = sys_a;
rb.sys_s    = sys_s;
rb.sys_aug = sys_aug;

%% solve LMI
if rb.EXE_LMI
    disp('solving LMI ...')
    [K, KL] = solveLMI10(sys_aug1.A, sys_aug1.B, sys_aug1.C, sys_aug1.E, sys_aug1.Q1, sys_aug1.Q2, sys_aug1.R, sys_aug1.rho);
    rb.K = K;
    rb.KL = KL;
    
%     norm(K)
%     norm(KL)
    rb.Save('K') 
    rb.Save('KL') 
end
% Fine tune of gain
% gain = [-1 zeros(1, sys_a.WINDOW-1)];
% rb.K(:, sys1.DIM_X + (1:sys_a.WINDOW)) = gain;
% rb.KL(4:6, :) = [1000 0 0; 0 100 0; 0 0 10];
% rb.KL(7:9,:) = [1000 0 0; 0 100 0; 0 0 10];
% rb.K(:, sys1.DIM_X + sys_a.WINDOW + (1:sys_s.WINDOW)) = zeros(1, sys_s.WINDOW);
% I0 = diag(1.1.^(0:rb.DIM_F-1));
% ...

% construt origin gain
rb.K = kron(rb.K, I);
rb.KL = kron(rb.KL, I);

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
    rb.tr.x0    = [x0_pos ones(1, sys_a.DIM_X) 0*ones(1, sys_s.DIM_X)]';
    rb.tr.xh0   = zeros(rb.sys_aug.DIM_X, 1);

    %% set disturbance
    x = .2*cos(1*rb.tr.t);
    % x = x + sqrt(.1)*randn(1, rb.tr.LEN);
    rb.tr.f1    = repmat(x, sys_a.DIM, 1);

    % sin wave
    x = 1*sin(1*rb.tr.t);
    % x = x + sqrt(.01)*randn(1, rb.tr.LEN);
    rb.tr.f2    = repmat(x, sys_s.DIM, 1);

    % smoothed square wave
    % t = linspace(0, rb.tr.T, 100);
    % x = .5*square(t, 60);
    % fx = fit(t', x', 'SmoothingSpline');
    % x3 = feval(fx, rb.tr.t)';
    % plot(rb.tr.t, x3)
    % rb.tr.f2 = repmat(x3, sys_a.DIM, 1);
    
    % constant 
    % rb.tr.f2    = 0.5*ones(sys_s.DIM, rb.tr.LEN);

    % square wave
    % b = [.5 -.5 .5 -.5]; n = length(b)+1;
    % a = round(linspace(1,rb.tr.LEN,n));
    % for i = 1 : n-1
    %     rb.tr.f2(:, a(i):a(i+1)) = b(i);
    % end

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
    start_index = sys.DIM_X;
    Plot(rb.tr.t, rb.tr.x, rb.tr.xh, start_index, sys_a.DIM, 'a')
    % Plot(rb.tr.t, rb.tr.x, rb.tr.xh, index, 1, 'a')
    start_index = sys.DIM_X + sys_a.DIM_X;
    Plot(rb.tr.t, rb.tr.x, rb.tr.xh, start_index, sys_s.DIM, 's')
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
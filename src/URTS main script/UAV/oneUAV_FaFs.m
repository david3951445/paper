%main script
% one uav, observer-based tracking control, feedfoward linearization, FTC (smoothed model, actuator and sensor fault), K solved by DIM = 1 method
clc; clear; close all; tic;
addpath(genpath('../../../src'))

uav = UAV_AGENTmodel();
% flow control of code
uav.EXE_LMI     = 0; % solving LMI
uav.EXE_TRAJ    = 0; % trajectory
uav.EXE_PLOT    = 1; % plot results

% time
uav.tr.dt   = .001; % Time step
uav.tr.T    = 40; % Final time
uav.tr.t    = 0 : uav.tr.dt : uav.tr.T;
uav.tr.LEN  = length(uav.tr.t);

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
dt_     = 600*uav.tr.dt; % multiply 1000 is better by testing
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
sys_aug1.R      = 2*10^(-2)*eye(sys_aug1.DIM_U);
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
uav = uav.get_K_L(sys1, sys_aug1);

%% trajectory
uav = uav.SetPath();

if uav.EXE_TRAJ
    %% set initial
    % x0_pos = zeros(1, sys.DIM_X);
    x0_pos = .1.*rand(1, sys.DIM_X) + 0;
    uav.tr.x0    = [x0_pos zeros(1, sys_a.DIM_X) zeros(1, sys_s.DIM_X)]';
    uav.tr.xh0   = zeros(uav.sys_aug.DIM_X, 1);

    uav = uav.trajectory();
    uav.Save('tr');
end

if uav.EXE_PLOT
    disp('Ploting trajectory ...')

    r = uav.tr.r{1};
    
    %% Tracking control results
%     uav.PlotTC(); 

    %% 3D, r(t), state
    % figure

    % plot3(r(1, :), r(2, :), r(3, :), DisplayName='reference');
    % hold on

    % X = uav.tr.x(DIM_F+1, :) + r(1, :);
    % Y = uav.tr.x(DIM_F+2, :) + r(2, :);
    % Z = uav.tr.x(DIM_F+3, :) + r(3, :);
    % plot3(X, Y, Z, DisplayName='state')

    % grid on
    % xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)')
    % legend(Interpreter="latex", Location='best')

    %% local motion planning
    fig = figure;
    
    plot3(uav.sigma(1, :), uav.sigma(2, :), uav.sigma(3, :), '^', DisplayName='$\sigma(t)$', MarkerSize=7)
    hold on

    plot3(r(1, :), r(2, :), r(3, :), DisplayName='$\sigma''(t)$');

    grid on
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)')
    legend(Interpreter="latex", Location='best')

    SaveFig(fig)

    %% state, error, estimated state   
    % Y_LABEL = {'x (m)', 'y (m)', 'z (m)', '\phi (rad)', '\theta (rad)', '\psi (rad)'};
end

%% Execution time
toc
%main script
% one uav, observer-based tracking control, feedfoward linearization, FTC (smoothed model, actuator and sensor fault), K solved by DIM = 1 method
clc; clear; close all; tic;
addpath(genpath('../../../src'))

uav = UAV_AGENTmodel();
uav.PATH  = ['./data/' mfilename]; % path of saved data
uav = uav.Load(); % load old data

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
sys1.Q1     = 10^(3)*diag([1 100 10]);
sys1.Q2     = 10^(1)*diag([1 100 10]); % corresponding to [Intergral{e}, e, de]

% for plot trajectories
uav.sys = LinearModel(kron(A0, I), kron(B0, I), kron(C0, I));

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
uav.sys_a       = SmoothModel(WINDOW, DIM_F, dt_, METHOD);
uav.sys_a.B     = kron(sys_a1.B, I);

%% smooth model (sensor)
WINDOW  = 4;
dt_     = 800*uav.tr.dt; % multiply 1000 is better by testing
METHOD = '2';

% for solving control and observer gain
sys_s1      = SmoothModel(WINDOW, DIM_SOLVE_K, dt_, METHOD);
sys_s1.B    = [0; .1; 1];
sys_s1.Q1   = diag(zeros(1, WINDOW));  % Can't stablilze unknown signal
sys_s1.Q2   = 2*10^(3)*diag((.1.^(0 : WINDOW-1)));

% for plot trajectories
uav.sys_s       = SmoothModel(WINDOW, DIM_F, dt_, METHOD);
uav.sys_s.B     = kron(sys_s1.B, I);

%% augment system
% for solving control and observer gain
[A, B, C]       = AugmentSystem(sys1.A, sys1.B, sys1.C, sys_a1.A, sys_a1.B, sys_a1.C, sys_s1.A, sys_s1.B, sys_s1.C);
sys_aug1        = LinearModel(A, B, C);
sys_aug1.Q1     = 10^(-2)*blkdiag(sys1.Q1, sys_a1.Q1, sys_s1.Q1); % weight of integral{e}, e, de, f1, f2
sys_aug1.Q2     = 10^(-2)*blkdiag(sys1.Q2, sys_a1.Q2, sys_s1.Q2); % weight of integral{e}, e, de, f1, f2
sys_aug1.E      = 1*eye(sys_aug1.DIM_X);
sys_aug1.R      = 2*10^(-2)*eye(sys_aug1.DIM_U);
sys_aug1.rho    = 30;

% for plot trajectories
[A, B, C]   = AugmentSystem(uav.sys.A, uav.sys.B, uav.sys.C, uav.sys_a.A, uav.sys_a.B, uav.sys_a.C, uav.sys_s.A, uav.sys_s.B, uav.sys_s.C);
uav.sys_aug     = LinearModel(A, B, C);
uav.sys_aug.Q1  = kron(sys_aug1.Q1, I);
uav.sys_aug.Q2  = kron(sys_aug1.Q2, I);
uav.sys_aug.R   = kron(sys_aug1.R, I);

%% Copy sys, to uav
uav.sys_a.begin = uav.sys.DIM_X;
uav.sys_s.begin = uav.sys.DIM_X + uav.sys_a.DIM_X;


%% solve LMI
uav = uav.get_K_L(sys1, sys_aug1);

%% trajectory
uav = uav.SetPath();

if uav.EXE_TRAJ
    %% set initial
    % x0_pos = zeros(1, uav.sys.DIM_X);
    x0_pos = .1.*rand(1, uav.sys.DIM_X) + 0;
    uav.tr.x0    = [x0_pos zeros(1, uav.sys_a.DIM_X) zeros(1, uav.sys_s.DIM_X)]';
    uav.tr.xh0   = zeros(uav.sys_aug.DIM_X, 1);

    uav = uav.trajectory();
    uav.Save('sys_a')
    uav.Save('sys_s')
    uav.Save('tr');
end

%% Plot
if uav.EXE_PLOT
    disp('Ploting trajectory ...')
  
    %% Tracking control results
    % PlotLMP(uav)
    % uav.PlotTC(); 
    % PlotActualControl(uav) % Actual control input
    PlotStateAndRef3D(uav) % 3D, r(t), state

end

%% Calculate Hinf Performance
% uav.GetHinfPerformance()

%% Execution time
toc
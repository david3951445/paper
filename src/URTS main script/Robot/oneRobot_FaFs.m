%main script
% one Robot, observer-based tracking control, feedfoward linearization, FTC (smoothed model, actuator and sensor fault), K solved by DIM = 1 method, path planning
% The slow parts (During the test, these parts should be taken care not to run repeatedly):
%   - calculating the inverse dynamic (line 20 in IK_leg.m)
%   - calculating the nonlinear terms M(x) and H(x, dx) in rb.trajectory()
clc; clear; close all; tic;
addpath(genpath('../../../src'))

rb = Robot();
% flow control of code
rb.EXE_LMI     = 0;
rb.EXE_Z2C     = 0; % ZMP to CoM converter
rb.EXE_IK      = 0; % inverse dynamic
rb.EXE_TRAJ    = 0; % trajectory
rb.EXE_PLOT    = 1; % plot results

% time
rb.tr.dt    = .001; % Time step
rb.tr.T     = 20; % Final time
rb.tr.t    = 0 : rb.tr.dt : rb.tr.T;
rb.tr.LEN  = length(rb.tr.t);

% global variable
DIM_F       = rb.DIM_F; % Dimension of e
DIM_SOLVE_K = 1;
I           = eye(DIM_F);

%% Agent system model 
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

%% Smooth model (acuator)
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

%% Smooth model (sensor)
WINDOW  = 3;
dt_     = 1000*rb.tr.dt; % multiply 1000 is better by testing
METHOD = '2';

% for solving control and observer gain
sys_s1      = SmoothModel(WINDOW, DIM_SOLVE_K, dt_, METHOD);
sys_s1.B    = [0; 0; 1];
sys_s1.Q1   = diag(zeros(1,WINDOW));  % Can't stablilze unknown signal
sys_s1.Q2   = 10^(2)*diag((.1.^(0 : WINDOW-1)));

% for plot trajectories
sys_s       = SmoothModel(WINDOW, DIM_F, dt_, METHOD);
sys_s.B     = kron(sys_s1.B, I);

%% Augment system
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

%% Copy sys, sys_a, sys_s, sys_aug to rb
sys_a.begin = sys.DIM_X;
sys_s.begin = sys.DIM_X + sys_a.DIM_X;
rb.sys      = sys;
rb.sys_a    = sys_a;
rb.sys_s    = sys_s;
rb.sys_aug = sys_aug;

%% solve LMI
rb = rb.get_K_L(sys1, sys_aug1);

%% Calculate trajectory
pp = PathPlanning(); % Find task space ref in an occupancy  map using RRT
rb = rb.Ref2Config(pp.r); % task space ref -> joint space ref

if rb.EXE_TRAJ
    % Set initial
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
    Show.PlotPP(pp); % path planning
%     Show.PlotLMP(pp, rb); % local motion planning
%     Show.PlotTC(rb) % tracking control
end
    
%% Execution time
toc
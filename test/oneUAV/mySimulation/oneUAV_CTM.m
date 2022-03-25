%main script
% one UAV, fuzzy, reference model tracking control, no observer
clc; clear; close all; tic; % warning off
addpath(genpath('../../../src'))
addpath(genpath('function'))

%% M, C
uav = UAV_CTMmodel();
DIM_F = uav.DIM_X/2;
I = eye(DIM_F*3);
O = zeros(DIM_F);
uav.A = [O eye(DIM_F) O; O O eye(DIM_F); O O O];
uav.B = [O; O; eye(DIM_F)];
uav.C = I;
[DIM_X, DIM_U] = size(uav.B);
[DIM_Y, ~] = size(uav.C);

% A matrix for unknown signal
WINDOW = 2;
dt = 0.001;
Af = zeros(WINDOW);
point = zeros(WINDOW, 1); point(1:2) = [1 -1];
Af(1, 1:WINDOW) = point/dt;
for i = 2 : WINDOW
    point = [1 0];
    Af(i, i-1:i) = FindFDC(point, 1)'/dt; % obtain coefficient
end
Af = kron(Af, I);
Cf = zeros(1, WINDOW); Cf(1) = 1;
Cf = kron(Cf, I);
DIM_X2 = DIM_X*WINDOW;

% augment system
Ab = [uav.A Cf; zeros(DIM_X2, DIM_X) Af];
Bb = [uav.B; zeros(DIM_X2, DIM_U)];
Cb = [uav.C zeros(DIM_Y, DIM_X2)];
DIM_X3 = size(Ab, 1);

%% L, K
Qf = 10^(-3)*ones(1, WINDOW);
Q2 = diag([10^(-3) Qf]);
weight = diag([ones(1, 6) 0.1*ones(1, 6) 0.01*ones(1, 6)]); % weight of integral{e}, e, de
Q2 = kron(weight, Q2);
Q2 = 10*Q2;
Q1 = 0.01*Q2; 
rho = 100;

[K, L] = solveLMI9(Ab, Bb, Cb, Q1, Q2, rho);

%% trajectory
if EXE.TRAJ
    uav.tr.dt           = dt; % Time step
    uav.tr.T            = 10; % Final time
    uav.tr.x0           = [0.1 0 0.1 0.5 0.1 0.5 0.51 0.59 0.52 0.52 0.55 0.52]';
    uav.tr.xr0          = [0 1 0.5 0 0 0.8 0 0 0 0 0 0]';
    uav.tr.IS_LINEAR    = 0; % Run fuzzy linear system or origin nonlinear system
    uav.tr.IS_RK4       = 0; % Run RK4 or Euler method

    uav = uav.trajectory(fz);
    uav.Save('tr');
end

if EXE.PLOT
    uav.tr.plot();
end


%% Execution time
toc

%% Controlability

%% Debug

%% Remove path
rmpath(genpath('function'))
rmpath(genpath('../../../src'))

%% functions
function y = symmetric(x)
    y = x + x';
end
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
A = [O eye(DIM_F) O; O O eye(DIM_F); O O O];
B = [O; O; eye(DIM_F)];
C = I;
[DIM_X, DIM_U] = size(B);
[DIM_Y, ~] = size(C);

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
Af = kron(Af, eye(DIM_F));
Cf = zeros(3, WINDOW); Cf(:, 1) = 1;
Cf = kron(Cf, eye(DIM_F));
DIM_X2 = DIM_F*WINDOW;

% augment system
uav.A = [A Cf; zeros(DIM_X2, DIM_X) Af];
uav.B = [B; zeros(DIM_X2, DIM_U)];
uav.C = [C zeros(DIM_Y, DIM_X2)];
uav.DIM_X3 = size(uav.A, 1);
Eb = kron(diag([0 0 0 ones(1, WINDOW)]), eye(DIM_F)); % disturbance matrix

%% L, K
% tracking weight
Qf = zeros(1, WINDOW); % Can't stablilze unknown signal
Q1 = 1*diag([1 0.1 0.01 Qf]); % weight of integral{e}, e, de, f(k), f(k-1), ...
Q1 = 0.1*kron(Q1, eye(DIM_F)); 

% estimated weight
Qf = 10^(-3)*ones(1, WINDOW);
Q2 = diag([1 0.1 0.01 Qf]); % weight of integral{e}, e, de, f(k), f(k-1), ...
Q2 = 1*kron(Q2, eye(DIM_F));

rho = 10;

if EXE.LMI
    [uav.K, uav.L] = solveLMI10(uav.A, uav.B, uav.C, Eb, Q1, Q2, rho);
    uav.Save('K')
    uav.Save('L') 
end
% disp(norm(K))
% disp(norm(L))

%% trajectory
if EXE.TRAJ
    uav.tr.dt           = dt; % Time step
    uav.tr.T            = 10; % Final time
    uav.tr.x0           = [0.1 0 0.1 0.5 0.1 0.5 0.51 0.59 0.52 0.52 0.55 0.52]';
    uav.tr.xr0          = [0 1 0.5 0 0 0.8 0 0 0 0 0 0]';
    uav.tr.IS_LINEAR    = 0; % Run fuzzy linear system or origin nonlinear system
    uav.tr.IS_RK4       = 0; % Run RK4 or Euler method

    uav = uav.trajectory();
    uav.Save('tr');
end

if EXE.PLOT
    uav.tr.plot();
end


%% Execution time
toc

%% Controlability
% rank(ctrb(uav.A, uav.B))

%% Debug

%% Remove path
rmpath(genpath('function'))
rmpath(genpath('../../../src'))

%% functions
function y = symmetric(x)
    y = x + x';
end
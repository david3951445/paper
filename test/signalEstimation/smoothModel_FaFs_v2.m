%v2, Ka = [-I 0 ... 0]

clc; clear; close all
addpath(genpath('../../src'))

dt = .001; t = 0 : dt : 10;
rho = 10000;
%% sys
DIM_F = 1; % Dimension of e
DIM_SOLVE_K = 1;
A = [0 1 0; 0 0 1; 0 0 0]; B = [0; 0; 1]; C = eye(3); % Intergral{e}, e, de
% A_ = kron(A, eye(DIM_F));
% B_ = kron(B, eye(DIM_F));
% C_ = kron(C, eye(DIM_F));
% sys = LinearModel(A_, B_, C_);
A_ = kron(A, eye(DIM_SOLVE_K));
B_ = kron(B, eye(DIM_SOLVE_K));
C_ = kron(C, eye(DIM_SOLVE_K));
sys1 = LinearModel(A_, B_, C_);
sys1.E = zeros(sys1.DIM_X);
% Error weighting. Tracking:1, Estimation:2
Q1 = 10^(-2)*[1 1 1]; % corresponding to [Intergral{e}, e, de]
sys1.Q1 = diag(Q1);
Q2 = 10^(-2)*[1 1 1]; % corresponding to [Intergral{e}, e, de]
sys1.Q2 = diag(Q2);
sys1.rho = rho;

%% smooth model (acuator)
WINDOW = 3; dt_ = 1*dt; METHOD = '2';
sys_a1 = SmoothModel(WINDOW, DIM_SOLVE_K, dt_, METHOD);
sys_a1.B = sys1.B;
sys_a1.rho = rho;

sys_a1.Q1 = zeros(WINDOW); % Can't stablilze unknown signal
Q2 = 10^(-3)*(.1.^(0 : WINDOW-1));
sys_a1.Q2 = diag(Q2);

%% smooth model (sensor)
WINDOW = 3; dt_ = 1*dt; METHOD = '1-3';
sys_s1 = SmoothModel(WINDOW, DIM_SOLVE_K, dt_, METHOD);
sys_s1.B = [1;1;1];
sys_s1.rho = rho;

sys_s1.Q1 = zeros(WINDOW);
Q2 = 10^(-3)*(.1.^(0 : WINDOW-1));
sys_s1.Q2 = diag(Q2);

%% augment sys
[A, B, C] = AugmentSystem(sys1, sys_a1, sys_s1);
sys_aug = LinearModel(A, B, C);

% sys_aug.Q1 = 10^(-1)*blkdiag(sys1.Q1, sys_a1.Q1, sys_s1.Q1); % weight of integral{e}, e, de, f1, f2
% sys_aug.Q2 = 10^(-1)*blkdiag(sys1.Q2, sys_a1.Q2, sys_s1.Q2); % weight of integral{e}, e, de, f1, f2
sys_aug.E = .1*eye(sys_aug.DIM_X);
% sys_aug.R = [];
% sys_aug.rho = rho;

%% L, K
[K, L] = solveLMI12(sys1, sys_a1, sys_s1);
% gain = zeros(1, sys_a.WINDOW); gain(1) = -1;
% K(:, sys.DIM_X + (1:sys_a.DIM_X)) = kron(gain, eye(DIM_F));
% L(sys.DIM_X+(1:sys_a.DIM_X), :) = -diag([100,200,700]);
% L(7:9,:) = -diag([10,100,100]);

%% trajectory
sys = sys1; sys_a = sys_a1; sys_s = sys_s1;
x = zeros(sys_aug.DIM_X, length(t));
xh = zeros(sys_aug.DIM_X, length(t));
x(:, 1) = [zeros(1, sys.DIM_X) zeros(1, sys_a.DIM_X) zeros(1, sys_s.DIM_X)]';

w = randn(1, length(t));
% v = 0.5*ones(sys_a.DIM, length(t)) + .5*cos(5*t);% + 1*w - 1*t + 0.001*t.^2;
% v2 = 0.1*ones(sys_s.DIM, length(t)) + .1*cos(5*t);
v1 = repmat(.2*cos(1*t), sys_a.DIM, 1);
v1_init = [repmat(v1(:, 1), 1, sys_a.WINDOW) v1];
v2 = repmat(.1*sin(1*t), sys_s.DIM, 1);
v2_init = [repmat(v2(:, 1), 1, sys_s.WINDOW) v2];

fun = @(t, p) [sys_aug.A, sys_aug.B*K; -L*sys_aug.C, sys_aug.A+sys_aug.B*K+L*sys_aug.C]*p;
for i = 1 : length(t) - 1
    xb = [x(:, i); xh(:, i)];
    xb = ODE_solver(fun, dt, xb, t(i), 'RK4');
    x(:, i+1) = xb(1:sys_aug.DIM_X);
    xh(:, i+1) = xb(sys_aug.DIM_X+(1:sys_aug.DIM_X));
    
    range = i : i + sys_a.WINDOW-1;
    v1_ = flip(reshape(v1_init(:, range)', [], 1));
    x(sys.DIM_X+(1:sys_a.DIM_X), i+1) = v1_;
    range = i : i + sys_s.WINDOW-1;
    v2_ = flip(reshape(v2_init(:, range)', [], 1));
    x(sys.DIM_X+sys_a.DIM_X+(1:sys_s.DIM_X), i+1) = v2_;
end

%% plot
figure
Layout = tiledlayout(sys.DIM_X, 1);
for i = 1 : sys.DIM_X
    nexttile
     hold on
    index = i;
    plot(t, x(index, :), '-o', 'Displayname', 'state')
    plot(t, xh(index, :), 'DisplayName', 'estimated')
    % plot(t, x(i, :)-xh(i, :), 'Displayname', 'error')
    grid on
    legend
    title(['x_' num2str(i)])
end
title(Layout, 'state')

index = sys.DIM_X;
Plot(t, x, xh, index, sys_a.DIM, 'a')

index = sys.DIM_X + sys_a.DIM_X;
Plot(t, x, xh, index, sys_s.DIM, 's')
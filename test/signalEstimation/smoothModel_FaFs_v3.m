%v3: v1 + ddf term in smooth model
clc; clear; close all
addpath(genpath('../../src'))

dt = .001; t = 0 : dt : 10;
%% sys
A = [0 1 0; 0 0 1; 0 0 0]; B = [0; 0; 1]; C = eye(3); % Intergral{e}, e, de
DIM_F = 1; % Dimension of e
A = kron(A, eye(DIM_F));
B = kron(B, eye(DIM_F));
C = kron(C, eye(DIM_F));
sys = LinearModel(A, B, C);
% Error weighting. Tracking:1, Estimation:2
Q1 = 10^3*[1 1 1]; % corresponding to [Intergral{e}, e, de]
Q1 = repelem(Q1, DIM_F); % Not nessasry for using repelem, assign to every element is ok.
sys.Q1 = diag(Q1);
Q2 = 10^3*[1 100 100]; % corresponding to [Intergral{e}, e, de]
Q2 = repelem(Q2, DIM_F);
sys.Q2 = diag(Q2);

%% smooth model (acuator)
WINDOW = 4; DIM = DIM_F;
sys_a = SmoothModel(WINDOW, DIM, 500*dt, '4');
sys_a.B = sys.B;

Q1 = 0*(.1.^(0 : sys_a.WINDOW-1)); % Can't stablilze unknown signal
Q1 = repelem(Q1, sys_a.DIM);
sys_a.Q1 = diag(Q1);
Q2 = 10^(3)*(.1.^(0 : sys_a.WINDOW-1));
Q2 = repelem(Q2, sys_a.DIM);
sys_a.Q2 = diag(Q2);

%% smooth model (sensor)
WINDOW = 4; DIM = DIM_F;
sys_s = SmoothModel(WINDOW, DIM, 800*dt, '4');
sys_s.B = kron([1; 1; 1], eye(DIM_F));

Q1 = 0*(.1.^(0 : sys_s.WINDOW-1)); % Can't stablilze unknown signal
Q1 = repelem(Q1, sys_s.DIM);
sys_s.Q1 = diag(Q1);
Q2 = 10^(1)*(.1.^(0 : sys_s.WINDOW-1));
Q2 = repelem(Q2, sys_s.DIM);
sys_s.Q2 = diag(Q2);

%% augment sys
[A, B, C] = AugmentSystem(sys, sys_a, sys_s);
sys_aug = LinearModel(A, B, C);

sys_aug.Q1 = 10^(-1)*blkdiag(sys.Q1, sys_a.Q1, sys_s.Q1); % weight of integral{e}, e, de, f1, f2
sys_aug.Q2 = 10^(-1)*blkdiag(sys.Q2, sys_a.Q2, sys_s.Q2); % weight of integral{e}, e, de, f1, f2
sys_aug.E = .1*eye(sys_aug.DIM_X);
sys_aug.R = [];
sys_aug.rho = 100;

%% L, K
[K, L, P1, P2] = solveLMI10(sys_aug.A, sys_aug.B, sys_aug.C, sys_aug.E, sys_aug.Q1, sys_aug.Q2, sys_aug.R, sys_aug.rho);
% gain = zeros(1, sys_a.WINDOW); gain(1) = -1;
% K(:, sys.DIM_X + (1:sys_a.DIM_X)) = kron(gain, eye(DIM_F));
% L(sys.DIM_X+(1:sys_a.DIM_X), :) = -diag([100,200,700]);
% L(7:9,:) = -diag([10,100,100]);

%% trajectory
x = zeros(sys_aug.DIM_X, length(t));
xh = zeros(sys_aug.DIM_X, length(t));
x(:, 1) = [zeros(1, sys.DIM_X) zeros(1, sys_a.DIM_X) zeros(1, sys_s.DIM_X)]';

w = randn(1, length(t));
% v = 0.5*ones(sys_a.DIM, length(t)) + .5*cos(5*t);% + 1*w - 1*t + 0.001*t.^2;
% v2 = 0.1*ones(sys_s.DIM, length(t)) + .1*cos(5*t);
v1 = repmat(.2*cos(1*t), sys_a.DIM, 1);
v1_init = [repmat(v1(:, 1), 1, sys_a.WINDOW) v1];
dv1 = repmat(-.2*sin(1*t), sys_a.DIM, 1);
dv1_init = [repmat(dv1(:, 1), 1, sys_a.WINDOW) dv1];
% v2 = repmat(.1*ones(1, length(t)), sys_s.DIM, 1);
v2 = repmat(.1*sin(1*t), sys_s.DIM, 1);
v2_init = [repmat(v2(:, 1), 1, sys_s.WINDOW) v2];
dv2 = repmat(.1*cos(1*t), sys_s.DIM, 1);
dv2_init = [repmat(dv2(:, 1), 1, sys_s.WINDOW) dv2];

i = 0;
range = 1 + (i : i + sys_a.WINDOW/2-1);
v1_ = flip(reshape(v1_init(:, range)', [], 1));
dv1_ = flip(reshape(dv1_init(:, range)', [], 1));
x(sys.DIM_X+(1:sys_a.DIM_X), i+1) = [v1_; dv1_];
range = 1 + (i : i + sys_s.WINDOW/2-1);
v2_ = flip(reshape(v2_init(:, range)', [], 1));
dv2_ = flip(reshape(dv2_init(:, range)', [], 1));
x(sys.DIM_X+sys_a.DIM_X+(1:sys_s.DIM_X), i+1) = [v2_; dv2_];

fun = @(t, p) [sys_aug.A, sys_aug.B*K; -L*sys_aug.C, sys_aug.A+sys_aug.B*K+L*sys_aug.C]*p;
for i = 1 : length(t) - 1
    xb = [x(:, i); xh(:, i)];
    xb = ODE_solver(fun, dt, xb, t(i), 'RK4');
    x(:, i+1) = xb(1:sys_aug.DIM_X);
    xh(:, i+1) = xb(sys_aug.DIM_X+(1:sys_aug.DIM_X));
    
    range = 1 + (i : i + sys_a.WINDOW/2-1);
    v1_ = flip(reshape(v1_init(:, range)', [], 1));
    dv1_ = flip(reshape(dv1_init(:, range)', [], 1));
    x(sys.DIM_X+(1:sys_a.DIM_X), i+1) = [v1_; dv1_];
    range = 1 + (i : i + sys_s.WINDOW/2-1);
    v2_ = flip(reshape(v2_init(:, range)', [], 1));
    dv2_ = flip(reshape(dv2_init(:, range)', [], 1));
    x(sys.DIM_X+sys_a.DIM_X+(1:sys_s.DIM_X), i+1) = [v2_; dv2_];
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
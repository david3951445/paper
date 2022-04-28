%v2, more clear code, testing Fa + Fs

clc; clear; close all
addpath(genpath('../../src'))

dt = .001; t = 0 : dt : 10;
%% sys
A = [0 1 0; 0 0 1; 0 0 0]; B = [0; 0; 1]; C = eye(3); % Intergral{e}, e, de
DIM_F = 1; % Dimension of e
A = kron(A, eye(DIM_F));
B = kron(B, eye(DIM_F));
C = kron(C, eye(DIM_F));
[DIM_X, DIM_U] = size(B);
[DIM_Y, ~] = size(C);

sys.A = A;
sys.B = B;
sys.C = C;
% Tracking:1, Estimation:2
sys.Q1 = [1 1 1]; % corresponding to [Intergral{e}, e, de]
sys.Q2 = [1 1 1]; % corresponding to [Intergral{e}, e, de]
sys.DIM = DIM_F;
sys.DIM_X = DIM_X;
%% smooth model (acuator)
WINDOW = 4; DIM = DIM_F;
sys_a = SmoothModel(WINDOW, DIM, dt);
sys_a.B = sys.B;
sys_a.Q1 = 0*10^(0)*(.1.^(0 : sys_a.WINDOW-1)); % Can't stablilze unknown signal
sys_a.Q2 = 10^(0)*(.1.^(0 : sys_a.WINDOW-1));
%% smooth model (sensor)
WINDOW = 3; DIM = 1;
sys_s = SmoothModel(WINDOW, DIM, dt);
sys_s.B = kron([1;1;1], eye(DIM_F));
sys_s.Q1 = 0*10^(-3)*(.1.^(0 : sys_s.WINDOW-1)); % Can't stablilze unknown signal
sys_s.Q2 = 10^(2)*(.1.^(0 : sys_s.WINDOW-1));
%% augment sys
sys_aug = AugmentSystem(sys, sys_a, sys_s);
Eb = 0*eye(sys_aug.DIM_X); % disturbance matrix

%% L, K
% tracking weight
Q = repelem(sys.Q1, sys.DIM); % Not nessasry for using repelem, assign to every element is ok.
Qa = repelem(sys_a.Q1, sys_a.DIM);
Qs = repelem(sys_s.Q1, sys_s.DIM); 
Q1 = 10^(-1)*diag([Q Qa Qs]); % weight of integral{e}, e, de, f(k), f(k-1), ...

% estimated weight
Q = repelem(sys.Q2, sys.DIM);
Qa = repelem(sys_a.Q2, sys_a.DIM);
Qs = repelem(sys_s.Q2, sys_s.DIM);
Q2 = 10^(2)*diag([Q Qa Qs]); % weight of integral{e}, e, de, f(k), f(k-1), ...
R = [];
rho = 10;

[K, L] = solveLMI10(sys_aug.A, sys_aug.B, sys_aug.C, Eb, Q1, Q2, R, rho);
%     L(1:2) = [-100; -200]*10;
%     L(7:8) = [500; 1000];

%% trajectory
x = zeros(sys_aug.DIM_X, length(t));
xh = zeros(sys_aug.DIM_X, length(t));
x(:, 1) = [ones(1, sys.DIM_X) zeros(1, sys_a.DIM_X) zeros(1, sys_s.DIM_X)]';

w = randn(1, length(t));
% v = 0.5*ones(sys_a.DIM, length(t)) + .5*cos(5*t);% + 1*w - 1*t + 0.001*t.^2;
% v2 = 0.1*ones(sys_s.DIM, length(t)) + .1*cos(5*t);
v = .5*cos(1*t);
v2 = .1*cos(1*t);

fun = @(t, p) [sys_aug.A, sys_aug.B*K; -L*sys_aug.C, sys_aug.A+sys_aug.B*K+L*sys_aug.C]*p;
for i = 1 : length(t) - 1
    xb = [x(:, i); xh(:, i)];
    xb = ODE_solver(fun, dt, xb, t(i), 'RK4');
    x(:, i+1) = xb(1:sys_aug.DIM_X);
    xh(:, i+1) = xb(sys_aug.DIM_X+(1:sys_aug.DIM_X));

    x(sys_a.DIM_X+(1:sys_a.DIM), i+1) = v(:, i);
    x(sys_a.DIM_X+sys_s.DIM_X+(1:sys_s.DIM), i+1) = v2(:, i);
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

figure
Layout = tiledlayout(sys_a.DIM, 1);
for i = 1 : sys_a.DIM
    nexttile
     hold on
    index =  sys_a.DIM_X + i;
    plot(t, x(i, :), '-o', 'Displayname', 'state')
    plot(t, xh(i, :), 'DisplayName', 'estimated')
    % plot(t, x(i, :)-xh(i, :), 'Displayname', 'error')
    grid on
    legend
    title(['va_' num2str(i)])
end
title(Layout, 'Fa')

figure
Layout = tiledlayout(sys_s.DIM, 1);
for i = 1 : sys_s.DIM
    nexttile
    hold on
    index = sys_a.DIM_X + sys_s.DIM_X + i;
    plot(t, x(index, :), '-o', 'Displayname', 'state')
    plot(t, xh(index, :), 'DisplayName', 'estimated')
    grid on
    legend
    title(['vs_' num2str(i)])
end
title(Layout, 'Fs')
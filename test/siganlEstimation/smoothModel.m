%Test smooth model performance
%
% Given a state-space system:
%   dx/dt = Ax + v
% where v is unknown siganl Our goal is to estimate v. Here, we use a
% smooth model to model the unknown siganl. The smooth model is construct
% by Finite Difference Method. After unknown siganl has a model, we can
% use a Luenberger observer to estimate it via augment it into the state.
%
% The observer gain L is found by H-infinity Theorem.

clc; clear; close all
addpath(genpath('../../src'))

% system
A = -1; C = 1;
[DIM_U, DIM_X] = size(C);
I = eye(DIM_X);
dt = 0.001; T = 10; t = 0 : dt : T;

MIN_e = realmax;
MAX_WINDOW = 10;
e = zeros(1, MAX_WINDOW-1);
x_log = cell(1, MAX_WINDOW-1);
xh_log = cell(1, MAX_WINDOW-1);
for WINDOW = 2 : MAX_WINDOW
% smooth model
% WINDOW = 10;
DIM_X2 = DIM_X*WINDOW;
Aa = zeros(WINDOW);
for i = 1 : WINDOW
    point = i-1 : -1 : -WINDOW+i;
    % another method, wrong in theory, but good performance
%     point = 0 : -1 : -WINDOW+1; 

    Aa(i, 1:WINDOW) = FindFDC(point, 1)'; % obtain coefficient
end
Aa = kron(Aa, I);
Ca = zeros(1, WINDOW); Ca(1) = 1;
Ca = kron(Ca, I);

% augment system
Ab = [A Ca; zeros(DIM_X2, DIM_X) Aa];
Cb = [C zeros(DIM_U, DIM_X2)];
DIM_X3 = size(Ab, 1);

% Qa = 10^(-10).^(WINDOW-1:-1:0)
Qa = 10^(-10)*ones(1, WINDOW);
Q = diag([1 Qa]);
Q = kron(I, Q); rho = 1;

L = solveLMI7(Ab, Cb, Q, rho);
L = L*1;

x = zeros(DIM_X3, length(t));
xh = zeros(DIM_X3, length(t));

x(:, 1) = [1; zeros(DIM_X2, 1)];
w = randn(1, length(t));
v = 5*cos(t) - 0.5*t + 0.01*t.^2;
dvdt = -5*sin(t) -0.5 + 0.02*t + w;
dvdt_withInit = [zeros(1, WINDOW), dvdt];

for i = 1 : length(t) - 1
    i_v_withInit = i + WINDOW;
    k = [
        [A Ca]*x(:, i)
        dvdt_withInit(i_v_withInit-1 : -1 : i_v_withInit-WINDOW)'
    ];
    x(:, i+1) = x(:, i) + dt*k;

    k = Ab*xh(:, i) + L*Cb*(x(:, i) - xh(:, i));
    xh(:, i+1) = xh(:, i) + dt*k;
end

e(WINDOW-1) = norm(x(2, :) - xh(2, :));
if e(WINDOW-1) < MIN_e
    MIN_e = e(WINDOW-1);
    MIN_e_WINDOW = WINDOW;
end

% disp(['error of attack signal and its estimation: ' num2str(e(WINDOW-1))])
x_log{WINDOW} = x;
xh_log{WINDOW} = xh;
end

figure
disp(['Min error of attack signal and its estimation: ' num2str(MIN_e)])
plot(e)
title('error of attack signal w.r.t window size')
xlabel('window size'); ylabel('error')

% plot
figure
TITLE = {'x'};
WIMDOW_peek = 3; % how many delay disturbance you want to plot
for i = 1 : WIMDOW_peek+1
    TITLE{i+1} = ['v(t-' num2str(i-1) ')'];
end
Layout = tiledlayout(WIMDOW_peek+1, 1);
for i = 1 : WIMDOW_peek+1
    nexttile
    hold on
    plot(t, x_log{MIN_e_WINDOW}(i, :), 'Displayname', 'state')
    plot(t, xh_log{MIN_e_WINDOW}(i, :), 'DisplayName', 'estimated')
    hold off
    legend
    title(TITLE{i})
end
title(Layout,'state and attack signal trajectory with best window size')
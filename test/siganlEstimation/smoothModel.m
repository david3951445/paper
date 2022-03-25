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
A = -2; C = 1;
[DIM_U, DIM_X] = size(C);
I = eye(DIM_X);
dt = 0.001; T = 5; t = 0 : dt : T;

MIN_e = realmax;
MAX_WINDOW = 10;
e = zeros(1, MAX_WINDOW-1);
x_log = cell(1, MAX_WINDOW-1);
xh_log = cell(1, MAX_WINDOW-1);
for WINDOW = 2 : MAX_WINDOW
    % smooth model
    DIM_X2 = DIM_X*WINDOW;
    Aa = zeros(WINDOW);
    
    % method 1
%     point = 0 : -1 : -WINDOW+1;
%     Aa(1, 1:WINDOW) = FindFDC(point, 1)'/dt;
    % method 1-1
    point = zeros(WINDOW, 1); point(1:2) = [1 -1];
    Aa(1, 1:WINDOW) = point/dt;

    for i = 2 : WINDOW
        point = [1 0];
        Aa(i, i-1:i) = FindFDC(point, 1)'/dt; % obtain coefficient
    end
    
    % method 3
%     point = zeros(WINDOW, 1); point(WINDOW-1:WINDOW) = [1 -1];
%     Aa(WINDOW, 1:WINDOW) = point/dt;
%     for i = 1 : WINDOW-1
%         point = [0 -1];
%         Aa(i, i:i+1) = FindFDC(point, 1)'/dt; % obtain coefficient
%     end
    
    % method 2
%     for i = 1 : WINDOW
%         point = i-1 : -1 : -WINDOW+i;
%         Aa(i, :) = FindFDC(point, 1)'/dt;
%     end

    if WINDOW == 5
%         Aa
%         Aa = [1 -1 0 0 0; 1 -1 0 0 0; 0 1 -1 0 0; 0 0 1 -1 0; 0 0 0 1 -1]/dt;
    end
    Aa = kron(Aa, I);
    Ca = zeros(1, WINDOW); Ca(1) = 1;
    Ca = kron(Ca, I);

    % augment system
    Ab = [A Ca; zeros(DIM_X2, DIM_X) Aa];
    Cb = [C zeros(DIM_U, DIM_X2)];
    DIM_X3 = size(Ab, 1);

    % Qa = 10^(-10).^(WINDOW-1:-1:0)
    Qa = 10^(-3)*ones(1, WINDOW);
    Q = diag([10^(-3) Qa]);
    Q = kron(I, Q);
    Q(1:DIM_X,1:DIM_X) = 1;
    rho = 1;

    L = solveLMI7(Ab, Cb, Q, rho);
    L = L*1;
    norm(L)
    
    x = zeros(DIM_X3, length(t));
    xh = zeros(DIM_X3, length(t));
    x(:, 1) = [1; zeros(DIM_X2, 1)];
  
    w = randn(1, length(t));
%     v = 0.5*cos(10*t) +0*w - 1*t + 0.001*t.^2;
    v = 1*w;
    v_withInit = [zeros(1, WINDOW-1), v];
%     xh(:, 1) = [1; v_withInit(WINDOW:-1:1)'];
    for i = 1 : length(t) - 1
        i_v_withInit = i + WINDOW-1;
        k = [A Ca]*x(:, i);
    %         dvdt_withInit(i_v_withInit-1 : -1 : i_v_withInit-WINDOW)'
    %     ];
        x(:, i+1) = [
            x(DIM_X, i) + dt*k
            v_withInit(i_v_withInit : -1 : i_v_withInit-WINDOW+1)'
        ];

        kh = Ab*xh(:, i) + L*Cb*(x(:, i) - xh(:, i));
        xh(:, i+1) = xh(:, i) + dt*kh;
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
plot(2:MAX_WINDOW, e)
title('error of attack signal w.r.t window size')
xlabel('window size'); ylabel('error')

% plot
figure
TITLE = {'x'};
WIMDOW_peek = 2; % how many delay disturbance you want to plot
for i = 1 : WIMDOW_peek+1
    TITLE{i+1} = ['v(t-' num2str(i-1) ')'];
end
Layout = tiledlayout(WIMDOW_peek+1, 1);

for i = 1 : WIMDOW_peek+1
    nexttile
    hold on
    if 1%i == 1
        plot(t, x_log{MIN_e_WINDOW}(i, :), 'Displayname', 'state')
        plot(t, xh_log{MIN_e_WINDOW}(i, :), 'DisplayName', 'estimated')
    else
        plot(t, x_log{MIN_e_WINDOW}(i, :)-xh_log{MIN_e_WINDOW}(i, :), 'Displayname', 'error')
    end
    hold off
    legend
    title(TITLE{i})
end
title(Layout,'state and attack signal trajectory with best window size')
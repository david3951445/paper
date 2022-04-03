%smooth model test, with stabilized
% Chose of WINDOM may related to method of calculate dxdt of x
% Here use lagrange method so WINDOM size about 2~3 is good. Still needed to validate.
% Control gain of estimated unknoen siganl should chose "[-I 0, .... 0]" rather calculated
% by H inf theorem due to we are going to canceling f(t) term i.e. -fh + f = 0.
% Add sensor fault Fs
clc; clear; close all
addpath(genpath('../../src'))

% system
O = zeros(3); I = eye(3);
dt = 0.001; T = 5; t = 0 : dt : T;
A = [0 1 0; 0 0 1; 0 0 0]; B = [0; 0; 1]; C = I;
% A = [1 dt dt^2/2; 0 1 dt; 0 0 1];
% B = [dt^3/6; dt^2/2; dt];
% C = I;
% A = .1; B = 1; C = -1;
DIM_F = 1;
[DIM_X, DIM_U] = size(B);
[DIM_Y, ~] = size(C);
I = eye(DIM_X);

MIN_e = realmax;
MIN_e_WINDOW = 2;
MAX_WINDOW = 5;
e = zeros(1, MAX_WINDOW-1);
x_log = cell(1, MAX_WINDOW-1);
xh_log = cell(1, MAX_WINDOW-1);
for WINDOW = 2 : MAX_WINDOW
    Aa = zeros(WINDOW);
    % method 1-1
%     point = 0 : -1 : -WINDOW+1;
%     Aa(1, 1:WINDOW) = FindFDC(point, 1)'/dt;
    % method 1-2
%     point = zeros(WINDOW, 1); point(1:2) = [1 -1];
%     Aa(1, 1:WINDOW) = point/dt;
    % method 1-3
%     coeff2 = 0.1.^(0:WINDOW-1);
%     coeff2 = coeff2/sum(coeff2);
%     Aa(1, 1:WINDOW) = zeros(1, WINDOW); Aa(1,1) = -1;
%     Aa(1, 1:WINDOW) = (Aa(1, 1:WINDOW) + coeff2)/dt;
    for i = 2 : WINDOW
        point = [1 0];
        Aa(i, i-1:i) = FindFDC(point, 1)'/dt; % obtain coefficient
    end
    % method 2
    for i = 1 : WINDOW
        point = i-1 : -1 : -WINDOW+i;
        Aa(i, :) = FindFDC(point, 1)'/dt/1000;
    end
    
    Ca = zeros(1, WINDOW); Ca(:, 1) = 1;
    Cs = [1;1;1]*Ca;
    DIM_Ys = size([1;1;1], 2);
    As = kron(Aa/1, eye(DIM_Ys));
    
    DIM_X2 = DIM_F*WINDOW;
    DIM_X2_Y = DIM_F*DIM_Ys*WINDOW;
    
    %% augment system
    Ab = [A B*Ca zeros(DIM_X, WINDOW*DIM_Ys); zeros(WINDOW, DIM_X) Aa zeros(WINDOW, WINDOW*DIM_Ys); zeros(WINDOW*DIM_Ys, DIM_X) zeros(WINDOW*DIM_Ys, WINDOW) As];
    Bb = [B; zeros(WINDOW, DIM_U); zeros(WINDOW*DIM_Ys, DIM_U)];
    Cb = [C zeros(DIM_Y, WINDOW) Cs];
    Ab = kron(Ab, eye(DIM_F));
    Bb = kron(Bb, eye(DIM_F));
    Cb = kron(Cb, eye(DIM_F));
    DIM_X3 = size(Ab, 1);
    Eb = 0*kron(diag([1 1 1 ones(1, WINDOW) ones(1, WINDOW*DIM_Ys)]), eye(DIM_F)); % disturbance matrix

    %% L, K
    % tracking weight
    Qf = 0*ones(1, WINDOW*(1+DIM_Ys)); % Can't stablilze unknown signal
    Q1 = 10^(0)*diag([1 1 1 Qf]); % weight of integral{e}, e, de, f(k), f(k-1), ...
    Q1 = kron(Q1, eye(DIM_F)); 
%     Q11 = 100*diag([10 100 10]);
%     Q11 = 1*kron(Q11, eye(DIM_F)); 

    % estimated weight
    Qf = 1*ones(1, WINDOW*(1+DIM_Ys));
    Q2 = 10^(3)*diag([1 1 1 Qf]); % weight of integral{e}, e, de, f(k), f(k-1), ...
    Q2 = kron(Q2, eye(DIM_F));
    R = [];
    rho = 100;
   
    gain = zeros(1, WINDOW); gain(1) = -1;
    % method 1
    [K, L] = solveLMI10(Ab, Bb, Cb, Eb, Q1, Q2, R, rho);
%     K(1:DIM_U*DIM_F, DIM_F*DIM_X + (1:DIM_X2)) = kron(gain, eye(DIM_F));
    % method 2, remove unknown siganl state in augment state
%     [K, L] = solveLMI11(Ab, Cb, Eb, Q11, Q2, R, rho, kron(B*Ca, eye(DIM_F)), kron(A, eye(DIM_F)), kron(B, eye(DIM_F)));
%     K = [K kron(gain, eye(DIM_F))];
%     eig([Ab+Bb*K -Bb*K; zeros(size(Ab)) Ab+L*Cb])
    
    %% trajectory
    x = zeros(DIM_X3, length(t));
    xh = zeros(DIM_X3, length(t));
    x(:, 1) = [ones(1, DIM_X*DIM_F) zeros(1, DIM_X2) zeros(1, DIM_X2_Y)]';

    w = randn(1, length(t));
    v = 0*ones(DIM_F, length(t)) + 2*cos(5*t) + 1*w - 1*t + 0.001*t.^2;
    v2 = 0*ones(DIM_F*DIM_Ys, length(t)) + 1*cos(5*t);
        
    fun = @(t, p) [Ab Bb*K; -L*Cb Ab+Bb*K+L*Cb]*p;
    for i = 1 : length(t) - 1
        xb = [x(:, i); xh(:, i)];
        xb = RKF(fun, dt, xb, t(i), 'RK4');
        x(:, i+1) = xb(1:DIM_X3);
        xh(:, i+1) = xb(DIM_X3+(1:DIM_X3));
        
        x(DIM_X*DIM_F+(1:DIM_F), i+1) = v(:, i);
        x(DIM_F*(DIM_X+WINDOW)+(1:DIM_F*DIM_Ys), i+1) = v2(:, i);
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
for i = 1 : DIM_F*DIM_X
    TITLE{i}  = ['x_' num2str(i)];
end
for i = 1 : DIM_F
    TITLE{DIM_F*DIM_X+i} = ['v_' num2str(i)];
end
figure
Layout = tiledlayout(DIM_X+1, DIM_F);

for i = 1 : DIM_F*DIM_X+DIM_F
    nexttile
    hold on
    if 1%i == 1
        plot(t, x_log{MIN_e_WINDOW}(i, :), '-o', 'Displayname', 'state')
        plot(t, xh_log{MIN_e_WINDOW}(i, :), 'DisplayName', 'estimated')
    else
        plot(t, x_log{MIN_e_WINDOW}(i, :)-xh_log{MIN_e_WINDOW}(i, :), 'Displayname', 'error')
    end
    grid on
    legend
    title(TITLE{i})
end
title(Layout, ['state, Fa trajectory with best window size ' num2str(MIN_e_WINDOW)])

figure
tiledlayout(DIM_Ys, DIM_F);
for i = 1 : DIM_F*DIM_Ys
    nexttile
    hold on
    index = DIM_F*(DIM_X+MIN_e_WINDOW) + i;
    plot(t, x_log{MIN_e_WINDOW}(index, :), '-o', 'Displayname', 'state')
    plot(t, xh_log{MIN_e_WINDOW}(index, :), 'DisplayName', 'estimated')
    grid on
    legend
end
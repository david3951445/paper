%main script
% one Robot, CTM, reference model tracking control, Fa + Fs, K solved by DIM = 1
clc; clear; close all; tic; % warning off
addpath(genpath('../../../src'))

uav = UAV_AGENTmodel();
dt = uav.dt;

%% sys
DIM_F = uav.DIM_F; % Dimension of e
DIM_SOLVE_K = 1; 
A1 = [0 1 0; 0 0 1; 0 0 0]; B1 = [0; 0; 1]; C1 = [1 0 0; 0 1 0; 0 0 1]; % Intergral{e}, e, de
A = kron(A1, eye(DIM_F));
B = kron(B1, eye(DIM_F));
C = kron(C1, eye(DIM_F));
sys = LinearModel(A, B, C);
A = kron(A1, eye(DIM_SOLVE_K));
B = kron(B1, eye(DIM_SOLVE_K));
C = kron(C1, eye(DIM_SOLVE_K));
sys1 = LinearModel(A, B, C);
% Error weighting. Tracking:1, Estimation:2
Q1 = 10^(0)*[1 100 10]; % corresponding to [Intergral{e}, e, de]
sys1.Q1 = diag(Q1);
Q2 = 10^(1)*[1 100 100]; % corresponding to [Intergral{e}, e, de]
sys1.Q2 = diag(Q2);

%% smooth model (acuator)
WINDOW = 3; dt_ = 1*dt; METHOD = '2';
sys_a1 = SmoothModel(WINDOW, DIM_SOLVE_K, dt_, METHOD);
sys_a = SmoothModel(WINDOW, DIM_F, dt_, METHOD);
sys_a1.B = sys1.B;
sys_a.B = kron(sys_a1.B, eye(DIM_F));

Q1 = 0*(.1.^(0 : WINDOW-1)); % Can't stablilze unknown signal
sys_a1.Q1 = diag(Q1);
Q2 = 10^(3)*(.1.^(0 : WINDOW-1));
sys_a1.Q2 = diag(Q2);

%% smooth model (sensor)
WINDOW = 3; dt_ = 1*dt; METHOD = '1-3';
sys_s1 = SmoothModel(WINDOW, DIM_SOLVE_K, dt_, METHOD);
sys_s = SmoothModel(WINDOW, DIM_F, dt_, METHOD);
sys_s1.B = [0; .1; .2];
sys_s.B = kron(sys_s1.B, eye(DIM_F));

Q1 = 0*(.1.^(0 : WINDOW-1)); % Can't stablilze unknown signal
sys_s1.Q1 = diag(Q1);
Q2 = 10^(1)*(.1.^(0 : WINDOW-1));
sys_s1.Q2 = diag(Q2);

%% augment sys
[A, B, C] = AugmentSystem(sys.A, sys.B, sys.C, sys_a.A, sys_a.B, sys_a.C, sys_s.A, sys_s.B, sys_s.C);
sys_aug = LinearModel(A, B, C);
[A, B, C] = AugmentSystem(sys1.A, sys1.B, sys1.C, sys_a1.A, sys_a1.B, sys_a1.C, sys_s1.A, sys_s1.B, sys_s1.C);
sys_aug1 = LinearModel(A, B, C);

sys_aug1.Q1 = 10^(-2)*blkdiag(sys1.Q1, sys_a1.Q1, sys_s1.Q1); % weight of integral{e}, e, de, f1, f2
sys_aug1.Q2 = 10^(-2)*blkdiag(sys1.Q2, sys_a1.Q2, sys_s1.Q2); % weight of integral{e}, e, de, f1, f2
sys_aug1.E = .1*eye(sys_aug1.DIM_X);
sys_aug1.R = 10^(-3)*eye(sys_aug1.DIM_U);
sys_aug1.rho = 5;

%% some mapping
uav.sys = sys;
uav.sys_a = sys_a;
uav.sys_s = sys_s;
uav.sys_aug = sys_aug;

if EXE.LMI
    disp('solving LMI ...')
    [K, KL] = solveLMI10(sys_aug1.A, sys_aug1.B, sys_aug1.C, sys_aug1.E, sys_aug1.Q1, sys_aug1.Q2, sys_aug1.R, sys_aug1.rho);
    uav.K = K;
    uav.KL = KL;

    uav.Save('K') 
    uav.Save('KL') 
end
I = eye(DIM_F);
uav.K = kron(uav.K, I);
uav.KL = kron(uav.KL, I);

%% trajectory
% Construct reference r(t) = [xd, yd, zd, phid]^T
uav.tr.dt   = dt; % Time step
uav.tr.T    = 10; % Final time
uav.tr.t    = 0 : dt : uav.tr.T;
uav.tr.LEN  = length(uav.tr.t);

uav.qr  = zeros(4, uav.tr.LEN);
amp_z   = 0.9;
amp     = 0.8;
freg    = 1;
for i = 1 : uav.tr.LEN - 1
    uav.qr(:, i) = [
        amp*sin(freg*uav.tr.t(i))
        amp*cos(freg*uav.tr.t(i))
        amp_z*uav.tr.t(i) + 1
        0
    ];
end

if EXE.TRAJ
    %% set initial
    x0_pos = zeros(1, sys.DIM_X);
    uav.tr.x0    = [x0_pos zeros(1, sys_a.DIM_X) zeros(1, sys_s.DIM_X)]';
    uav.tr.xh0   = zeros(uav.sys_aug.DIM_X, 1);

    %% set distuuavance
    uav.tr.f1    = repmat(.2*sin(1*uav.tr.t), sys_a.DIM, 1);
    % uav.tr.f2    = repmat(.01*sin(1*uav.tr.t), sys_s.DIM, 1);

    uav.tr.f2    = 0.1*ones(sys_s.DIM, uav.tr.LEN);
    % b = [0.1 -0.05 0.05]; n = length(b)+1;
    % a = round(linspace(1,uav.tr.LEN,n));
    % for i = 1 : n-1
    %     uav.tr.f2(:, a(i):a(i+1)) = b(i);
    % end

    uav = uav.trajectory();
    uav.Save('tr');
end

if EXE.PLOT
    disp('Ploting trajectory ...')
    
    %% state, error, estimated state
    fig = figure;
    DIM = DIM_F;
    div = divisors(DIM);
    i = ceil((length(div))/2);
    Layout = tiledlayout(DIM/div(i), div(i));
    
    r = uav.tr.r{1};
    % timeInterval = 1:length(uav.tr.t)-1;
    for i = 1 : DIM_F % position
        nexttile
        hold on
        index = DIM_F + i;
        plot(uav.tr.t, uav.tr.x(index, :)+r(i, :), 'DisplayName', 'state', 'LineWidth', 2)
        plot(uav.tr.t, uav.tr.xh(index, :)+r(i, :), 'DisplayName', 'estimated', 'LineWidth', 2)
        plot(uav.tr.t, r(i, :), 'DisplayName', 'reference', 'LineWidth', 2)
        
        title(['$q_' num2str(i) '$'], 'Interpreter','latex')
        legend
        xlabel("t")
        % ylim([-2 2])
    end
    
    FILE_NAME = ['results/fig' num2str(fig.Number) '.pdf'];
    saveas(fig, FILE_NAME)
    FILE_NAME = ['data/fig/fig' num2str(fig.Number) '.fig'];
    savefig(FILE_NAME)
    
    %% fault signals (Fa, Fs)
    start_index = sys.DIM_X;
    Plot(uav.tr.t, uav.tr.x, uav.tr.xh, start_index, sys_a.DIM, 'a')
    % Plot(uav.tr.t, uav.tr.x, uav.tr.xh, index, 1, 'a')
    start_index = sys.DIM_X + sys_a.DIM_X;
    Plot(uav.tr.t, uav.tr.x, uav.tr.xh, start_index, sys_s.DIM, 's')
    % Plot(uav.tr.t, uav.tr.x, uav.tr.xh, index, 1, 's')
    
    %% control u(t)
    fig = figure;
    DIM = size(uav.tr.u, 1);
    div = divisors(DIM);
    i = ceil((length(div))/2);
    Layout = tiledlayout(DIM/div(i), div(i));
    
    % timeInterval = 1:length(uav.tr.t)-1;
    for i = 1 : DIM % position
        nexttile
        hold on
        index = i;
        plot(uav.tr.t, uav.tr.u(index, :), 'DisplayName', 'control', 'LineWidth', 2)    
        title(['$u_' num2str(i) '$'], 'Interpreter','latex')
        legend
        xlabel("t")
        % ylim([-2 2])
    end
    
%     FILE_NAME = ['results/fig' num2str(fig.Number) '.pdf'];
%     saveas(fig, FILE_NAME)
%     FILE_NAME = ['data/fig/fig' num2str(fig.Number) '.fig'];
%     savefig(FILE_NAME)
end

%% Execution time
toc
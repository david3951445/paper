%main script
% one Robot, CTM, reference model tracking control, Fa + Fs, K solved by DIM = 1
clc; clear; close all; tic;
addpath(genpath('../../../src'))
addpath(genpath('function'))

rb = Robot();

dt = .001;
%% sys
DIM_F = rb.DIM_F; % Dimension of e
DIM_SOLVE_K = 1; 
A1 = [0 1 0; 0 0 1; 0 0 0]; B1 = [0; 0; 1]; C1 = eye(3); % Intergral{e}, e, de
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
sys_a.B = sys.B;

Q1 = 0*(.1.^(0 : WINDOW-1)); % Can't stablilze unknown signal
sys_a1.Q1 = diag(Q1);
Q2 = 10^(3)*(.1.^(0 : WINDOW-1));
sys_a1.Q2 = diag(Q2);

%% smooth model (sensor)
WINDOW = 3; dt_ = 1*dt; METHOD = '1-3';
sys_s1 = SmoothModel(WINDOW, DIM_SOLVE_K, dt_, METHOD);
sys_s = SmoothModel(WINDOW, DIM_F, dt_, METHOD);
sys_s1.B = [0; .1; 1];
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
sys_aug1.R = [];
sys_aug1.rho = 5;

%% some mapping
rb.sys = sys;
rb.sys_a = sys_a;
rb.sys_s = sys_s;
rb.sys_aug = sys_aug;

if EXE.LMI
    disp('solving LMI ...')
    [K, KL] = solveLMI10(sys_aug1.A, sys_aug1.B, sys_aug1.C, sys_aug1.E, sys_aug1.Q1, sys_aug1.Q2, sys_aug1.R, sys_aug1.rho);
    rb.K = K;
    rb.KL = KL;

    rb.Save('K') 
    rb.Save('KL') 
end
% Fine tune of gain
% gain = [-1 zeros(1, sys_a.WINDOW-1)];
% rb.K(:, sys1.DIM_X + (1:sys_a.WINDOW)) = gain;
% rb.KL(4:6, :) = [1000 0 0; 0 100 0; 0 0 10];
% rb.KL(7:9,:) = [1000 0 0; 0 100 0; 0 0 10];
% rb.K(:, sys1.DIM_X + sys_a.WINDOW + (1:sys_s.WINDOW)) = zeros(1, sys_s.WINDOW);
% I0 = diag(1.1.^(0:rb.DIM_F-1));
I = eye(DIM_F);
rb.K = kron(rb.K, I);
rb.KL = kron(rb.KL, I);
% disp(norm(K))
% disp(norm(L))

%% trajectory
% Find task space ref
pp = PathPlanning();
rb.r = pp.r;

% find joint ref
rb = rb.Ref2Config(); % rb.r -> rb.qr

% testing ref
% t = 0 : dt : 10;
% ref = zeros(DIM_F, length(t));
% ref = repmat(.1*cos(t), DIM_F, 1);
% ref(1:DIM_F/2, :) = repmat(.1*cos(t), DIM_F/2, 1);
% ref(DIM_F/2+1:DIM_F, :) = repmat(.1*sin(t), DIM_F/2, 1);
% rb.qr = ref;

if EXE.TRAJ
    rb.tr.dt    = dt; % Time step
    rb.tr.LEN   = length(rb.qr);
    rb.tr.T     = dt*(rb.tr.LEN-1); % Final time
    rb.tr.t     = 0 : dt : rb.tr.T;
    %% set initial
    % x0_pos = [0.2 0.2 0 0.1 0.1 0.5 0.2 0.2 0 0.1 0.1 0.5];
    % x0_pos = .1*[0.2 0.2 0 0.1 0.1 0.5 0.2 0.2 0 0.1 0.1 0.5];
    x0_pos = zeros(1, sys.DIM_X);
%     x0_pos = [zeros(1,DIM_F) x0_pos zeros(DIM_F)]
    rb.tr.x0    = [x0_pos zeros(1, sys_a.DIM_X) zeros(1, sys_s.DIM_X)]';
    rb.tr.xh0   = zeros(rb.sys_aug.DIM_X, 1);
    %% set disturbance
    rb.tr.f1    = repmat(.2*sin(1*rb.tr.t), sys_a.DIM, 1);
    % rb.tr.f2    = repmat(.01*sin(1*rb.tr.t), sys_s.DIM, 1);
    rb.tr.f2    = 0.1*ones(sys_s.DIM, rb.tr.LEN);

    rb = rb.trajectory();
    rb.Save('tr');
end

if EXE.PLOT
    disp('Ploting trajectory ...')
    
    %% fig, map and path
%     fig = figure;
%     show(pp.map);
%     hold on;
%     plot(pp.tree(:,1), pp.tree(:,2),'.-', 'DisplayName','tree expansion'); % tree expansion
%     % plot(pthObj.States(:,1), pthObj.States(:,2),'r-','LineWidth',2, 'DisplayName','path') % draw path  
%     plot(rb.r(1, :), rb.r(2, :), '-o', 'DisplayName', 'r(t)')
%     len2 = length(rb.r);
%     plot(rb.r_lr(1, 1:2:len2), rb.r_lr(2, 1:2:len2), '-o', 'DisplayName', 'left foot')
%     plot(rb.r_lr(1, 2:2:len2), rb.r_lr(2, 2:2:len2), '-o', 'DisplayName', 'right foot')
%     plot(rb.zmp(1, :), rb.zmp(2, :), '-s', 'Displayname', 'ZMP trajectory')
%     plot(rb.CoM(1, :), rb.CoM(2, :), 'Displayname', 'CoM trajectory')
%     axis equal
%     title('foot trajectory')
%     xlabel('x'); ylabel('y')
%     legend
%     
%     FILE_NAME = ['results/fig' num2str(fig.Number) '.pdf'];
%     saveas(fig, FILE_NAME)
%     FILE_NAME = ['data/fig/fig' num2str(fig.Number) '.fig'];
%     savefig(FILE_NAME)
    
    %% fig
    fig = figure;
    DIM = 1;%sys.DIM_X/div(i);
    div = divisors(DIM);
    i = ceil((length(div))/2);
    Layout = tiledlayout(DIM, div(i));
    
    r = rb.tr.r{1};
    % timeInterval = 1:length(rb.tr.t)-1;
%     rb.tr.t = rb.tr.t(1:length(rb.tr.t)-1); % state have one more step than reference
    for i = 1 : 1%DIM_F % position
        nexttile
        hold on
%         plot(rb.tr.t, rb.tr.x(DIM_F+i, :), 'DisplayName', 'state')
        index = DIM_F + i;
        plot(rb.tr.t, rb.tr.x(index, :)+r(i, :), 'DisplayName', 'state', 'LineWidth', 2)
        plot(rb.tr.t, rb.tr.xh(index, :)+r(i, :), 'DisplayName', 'estimated', 'LineWidth', 2)
%         plot(t, rb.tr.x(1, :), 'DisplayName', 'state1', 'LineWidth', 2)
%         plot(t, rb.tr.x(2, :), 'DisplayName', 'state2', 'LineWidth', 2)
%         plot(t, rb.tr.x(3, :), 'DisplayName', 'state3', 'LineWidth', 2)
        plot(rb.tr.t, r(i, :), 'DisplayName', 'reference', 'LineWidth', 2)
        
        title(['$q_' num2str(i) '$'], 'Interpreter','latex')
        legend
        xlabel("t")
        % ylim([-2 2])
    end
    
    FILE_NAME = ['results/fig' num2str(fig.Number) '.pdf'];
    saveas(fig, FILE_NAME)
    FILE_NAME = ['data/fig/fig' num2str(fig.Number) '.fig'];
    savefig(FILE_NAME)
    
    %% fig, Fa and Fs
    index = sys.DIM_X;
    Plot(rb.tr.t, rb.tr.x, rb.tr.xh, index, sys_a.DIM, 'a')
    % Plot(rb.tr.t, rb.tr.x, rb.tr.xh, index, 1, 'a')
    index = sys.DIM_X + sys_a.DIM_X;
    Plot(rb.tr.t, rb.tr.x, rb.tr.xh, index, sys_s.DIM, 's')
    % Plot(rb.tr.t, rb.tr.x, rb.tr.xh, index, 1, 's')
    
%     FILE_NAME = ['results/fig' num2str(fig.Number) '.pdf'];
%     saveas(fig, FILE_NAME)
%     FILE_NAME = ['data/fig/fig' num2str(fig.Number) '.fig'];
%     savefig(FILE_NAME)
end


%% Execution time
toc

%% Controlability
% rank(ctrb(rb.A, rb.B))

%% Debug

%main script
% one Robot, CTM, reference model tracking control, no observer
clc; clear; close all; tic; % warning off
addpath(genpath('../../../src'))
addpath(genpath('function'))

rb = Robot();

dt = .001; t = 0:dt:10;
%% sys
DIM_F = 1; % Dimension of e
A = [0 1 0; 0 0 1; 0 0 0]; B = [0; 0; 1]; C = eye(3); % Intergral{e}, e, de
A = kron(A, eye(DIM_F));
B = kron(B, eye(DIM_F));
C = kron(C, eye(DIM_F));
sys = LinearModel(A, B, C);
% Error weighting. Tracking:1, Estimation:2
Q1 = [1 1 1]; % corresponding to [Intergral{e}, e, de]
Q1 = repelem(Q1, DIM_F); % Not nessasry for using repelem, assign to every element is ok.
sys.Q1 = diag(Q1);
Q2 = [1 1 1]; % corresponding to [Intergral{e}, e, de]
Q2 = repelem(Q2, DIM_F);
sys.Q2 = diag(Q2);

%% smooth model (acuator)
WINDOW = 2; DIM = DIM_F;
sys_a = SmoothModel(WINDOW, DIM, 1*dt, '1-3');
sys_a.B = sys.B;

Q1 = 0*10^(0)*(.1.^(0 : sys_a.WINDOW-1)); % Can't stablilze unknown signal
Q1 = repelem(Q1, sys_a.DIM);
sys_a.Q1 = diag(Q1);
Q2 = 10^(0)*(.1.^(0 : sys_a.WINDOW-1)); % Can't stablilze unknown signal
Q2 = repelem(Q2, sys_a.DIM);
sys_a.Q2 = diag(Q2);

%% smooth model (sensor)
WINDOW = 2; DIM = DIM_F;
sys_s = SmoothModel(WINDOW, DIM, 1*dt, '1-3');
sys_s.B = kron([1;1;1], eye(DIM_F));

Q1 = 0*10^(-3)*(.1.^(0 : sys_s.WINDOW-1)); % Can't stablilze unknown signal
Q1 = repelem(Q1, sys_s.DIM);
sys_s.Q1 = diag(Q1);
Q2 = 10^(3)*(.1.^(0 : sys_s.WINDOW-1)); % Can't stablilze unknown signal
Q2 = repelem(Q2, sys_s.DIM);
sys_s.Q2 = diag(Q2);

%% augment sys
[A, B, C] = AugmentSystem(sys, sys_a, sys_s);
sys_aug = LinearModel(A, B, C);

sys_aug.Q1 = 10^(-1)*blkdiag(sys.Q1, sys_a.Q1, sys_s.Q1); % weight of integral{e}, e, de, f1, f2
sys_aug.Q2 = 10^(-1)*blkdiag(sys.Q2, sys_a.Q2, sys_s.Q2); % weight of integral{e}, e, de, f1, f2
sys_aug.R = [];
sys_aug.rho = 10;

%% mapping
rb.sys = sys;
rb.sys_a = sys_a;
rb.sys_s = sys_s;
rb.sys_aug = sys_aug;

rb.A = sys_aug.A;
rb.B = sys_aug.B;
rb.C = sys_aug.C;
Eb = sys_aug.E;
Q1 = sys_aug.Q1;
Q2 = sys_aug.Q2;
R = sys_aug.R;
rho = sys_aug.rho;

if EXE.LMI
    disp('solving LMI ...')
    % gain = zeros(1, WINDOW); gain(1) = -1;
    % method 1
    [rb.K, rb.KL] = solveLMI10(rb.A, rb.B, rb.C, Eb, Q1, Q2, R, rho);
    % rb.K(:, DIM_X + (1:DIM_F*WINDOW)) = kron(gain, eye(DIM_F));
    % method 2
    % [rb.K, rb.L] = solveLMI11(Ab, Cb, Eb, Q11, Q2, R, rho, kron(Ca, eye(DIM_F)), kron(A, eye(DIM_F)), kron(B, eye(DIM_F)));
    % rb.K = [rb.K kron(gain, eye(DIM_F))];
    
    rb.Save('K') 
    rb.Save('KL') 
end
% Fine tune of gain
% gain = zeros(1, WINDOW); gain(1) = -1;
% rb.K(:, DIM_X + (1:DIM_F*WINDOW)) = kron(gain, eye(DIM_F));

% disp(norm(K))
% disp(norm(L))

%% trajectory
% Find task space ref
pp = PathPlanning();
rb.r = pp.r;

% find joint ref
rb = rb.Ref2Config(); % rb.r -> rb.qr

% testing ref
ref = zeros(DIM_F, length(t));
ref = .1*cos(t);
% ref(1:DIM_F/2, :) = repmat(.1*cos(t), DIM_F/2, 1);
% ref(DIM_F/2+1:DIM_F, :) = repmat(.1*sin(t), DIM_F/2, 1);
rb.qr = ref;

% dqr = gradient(qr);

if EXE.TRAJ
    rb.tr.dt           = dt; % Time step
%     rb.tr.T            = T; % Final time
    % x0_pos = [0.2 0.2 0 0.1 0.1 0.5 0.2 0.2 0 0.1 0.1 0.5];
    % x0_pos = .1*[0.2 0.2 0 0.1 0.1 0.5 0.2 0.2 0 0.1 0.1 0.5];
    x0_pos = 0;
    rb.tr.x0           = [zeros(1,DIM_F) x0_pos zeros(DIM_F) zeros(1, sys_a.DIM_X) zeros(1, sys_s.DIM_X)]';
    rb.tr.xh0          = zeros(rb.sys_aug.DIM_X, 1);

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
    tiledlayout(3, 4);
    
    r = rb.tr.r{1};
    timeInterval = 1:length(rb.tr.t)-1;
    t = rb.tr.t(timeInterval);
    for i = 1 : DIM_F % position
        nexttile
        hold on
%         plot(rb.tr.t, rb.tr.x(DIM_F+i, :), 'DisplayName', 'state')
        plot(t, rb.tr.xh(i, timeInterval)+r(i, timeInterval), 'DisplayName', 'estimated')        
        plot(t, rb.tr.x(i, timeInterval)+r(i, timeInterval), 'DisplayName', 'state')
        plot(t, r(i, timeInterval), 'DisplayName', 'reference')
        
        title(['$q_' num2str(i) '$'], 'Interpreter','latex')
        legend
        xlabel("t")
        % ylim([-2 2])
    end
    
    FILE_NAME = ['results/fig' num2str(fig.Number) '.pdf'];
    saveas(fig, FILE_NAME)
    FILE_NAME = ['data/fig/fig' num2str(fig.Number) '.fig'];
    savefig(FILE_NAME)
    
    %% fig
    fig = figure;
    Tiledlayout = tiledlayout(4, 3);
    for i = 1 : DIM_F % unknown signal
        index = 3*DIM_F + i;

        nexttile
        hold on
%         plot(t, rb.tr.f(i, timeInterval), 'DisplayName', 'state')
        plot(t, rb.tr.x(index, timeInterval), 'DisplayName', 'state')
        plot(t, rb.tr.xh(index, timeInterval), 'DisplayName', 'estimated')
        title(['$q_' num2str(i) '$'], 'Interpreter','latex')
        legend
        xlabel("t")
    end
    title(Tiledlayout, 'unknown siganl')
    
    FILE_NAME = ['results/fig' num2str(fig.Number) '.pdf'];
    saveas(fig, FILE_NAME)
    FILE_NAME = ['data/fig/fig' num2str(fig.Number) '.fig'];
    savefig(FILE_NAME)
end


%% Execution time
toc

%% Controlability
% rank(ctrb(rb.A, rb.B))

%% Debug

%% functions
function y = symmetric(x)
    y = x + x';
end
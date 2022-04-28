%main script
% one Robot, CTM, reference model tracking control, no observer
clc; clear; close all; tic; % warning off
addpath(genpath('../../../src'))
addpath(genpath('function'))

rb = Robot();

%% Construct unknown signal model

%% linear matrixs after linearize feedback(based on CTM). A, B, C
DIM_F = rb.DIM_F;
I = eye(DIM_F*3);
O = zeros(DIM_F);
A = [O eye(DIM_F) O; O O eye(DIM_F); O O O];
B = [O; O; eye(DIM_F)];
C = I;
[DIM_X, DIM_U] = size(B);
[DIM_Y, ~] = size(C);

%% A matrix for unknown signal
WINDOW = rb.WINDOW;
dt = 0.001;
Af = zeros(WINDOW);
% method 1
% point = zeros(WINDOW, 1); point(1:2) = [1 -1];
% Af(1, 1:WINDOW) = point/dt;
% for i = 2 : WINDOW
%     point = [1 0];
%     Af(i, i-1:i) = FindFDC(point, 1)'/dt; % obtain coefficient
% end
% method 2
for i = 1 : WINDOW
    point = i-1 : -1 : -WINDOW+i;
    Af(i, :) = FindFDC(point, 1)'/dt;
end
Af = kron(Af, eye(DIM_F));
Cf = zeros(1, WINDOW); Cf(:, 1) = 1;
Cf = kron(Cf, eye(DIM_F));
Cf = B*Cf;
DIM_X2 = DIM_F*WINDOW;

%% augment system
rb.A = [A Cf; zeros(DIM_X2, DIM_X) Af];
rb.B = [B; zeros(DIM_X2, DIM_U)];
rb.C = [C zeros(DIM_Y, DIM_X2)];
rb.DIM_X3 = size(rb.A, 1);
Eb = kron(diag([0 0 0 0*ones(1, WINDOW)]), eye(DIM_F)); % disturbance matrix

%% L, K
% tracking weight
Qf = zeros(1, WINDOW); % Can't stablilze unknown signal
Q1 = 1*diag([1 100 10 Qf]); % weight of integral{e}, e, de, f(k), f(k-1), ...
Q1 = kron(Q1, eye(DIM_F)); 

% estimated weight
Qf = 10*0.1.^(0:(WINDOW-1));
Q2 = 1*diag([1 100 10 Qf]); % weight of integral{e}, e, de, f(k), f(k-1), ...
Q2 = kron(Q2, eye(DIM_F));

rho = 10;

if EXE.LMI
    disp('solving LMI ...')
    % gain = zeros(1, WINDOW); gain(1) = -1;
    % method 1
    [rb.K, rb.KL] = solveLMI10(rb.A, rb.B, rb.C, Eb, Q1, Q2, [], rho);
    % rb.K(:, DIM_X + (1:DIM_F*WINDOW)) = kron(gain, eye(DIM_F));
    % method 2
    % [rb.K, rb.L] = solveLMI11(Ab, Cb, Eb, Q11, Q2, R, rho, kron(Ca, eye(DIM_F)), kron(A, eye(DIM_F)), kron(B, eye(DIM_F)));
    % rb.K = [rb.K kron(gain, eye(DIM_F))];
    
    rb.Save('K') 
    rb.Save('KL') 
end
% Fine tune of gain
gain = zeros(1, WINDOW); gain(1) = -1;
rb.K(:, DIM_X + (1:DIM_F*WINDOW)) = kron(gain, eye(DIM_F));

% rb.K(:, 19:30)=0;
% disp(norm(K))
% disp(norm(L))

%% trajectory
if EXE.QR
    % Find task space ref
    pp = PathPlanning();
    rb.r = pp.r;

    % find joint ref
    rb = rb.Ref2Config(); % rb.r -> rb.qr
end
% testing ref
% T = 7;
% t = 0:dt:7;
% ref = zeros(12, length(t));
% ref(1:DIM_F/2, :) = repmat(.1*cos(t), DIM_F/2, 1);
% ref(DIM_F/2+1:DIM_F, :) = repmat(.1*sin(t), DIM_F/2, 1);
% rb.qr = ref;

% dqr = gradient(qr);

if EXE.TRAJ
    rb.tr.dt           = dt; % Time step
%     rb.tr.T            = T; % Final time
    % x0_pos = [0.2 0.2 0 0.1 0.1 0.5 0.2 0.2 0 0.1 0.1 0.5];
    x0_pos = .1*[0.2 0.2 0 0.1 0.1 0.5 0.2 0.2 0 0.1 0.1 0.5];
    rb.tr.x0           = [zeros(1,12) x0_pos zeros(1,12) zeros(1, DIM_F*WINDOW)]';
    rb.tr.xh0          = zeros(rb.DIM_X3, 1);

    rb = rb.trajectory();
    rb.Save('tr');
end

if EXE.PLOT
    disp('Ploting trajectory ...')
    
    %% fig
    fig = figure;
    show(pp.map);
    hold on;
    plot(pp.tree(:,1), pp.tree(:,2),'.-', 'DisplayName','tree expansion'); % tree expansion
    % plot(pthObj.States(:,1), pthObj.States(:,2),'r-','LineWidth',2, 'DisplayName','path') % draw path  
    plot(rb.r(1, :), rb.r(2, :), '-o', 'DisplayName', 'r(t)')
    len2 = length(rb.r);
    plot(rb.r_lr(1, 1:2:len2), rb.r_lr(2, 1:2:len2), '-o', 'DisplayName', 'left foot')
    plot(rb.r_lr(1, 2:2:len2), rb.r_lr(2, 2:2:len2), '-o', 'DisplayName', 'right foot')
    plot(rb.zmp(1, :), rb.zmp(2, :), '-s', 'Displayname', 'ZMP trajectory')
    plot(rb.CoM(1, :), rb.CoM(2, :), 'Displayname', 'CoM trajectory')
    axis equal
    title('foot trajectory')
    xlabel('x'); ylabel('y')
    legend
    
    FILE_NAME = ['data/fig' num2str(fig.Number) '.pdf'];
    saveas(fig, FILE_NAME)
    FILE_NAME = ['results/fig/fig' num2str(fig.Number) '.fig'];
    savefig(FILE_NAME)
    
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
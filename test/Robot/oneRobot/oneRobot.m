%main script
% one Robot, CTM, reference model tracking control, no observer
clc; clear; close all; tic; % warning off
addpath(genpath('../../../src'))
addpath(genpath('function'))

%% Find task space ref
rb = Robot();
pp = PathPlanning();
rb.r = pp.r;

%% find joint ref
if EXE.QR
    rb = rb.Ref2Config(); % rb.r -> rb.qr
    rb.Save('qr');
end
qr = rb.qr;
% dqr = gradient(qr);

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
WINDOW = 2; rb.WINDOW = WINDOW;
dt = 0.001;
Af = zeros(WINDOW);
% method 1
point = zeros(WINDOW, 1); point(1:2) = [1 -1];
Af(1, 1:WINDOW) = point/dt;
for i = 2 : WINDOW
    point = [1 0];
    Af(i, i-1:i) = FindFDC(point, 1)'/dt; % obtain coefficient
end
% method 2
% for i = 1 : WINDOW
%     point = i-1 : -1 : -WINDOW+i;
%     Af(i, :) = FindFDC(point, 1)'/dt;
% end
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
Eb = kron(diag([0 0 0 ones(1, WINDOW)]), eye(DIM_F)); % disturbance matrix

%% L, K
% tracking weight
Qf = zeros(1, WINDOW); % Can't stablilze unknown signal
Q1 = 1*diag([1 0.1 0.01 Qf]); % weight of integral{e}, e, de, f(k), f(k-1), ...
Q1 = 0.1*kron(Q1, eye(DIM_F)); 

% estimated weight
Qf = 10^(-3)*ones(1, WINDOW);
Q2 = diag([1 0.1 0.01 Qf]); % weight of integral{e}, e, de, f(k), f(k-1), ...
Q2 = 1*kron(Q2, eye(DIM_F));

rho = 10;

if EXE.LMI
    disp('solving LMI ...')
    gain = zeros(1, WINDOW); gain(1) = -1;
    % method 1
    [rb.K, rb.KL] = solveLMI10(rb.A, rb.B, rb.C, Eb, Q1, Q2, [], rho);
    % rb.K(:, DIM_X + (1:DIM_F*WINDOW)) = kron(gain, eye(DIM_F));
    % method 2
    % [rb.K, rb.L] = solveLMI11(Ab, Cb, Eb, Q11, Q2, R, rho, kron(Ca, eye(DIM_F)), kron(A, eye(DIM_F)), kron(B, eye(DIM_F)));
    % rb.K = [rb.K kron(gain, eye(DIM_F))];
    
    rb.Save('K')
    rb.Save('KL') 
end
% disp(norm(K))
% disp(norm(L))
% rb.K(:, 19:30)=0;

%% trajectory
if EXE.TRAJ
    rb.tr.dt           = dt; % Time step
    rb.tr.T            = 10; % Final time
    rb.tr.x0           = [zeros(1,12) 0.2 0.2 0 0.1 0.1 0.5 0.2 0.2 0 0.1 0.1 0.5 zeros(1,12) zeros(1, DIM_F*WINDOW)]';
    rb.tr.xh0          = zeros(rb.DIM_X3, 1);
    rb.tr.IS_RK4       = 0; % Run RK4 or Euler method

    rb = rb.trajectory();
%     rb.Save('tr');
end

if EXE.PLOT
    disp('Ploting trajectory ...')

    tiledlayout(3, 4);
%     axes('Units', 'normalized', 'Position', [0 0 1 1]);
    
    r = [rb.tr.r{1}; rb.tr.r{2}];
    timeInterval = 1:length(rb.tr.t)-1;
    t = rb.tr.t(timeInterval);
    for i = 1 : DIM_F % position
        nexttile
        hold on
%         plot(rb.tr.t, rb.tr.x(DIM_F+i, :), 'DisplayName', 'state')
%         plot(rb.tr.t, rb.tr.xh(DIM_F+i, :), 'DisplayName', 'estimated')
%         timeInterval = 1 : rb.tr.LEN;
        
%         plot(t, rb.tr.x(DIM_F+i, timeInterval)+r(i, timeInterval), '-o', 'DisplayName', 'state')
        plot(t, rb.tr.x2(i, timeInterval)+r(i, timeInterval), '-o', 'DisplayName', 'state')
        plot(t, r(i, timeInterval), 'DisplayName', 'reference')
        
        title(['$q_' num2str(i) '$'], 'Interpreter','latex')
        legend
        xlabel("t")
        % ylim([-2 2])
    end
    % figure
    % Tiledlayout = tiledlayout(2, 3);
    % TITLE = {'x', 'y', 'z', '$\phi$', '$\theta$', '$\psi$'};
    % for i = 1 : DIM_F % unknown signal
    %     nexttile
    %     hold on
    %     plot(t, rb.tr.x(3*DIM_F + i, timeInterval), 'DisplayName', 'state')
    %     plot(t, rb.tr.xh(3*DIM_F + i, timeInterval), 'DisplayName', 'estimated')
    %     title(TITLE{i}, 'Interpreter','latex')
    %     legend
    %     xlabel("t")
    % end
    % title(Tiledlayout, 'unknown siganl')
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
%main script
% one UAV, fuzzy, reference model tracking control, no observer
clc; clear; close all; tic; % warning off
addpath(genpath('../../../src'))
addpath(genpath('function'))

%% linear matrixs after linearize feedback(based on CTM). A, B, C
uav = UAV_CTMmodel();
DIM_F = uav.DIM_X/2;
I = eye(DIM_F*3);
O = zeros(DIM_F);
A = [O eye(DIM_F) O; O O eye(DIM_F); O O O];
B = [O; O; eye(DIM_F)];
C = I;
[DIM_X, DIM_U] = size(B);
[DIM_Y, ~] = size(C);

%% A matrix for unknown signal
WINDOW = 2; uav.WINDOW = WINDOW;
dt = 0.001;
Af = zeros(WINDOW);
point = zeros(WINDOW, 1); point(1:2) = [1 -1];
Af(1, 1:WINDOW) = point/dt;
for i = 2 : WINDOW
    point = [1 0];
    Af(i, i-1:i) = FindFDC(point, 1)'/dt; % obtain coefficient
end
Af = kron(Af, eye(DIM_F));
Cf = zeros(1, WINDOW); Cf(:, 1) = 1;
Cf = kron(Cf, eye(DIM_F));
Cf = B*Cf;
DIM_X2 = DIM_F*WINDOW;

%% augment system
uav.A = [A Cf; zeros(DIM_X2, DIM_X) Af];
uav.B = [B; zeros(DIM_X2, DIM_U)];
uav.C = [C zeros(DIM_Y, DIM_X2)];
uav.DIM_X3 = size(uav.A, 1);
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
    [uav.K, uav.L] = solveLMI10(uav.A, uav.B, uav.C, Eb, Q1, Q2, [], rho);
    uav.Save('K')
    uav.Save('L') 
end
% disp(norm(K))
% disp(norm(L))
% uav.K(:, 19:30)=0;

%% trajectory
if EXE.TRAJ
    uav.tr.dt           = dt; % Time step
    uav.tr.T            = 5; % Final time
    uav.tr.x0           = [0 0 0 0 0 0 0.2 0.2 0 0.1 0.1 0.5 0 0 0 0 0 0 zeros(1, DIM_F*WINDOW)]';
    uav.tr.xh0          = zeros(uav.DIM_X3, 1);
    uav.tr.IS_LINEAR    = 0; % Run fuzzy linear system or origin nonlinear system
    uav.tr.IS_RK4       = 0; % Run RK4 or Euler method

    uav = uav.trajectory2();
    uav.Save('tr');
end

if EXE.PLOT
    disp('Ploting trajectory ...')

    r = [uav.tr.r{1}; uav.tr.r{2}];
    Tiledlayout = tiledlayout(3, 4);
    TITLE = {'x', 'y', 'z', '$\phi$', '$\theta$', '$\psi$', ...
        'dx', 'dy', 'dz', '$d\phi$', '$d\theta$', '$d\psi$'};
    axes('Units', 'normalized', 'Position', [0 0 1 1]);
    for i = 1 : uav.DIM_X%size(o.x, 1)
        nexttile
        hold on
%         plot(uav.tr.t, uav.tr.x(DIM_F+i, :), 'DisplayName', 'state')
%         plot(uav.tr.t, uav.tr.xh(DIM_F+i, :), 'DisplayName', 'estimated')
%         timeInterval = 1 : uav.tr.LEN;
        timeInterval = 1:length(uav.tr.t)-1;
        t = uav.tr.t(timeInterval);
        plot(t, uav.tr.x(DIM_F+i, timeInterval)+r(i, timeInterval), '-', 'DisplayName', 'state')
        plot(t, r(i, timeInterval), 'DisplayName', 'reference')
        
        title(TITLE{i}, 'Interpreter','latex')
        legend
        xlabel("t")
        % ylim([-2 2])
    end 
end


%% Execution time
toc

%% Controlability
% rank(ctrb(uav.A, uav.B))

%% Debug

%% Remove path
rmpath(genpath('function'))
rmpath(genpath('../../../src'))

%% functions
function y = symmetric(x)
    y = x + x';
end
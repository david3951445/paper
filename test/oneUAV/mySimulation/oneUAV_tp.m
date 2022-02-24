%main script
% one UAV, fuzzy, reference model tracking control, no observer
clc; clear; close all; tic; % warning off
addpath(genpath('../../../src'))
addpath(genpath('function'))

fz  = Fuzzy();
uav = UAV_TPmodel() ;
ref = REF(uav);

%% find A, B (linearize)
if EXE.A_B
    uav.AB = TPmodel();
    % If you want to tune parameter
    uav.ABl.domain       = 0.5*[-1 1; -1 1; -1 1];
    uav.ABl.gridsize     = 20*[1 1 1];
    uav.ABl.SV_TOLERANCE = 0.001;
    
    uav = uav.getAB();
    uav.Save('AB')
end 

%% find K, L
if EXE.LMI
    % If you want to tune parameter
    uav.rho             = 10^(1);
    uav.Q               = 10^(-1)*diag([1, 0.001, 1.5, 0.002, 1, 0.001, 0.1, 0, 0.1, 0, 1, 0.001]); % Correspond to x - xr
    uav.E               = 10^(-1)*diag([0 1 0 1 0 1 0 1 0 1 0 1]); % Disturbance matrix 

    uav = uav.getKL(ref);
    uav.Save('K')
end

%% trajectory
if EXE.TRAJ
    % If you want to tune parameter
    uav.tr.dt           = 0.001; % Time step
    uav.tr.T            = 10; % Final time
    uav.tr.IS_LINEAR    = 0; % Run fuzzy linear system or origin nonlinear system
    uav.tr.IS_RK4       = 1; % Run RK4 or Euler method

    uav = uav.trajectory(ref);
    uav.Save('tr');
end

if EXE.PLOT
    uav.tr.plot();
end

%% Execution time
toc

%% if you want to check if sum of membership function is 1
% checkLinearizedSum(uav.AB.val, @(x,i)uav.AB.mf(x,i))

%% Controlability
% for i = 1 : fz.num
%     disp(['Rank of linear system ' num2str(i) ': ' num2str(rank(ctrb(uav.A{i}, uav.B{i})))])   
% end

%% Remove path
rmpath(genpath('function'))
rmpath(genpath('../../../src'))
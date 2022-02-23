%main script
% one UAV, fuzzy, reference model tracking control, no observer
clc; clear; close all; tic; % warning off
addpath(genpath('../../../src'))
addpath(genpath('function'))

fz  = Fuzzy();
% If you want to tune parameter
% EXE.A_B
% fz.a = 1;
% ...

uav = UAV_TPmodel();
%% If you want to tune parameter
% EXE.LMI
uav.rho             = 10^(1);
uav.Q               = 10^(-1)*diag([1, 0.001, 1.5, 0.002, 1, 0.001, 0.1, 0, 0.1, 0, 1, 0.001]); % Correspond to x - xr
uav.E               = 10^(-1)*diag([0 1 0 1 0 1 0 1 0 1 0 1]); % Disturbance matrix
% EXE.TRAJ
uav.tr.dt           = 0.001; % Time step
uav.tr.T            = 10; % Final time
uav.tr.IS_LINEAR    = 0; % Run fuzzy linear system or origin nonlinear system
uav.tr.IS_RK4       = 1; % Run RK4 or Euler method

ref = REF(uav);

%% find A, B (linearize)
if EXE.A_B
    uav.AB = TPmodel(uav.ABl);
    uav.save('A')
    uav.save('B')  
end 

%% find K, L
% let A more negtive
% for i = 1 : size(obj.A{1})
%     obj.A{i} = obj.A{i} - 0.05*eye(obj.dim);
% end
if EXE.LMI
    uav = uav.getKL(fz, ref);
    uav.save('K')
end

%% trajectory
if EXE.TRAJ
    uav = uav.trajectory(ref, fz);
    uav.save('tr');
end

if EXE.PLOT
    uav.tr.plot();
end

%% Execution time
toc

%% Controlability
% for i = 1 : fz.num
%     disp(['Rank of linear system ' num2str(i) ': ' num2str(rank(ctrb(uav.A{i}, uav.B{i})))])   
% end

%% Remove path
rmpath(genpath('function'))
rmpath(genpath('../../../src'))
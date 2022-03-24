%main script
% one UAV, fuzzy, reference model tracking control, no observer
clc; clear; close all; tic; % warning off
addpath(genpath('../../../src'))
addpath(genpath('function'))

%% M, C


%% trajectory
if EXE.TRAJ
    uav.tr.dt           = 0.001; % Time step
    uav.tr.T            = 10; % Final time
    uav.tr.x0           = [0.1 0 0.1 0.5 0.1 0.5 0.51 0.59 0.52 0.52 0.55 0.52]';
    uav.tr.xr0          = [0 1 0.5 0 0 0.8 0 0 0 0 0 0]';
    uav.tr.IS_LINEAR    = 0; % Run fuzzy linear system or origin nonlinear system
    uav.tr.IS_RK4       = 0; % Run RK4 or Euler method

    uav = uav.trajectory(fz);
    uav.Save('tr');
end

if EXE.PLOT
    uav.tr.plot();
end


%% Execution time
toc

%% Controlability

%% Debug

%% Remove path
rmpath(genpath('function'))
rmpath(genpath('../../../src'))

%% functions
function y = Ab(x)
    Ab = @(x) [A+B*K -B*K; O Ar];
end

function y = symmetric(x)
    y = x + x';
end
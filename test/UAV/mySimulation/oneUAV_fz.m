%main script
% one UAV, fuzzy, reference model tracking control, no observer
clc; clear; close all; tic; % warning off
addpath(genpath('../../../src'))
addpath(genpath('function'))

fz  = Fuzzy();
uav = UAV_FZmodel(fz);

%% find A, B (linearize)
if EXE.A_B
    % uav.fz. = ...
    % ...
    
    uav = uav.getAB(fz);    
    uav.Save('A')
    uav.Save('B')  
end 

%% find K, L
% let A more negtive
% for i = 1 : size(obj.A{1})
%     obj.A{i} = obj.A{i} - 0.05*eye(obj.dim);
% end
if EXE.LMI
    uav.rho             = 10^(1);
    uav.Q               = 10^(-1)*diag([1, 0.001, 1.5, 0.002, 1, 0.001, 0.1, 0, 0.1, 0, 1, 0.001]); % Correspond to x - xr
    uav.E               = 10^(-1)*diag([0 1 0 1 0 1 0 1 0 1 0 1]); % Disturbance matrix

    uav = uav.getKL(fz);
    uav.Save('K')
end

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

%% Eigenvalue of LMI
% eigOfLMI(uav, fz, ref, p);

%% Execution time
toc

%% Controlability
% for i = 1 : fz.num
%     disp(['Rank of linear system ' num2str(i) ': ' num2str(rank(ctrb(uav.A{i}, uav.B{i})))])   
% end

%% Debug
% Test if sum of membership function is 1
% sum = 0;
% x0 = rand(12,1);
% for j = 1 : fz.num
%     sum = sum + fz.mbfun(j, x0);
% end

%% Remove path
rmpath(genpath('function'))
rmpath(genpath('../../../src'))

%% functions
function y = Ab(x)
    Ab = @(x) [A+B*K -B*K; O Ar];
end

function eigOfLMI(uav, fz, ref, p)
I = eye(uav.dim);
O = zeros(uav.dim);
Qb = [p.Q -p.Q; -p.Q p.Q];
Fb = [I O; O ref.B];
% Ab = zeros(2*uav.dim, 2*uav.dim, fz.num);
% Bb = zeros(2*uav.dim, uav.dim_u, fz.num);
% Kb = zeros(uav.dim_u, 2*uav.dim, fz.num);
Mb = zeros(2*uav.dim, 2*uav.dim);

% calculate if LMI is positive definite
for i = 1 : fz.num
    Ab = [uav.A(:, :, i) O; O ref.A];
    Bb = [uav.B(:, :, i); zeros(uav.dim, uav.dim_u)];
    Kb = [uav.K(:, :, i) -uav.K(:, :, i)];
    Pb = [p.P1(:, :, i) O; O p.P2(:, :, i)];
    
    Mb = Mb + Qb + symmetric((Ab + Bb*Kb)'*Pb) + p.rho^(-2)*Pb*(Fb*Fb')*Pb;
end
disp('eig of LMI: ')
disp(max(eig(Mb)))
end

function y = symmetric(x)
    y = x + x';
end

% method 2 : https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7535919
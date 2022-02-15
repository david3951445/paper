%main script
% one UAV, fuzzy, reference model tracking control, no observer
clc; clear; close all; tic
addpath(genpath('function'))
addpath(genpath('../../../src'))

uav = UAV;
fz = Fuzzy;
ref = REF(uav);

% tunable parameter
% p : tf, dt, rho, Q, A, B, K, P1, P2
p.tf    = 2*pi;      % final time of trajectory
p.dt    = 0.002;     % time step of RK4
p.rho   = 1*10^(1);
p.Q     = 10^(-2)*diag([1, 0.001, 1, 0.001, 1, 0.001, 1, 0.001, 1, 0.001, 1, 0.001]);

%% linearize
if EXE.A_B
    [A, B] = getLocalMatrix(uav, fz);
    save('Matrix.mat', 'A', '-append')
    save('Matrix.mat', 'B', '-append')
else
    A = load('Matrix.mat').A;
    B = load('Matrix.mat').B;
end
p.A = A; p.B = B;
uav.A = p.A; uav.B = p.B;

%% find K
for i = 1 : size(A, 3)
    A(:, :, i) = A(:, :, i) - 0.05*eye(uav.dim);
end

if EXE.LMI
    pp = getControlGain2(uav, fz, ref, p);
    save('Matrix.mat', '-struct', 'pp', 'P1', '-append')
    save('Matrix.mat', '-struct', 'pp', 'P2', '-append')
    save('Matrix.mat', '-struct', 'pp', 'K', '-append')
else
    pp.P1 = load('Matrix.mat').P1;
    pp.P2 = load('Matrix.mat').P2;
    pp.K = load('Matrix.mat').K;
end
p.P1 = pp.P1; p.P2 = pp.P2; p.K = pp.K;
uav.K = p.K;

%% trajectory
if EXE.TRAJ
    tr = trajectory(uav, fz, ref, p);
end


%% plot
if EXE.PLOT
    Plot(tr)
end
% plot(tr.t, tr.r(7:12, :));
% legend

%% Calculate eigenvalue of LMI
% eigOfLMI(uav, fz, ref, p);

%% Calculate execution time
toc
rmpath(genpath('function'))
rmpath(genpath('../../../src'))

%% functions
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
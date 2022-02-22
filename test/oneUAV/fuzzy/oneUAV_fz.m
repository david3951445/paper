%main script
% one UAV, fuzzy, reference model tracking control, no observer
clc; clear; close all; tic; warning off
addpath(genpath('../../../src'))
addpath(genpath('function'))

fz  = Fuzzy();
uav = UAV(fz);
ref = REF(uav);

%% find K
% let A more negtive
% for i = 1 : size(uav.A{1})
%     uav.A{i} = uav.A{i} - 0.05*eye(uav.dim);
% end

if EXE.LMI
    uav.K = getControlGain2(fz, uav, ref);
    uav.save('data/uav.mat', 'K')
    % save('Matrix.mat', '-struct', 'pp', 'P1', '-append')
end
% pp.P1 = load('Matrix.mat').P1;

%% trajectory
if EXE.TRAJ
    tr = Trajectory(uav, ref, fz);
    % tr = trajectory2(uav, fz, ref, p);
end

%% plot
if EXE.PLOT
    tr.plot();
    % Plot(tr)
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
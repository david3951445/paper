clc; clear;close all
addpath(genpath('../../../src'))

%% reference
uav = UAV_zeroDynamics();
dt = 0.001; T = 10;
t = 0 : dt : T;
xd = cos(t);
yd = sin(t);
zd = t;
phid = zeros(1, length(t));
r = [xd; yd; zd; phid];
DIM_F2 = size(r, 1);
dr = [zeros(DIM_F2, 1) diff(r,1,2)/dt];
% ddr = [zeros(DIM_F2, 1) diff(dr,1,2)/dt];

%% system matrix
% A = [1 dt dt^2/2; 0 1 dt; 0 0 1];
% B = [dt^3/6; dt^2/2; dt];
A = [0 1 0; 0 0 1; 0 0 0];
B = [0; 0; 1];
% E = [0; 0; 1]; % disturbance matrix
DIM_F2 = size(r, 1);
A = kron(A, eye(DIM_F2)); 
B = kron(B, eye(DIM_F2)); 
% E = kron(E, eye(DIM_F2)); % disturbance matrix

%% control gain
Q = diag(1*[.1 100 10]); % I, P, D
Q = kron(Q, diag([1 1 1 1])); % x, y, z
R = [];
rho = 100;
K = solveLMI6(A,B,Q,R,rho);

norm(K)

%% plot
ind.I = (1:DIM_F2);
ind.P = DIM_F2+(1:DIM_F2);
ind.D = DIM_F2*2+(1:DIM_F2);

x = zeros(3*DIM_F2, length(t)); % error
x(ind.P, 1) = ones(DIM_F2, 1) - r(:, 1); % initial

ie = zeros(DIM_F2, 1);
e = zeros(DIM_F2, 1);
v = 1*randn;
v_old = zeros(DIM_F2, 1);

for i = 1 : length(t)-1
    y = x(DIM_F2+(1:DIM_F2*2), i);% + v; % output measurement
    ie = ie + e;
    e = y(1:DIM_F2) - r(:, 1);
    de = y(DIM_F2+(1:DIM_F2)) - dr(:, i);
    
%     x(:, i+1) = A*x(:, i) + B*K*[ie; e; de];
    x(:, i+1) = x(:, i) + (A*x(:, i) + B*K*x(:, i) + v)*dt;
end

TITLE = {'xd', 'yd', 'zd', 'phid'};
for i = 1 : DIM_F2
figure; hold on
title(TITLE{i})
plot(t, r(i, :), 'DisplayName', 'reference')
plot(t, x(DIM_F2+i, :), 'DisplayName', 'error')
plot(t, x(DIM_F2+i, :)+r(i, :), 'DisplayName', 'state')
legend
end
% plot(t, dxd)
%A simple linear system to test solveLMI()
clc; clear; close all;
addpath(genpath('../../src'))

I = eye(2); O = zeros(2);
A = [2 0; 1 1]; B = [1.5; 1]; E = 1*[1 0; 0 1]; Ar = -10*I; Br = -Ar;
[DIM_X, DIM_U] = size(B);
Q1 = 0; Q2 = 10^(0)*[1 0; 0 0.001]; rho = 10; R = 0;
dt = 0.001; T = 10; x0 = [0.1; 0.2]; xr0 = [1; 0];
freq = 1; amp = 1;
r = @(t)[amp*sin(freq*t); amp*freq*cos(freq*t)];

K2 = solveLMI1(A, B, E, Ar, Br, Q2, rho);
% K2 = solveLMI6(A, B, R, Q2, rho);
K = K2;
norm(K)

t = 0 : dt : T;
x = zeros(DIM_X, length(t)); 
xr = x;
x(:, 1) = x0; xr(:, 1) = xr0;
for i = 1 : length(t) - 1
    k = A*x(:, i) + B*K*(x(:, i) - xr(:, i));
    x(:, i+1) = x(:, i) + dt*k;
    k = Ar*xr(:, i+1) + Br*r(i*dt);
%     xr(:, i+1) = xr(:, i) + dt*k;
    xr(:, i+1) = r(i*dt);
end

plot(t, x, t, xr)
legend('x1', 'x2', 'xr1', 'xr2')

if rank(ctrb(A, B)) == size(A, 1)
    disp('controllable')
else
    disp('rank of system:')
    disp(rank(ctrb(A, B)))
end

rmpath(genpath('../../src'))
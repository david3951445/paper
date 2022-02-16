%A simple linear system to test solveLMI()
clc; clear; close all;
addpath(genpath('../../src'))

I = eye(2); O = zeros(2);
A = [0 1; 1 2]; B = [1; -1]; E = [1 0; 0 1]; Ar = -10*I; Br = -Ar;
Q1 = 0; Q2 = 10^(-3)*[1 0; 0 1]; rho = 1;
dt = 0.002; T = 20; x0 = [0.1; 0.2]; xr0 = [1; 0];
freq = 0.5;
v = @(t)[0.1*randn + 0; 0.1*randn + 0];
r = @(t)[sin(freq*t); freq*cos(freq*t)];

% K1 = solveLMI(A, B, E, Ar, Br, Q1, Q2, rho);
K2 = solveLMI1(A, B, E, Ar, Br, Q2, rho);
K = K2*100;

sysArg = LTI([A+B*K -B*K; O Ar], [E O; O Br], dt, T, [x0; xr0], @(t)[v(t); r(t)]);
sysRef = LTI(Ar, Br, dt, T, xr0, r);

sys = sysArg;
plot(sys.t, sys.x)
legend('x1', 'x2', 'xr1', 'xr2')

% rank(ctrb(A, B))

rmpath(genpath('../../src'))
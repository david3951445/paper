%A simple linear system to test solveLMI()
clc; clear; close all;
addpath(genpath('../../src'))

I = eye(2); O = zeros(2);
A = [0 1; 1 2]; B = [0; 1]; E = 1*[1 0; 0 1]; Ar = -10*I; Br = -Ar;
Q1 = 0; Q2 = 10^(0)*[1 0; 0 0.001]; rho = 3;
dt = 0.002; T = 10; x0 = [0.1; 0.2]; xr0 = [1; 0];
freq = 1; amp = 1;
v = @(t)[0.5*randn + 0; 0.5*randn + 0];
r = @(t)[amp*sin(freq*t); amp*freq*cos(freq*t)];

% K1 = solveLMI(A, B, E, Ar, Br, Q1, Q2, rho);
K2 = solveLMI1(A, B, E, Ar, Br, Q2, rho);
K = K2*1;

Ab = @(x) [A+B*K -B*K; O Ar];
Eb = [E O; O Br];
xb0 = [x0; xr0];
vb = @(t)[v(t); r(t)];
sysArg = LTI(Ab, Eb, dt, T, xb0, vb);
% sysRef = LTI(Ar, Br, dt, T, xr0, r);

sys = sysArg;
plot(sys.t, sys.x)
legend('x1', 'x2', 'xr1', 'xr2')

if rank(ctrb(A, B)) == size(A, 1)
    disp('controllable')
end

rmpath(genpath('../../src'))
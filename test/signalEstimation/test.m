% fault signal estimation, no smooth model, i.e.,
% dx/dt = A*x + v -> dx/dt = 0*x + dx
clc; clear; close all
addpath(genpath('../../src'))

dt = .001; t = 0 : dt : 10;
A = [0 1 0; 0 0 1; 0 0 0]; B = [0; 0; 1]; C = eye(3);
[DIM_X, DIM_U] = size(B);
[DIM_Y, ~] = size(C);
DIM_F = 1;

WINDOW = 1;
Ab = [A B*1 zeros(DIM_X, 1); zeros(1, DIM_X) 0 0; zeros(1, DIM_X+2)];
Bb = [B; 0; 0];
Cb = [C zeros(DIM_Y, 1) ones(DIM_Y, 1)];
DIM_X3 = size(Ab, 1);
DIM_Ys = 1;

I = eye(DIM_X);
E = zeros(DIM_X*DIM_F+DIM_F+DIM_F*DIM_Ys);
Q1 = 10^(3)*diag([1 1 1 1 1]);
Q2 = 10^(3)*diag([1 1 1 1 1]);
R = [];
rho = 1;
[K, L] = solveLMI10(Ab, Bb, Cb, E, Q1, Q2, R, rho);

%% traj
x = zeros(DIM_X3, length(t));
xh = zeros(DIM_X3, length(t));
x(:, 1) = [1 1 1 0 0]';

v1 = 0.1*ones(DIM_F, length(t)) + 0*cos(t);
v2 = 0.2*ones(DIM_F*DIM_Ys, length(t));

fun = @(t, p) [Ab, Bb*K; -L*Cb, Ab+Bb*K+L*Cb]*p;
for i = 1 : length(t) - 1
    xb = [x(:, i); xh(:, i)];
    xb = ODE_solver(fun, dt, xb, t(i), 'RK4');
    x(:, i+1) = xb(1:DIM_X3);
    xh(:, i+1) = xb(DIM_X3+(1:DIM_X3));

    x(DIM_X*DIM_F+(1:DIM_F), i+1) = v1(:, i);
    x(DIM_F*(DIM_X+WINDOW)+(1:DIM_F*DIM_Ys), i+1) = v2(:, i);
end

%% plot
for i = 1 : DIM_X3
figure; hold on
plot(t, x(i, :), 'DisplayName', 'state')
plot(t, xh(i, :), 'DisplayName', 'estimate state')
legend
grid on
end
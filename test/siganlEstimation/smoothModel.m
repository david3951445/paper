%Test smooth model performance
clc; clear; close all
addpath(genpath('../../src'))

% system
A = -1; C = 1;
[DIM_U, DIM_X] = size(C);
I = eye(DIM_X);

% smooth model
WINDOW = 2;
DIM_X2 = DIM_X*WINDOW;
Aa = kron([1 -1; 1 -1], I);
Ca = kron([1 0], I);

% augment sys
Ab = [A Ca; zeros(DIM_X2, DIM_X) Aa];
Cb = [C zeros(DIM_U, DIM_X2)];
DIM_X3 = size(Ab, 1);

Q = kron(I, diag([1 1 1])); rho = 1;

L = solveLMI7(Ab, Cb, Q, rho);

dt = 0.001; T = 10; t = 0 : dt : T;
x = zeros(DIM_X3, length(t));
xh = zeros(DIM_X3, length(t));

x(:, 1) = [1; zeros(DIM_X2, 1)];
w = randn(1, length(t));
v = 3*cos(t) - 0.1*t + 0.01*t.^2;
dvdt = -3*sin(t) -0.1 + 0.02*t;
dvdt_withInit = [zeros(1, WINDOW), dvdt];
L = 5;

for i = 1 : length(t) - 1
    i_v_withInit = i + WINDOW;
    k = [
        [A Ca]*x(:, i)
        dvdt_withInit(i_v_withInit-1 : -1 : i_v_withInit-WINDOW)'
    ];
    x(:, i+1) = x(:, i) + dt*k;

    k = Ab*xh(:, i) + L*Cb*(x(:, i) - xh(:, i));
    xh(:, i+1) = xh(:, i) + dt*k;
end

TITLE = {'x', 'v(t)', 'v(t-h)'};
for i = 1 : 3
    subplot(3, 1, i)
    hold on
    plot(t, x(i, :), 'Displayname', 'state')
    plot(t, xh(i, :), 'DisplayName', 'estimated')
    hold off
    legend
    title(TITLE{i})
end
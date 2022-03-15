%Test smooth model performance
clc; clear; close all
addpath(genpath('../../src'))

A = -1;
DIM_X = size(A, 1);
dt = 0.001; T = 10; t = 0 : dt : T;

x = zeros(DIM_X, length(t));
xh = x;
x(:, 1) = 1;
w = randn(1, length(t));
v = 3*cos(t) + 0.1*w - 0.1*t + 0.01*t.^2;

L = 5;

for i = 1 : length(t) - 1  

    k = A*x(:, i) + v(i);
    x(:, i+1) = x(:, i) + dt*k;

    k = A*xh(:, i) + L*(x(:, i) - xh(:, i));
    xh(:, i+1) = xh(:, i) + dt*k;
end

subplot(2, 1, 1)
plot(t, x, t, xh)
legend('x', 'xh')
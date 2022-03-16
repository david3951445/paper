clc; clear; close all

A = -1; C = 1; L = 5;
[DIM_U, DIM_X] = size(C);
I = eye(DIM_X);
dt = 0.001; T = 10; t = 0 : dt : T;
x = zeros(DIM_X, length(t));
xh = x;
x(:, 1) = 1;
x_n = x;
v = cos(t);
for i = 1 : length(t)-1
    k = A*x(:, i);
    x_n(:, i+1) = x_n(:, i) + dt*k; 
    k = A*x(:, i) + v(i);
    x(:, i+1) = x(:, i) + dt*k;
    k = A*xh(:, i) + L*C*(x(:, i) - xh(:, i));
    xh(:, i+1) = xh(:, i) + dt*k;
     
end
% plot(t, x, t, xh, t, x_n)
plot(t, v, t, x - x_n)
legend({'1','2'})
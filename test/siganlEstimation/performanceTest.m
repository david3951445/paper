%Test if estimate a non-bounded disturbance is helpful for performance
% Here use polynomial fitting to approximate a unknown signal base on the Prior Knowledge (past data points).
% After the approximation, signal will be seperated to a known siganl and a "smaller" signal.
% The known signal can be eliminated by an adaptive law.
% The "smaller" signal will be attenuated by H infinity robust control

clc; clear; close all
addpath(genpath('../../src'))

I = eye(2); O = zeros(2);
A = [0 1; 2 1]; B = [0; 1];
[DIM_X, DIM_U] = size(B);
R = 0; Q = [1 0; 0 0.001]; rho = 1;
dt = 0.001; T = 10; t = 0 : dt : T;

K = solveLMI6(A, B, [], Q, rho);

x = zeros(DIM_X, length(t));
x(:, 1) = [0; -0];
w = randn(1, length(t));
v = 3*cos(t) + 0.1*w - 0.1*t + 0.01*t.^2;

window = 100;
y_withInit = [zeros(1, window), v];
for i = 1 : length(t) - 1  
    i_y_withInit = i + window;
    y_cur = y_withInit(i_y_withInit - window : i_y_withInit - 1);

    p = polyfit(t(1:window), y_cur, 2);   
    vh = polyval(p, t(window + 1));

    e(i+1) = v(i) - vh;
    k = A*x(:, i) + B*K*(x(:, i)) + [0; v(i) - vh];
    x(:, i+1) = x(:, i) + dt*k;
end

subplot(2, 1, 1)
plot(t, x)
legend('x1', 'x2')
% ylim([-4 4])
subplot(2, 1, 2)
plot(t, v, t, e)
legend('v', 'e')

disp(['err = ' num2str(norm(e))])

rmpath(genpath('../../src'))
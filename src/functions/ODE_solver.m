%Runge-Kutta-Fehlberg method
% dx(t)/dt = f(t, x)
%
% input:
% f     - dx(t)/dt
% h     - time step
%
% output:
% y     - x(t+h)

function y = ODE_solver(f, h, x, t, method)
switch method
case 'Euler'
k = h*f(t, x);
y = x + k;

case 'RK4'
CH = [1/6 2/6 2/6 1/6];
k = zeros(length(x), 4);
k(:, 1) = h*f(t, x);
k(:, 2) = h*f(t, x+k(:, 1)/2);
k(:, 3) = h*f(t, x+k(:, 2)/2);
k(:, 4) = h*f(t, x+k(:, 3));
y = x + k*CH';

case 'RKF'
% table 1
A = [0 2/9 1/3 3/4 1 5/6];
B = [
    0 0 0 0 0
    2/9 0 0 0 0
    1/12 1/4 0 0 0
    69/128 -243/128 135/64 0 0
    -17/12 27/4 -27/5 16/15 0
    65/432 -5/16 13/16 4/27 5/144
];
CH = [47/450 0 12/25 32/225 1/30 6/25];
CT = [-1/150 0 3/100 -16/75 -1/20 6/25];

% % table 2
% A = [0 1/2 1/2 1 2/3 1/5];
% B = [
%     0 0 0 0 0
%     1/2 0 0 0 0
%     1/4 1/4 0 0 0
%     0 -1 2 0 0
%     7/27 10/27 0 1/27 0
%     1/5 28/625 -1/5 546/625 54/625 -378/625
% ];
% CH = [1/24 0 0 5/48 27/56 125/336]

k = zeros(length(x), 6);
k(:, 1) = h*f(t+A(1)*h, x);
for i = 2 : 6
    k(:, i) = h*f(t+A(i)*h, x+k(:, 1:i-1)*B(i,1:i-1)');
end
y = x + k*CH';
% TE = norm(k*CT')
end

end


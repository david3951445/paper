% linear multivariate regression algorithm
% point : point to be linearlize
% num : location of regression parameters
% fun : orresponding nonlinear function
% cost : for i = 1 : N { (fun(x_i) - y*x_i)^2 }
function y = regress_2(point, loc, fun)
range = [1 1 1 1]*10^(-5); 
N = 100; % # of sample point
rng(0)

num = length(loc);
xx = zeros(length(point), N);
x = zeros(num, N);
for i = 1 : num
%     x(i, :) = linspace(-range(i) + set(i), range(i) + set(i), N);
    a = -range(i) + point(i); c = range(i) + point(i);
    x(i, :) = a + (c-a).*rand(1,N);
end
xx(loc, :) = x;

% find y
S = zeros(num); b = zeros(num, 1);
for i = 1 : N
     b = b+ x(:, i)*fun(xx(:, i));
     S = S + x(:, i)*x(:, i)';
end

y = S\b;
end
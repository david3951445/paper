% linear multivariate regression algorithm

function y = regress_2(set, f, m1, m2, l1, l2, g)
num = length(set); % # of regression parameter
range = [1 1 1 1]*10^(-5); 
N = 100; % # of sample point
rng(0)

x = zeros(num, N);
for i = 1 : num
%     x(i, :) = linspace(-range(i) + set(i), range(i) + set(i), N);
    a = -range(i) + set(i); c = range(i) + set(i);
    x(i, :) = a + (c-a).*rand(1,N);
end

S = zeros(num); b = zeros(num, 1);
for i = 1 : N
     b = b+ x(:, i)*f(x(:, i), m1, m2, l1, l2, g);
     S = S + x(:, i)*x(:, i)';
end

% c1 = Sx2x2/Sx1x2
% c2= Sx1x2/Sx1x1
% Sb = S([1 3], [1 3]);
% bb = b([1 3]);

y = S\b;
end
%Numerical differentiation
% Numerical differentiation of a vector sequence. Different from MATLAB
% diff(), return same sequence length N, not N-1.
% 
% input:
% x     - M by N matrix, M is number of component of x, N is sequence length
%
% output:
% y     - differentiation of x

function y = my_diff(x)
N = length(x);
if N < 2
    err('2*nd dimension of input must larger than 2')
end

y(:, 1) = x(:, 2) - x(:, 1); % forward difference
y(:, N) = x(:, N) - x(:, N-1); % backward difference
for i = 2 : N-1
    y(:, i) = (x(:, i+1) - x(:, i-1))/2; % center difference
end
end
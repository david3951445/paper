%Find Coefficient of Finite Difference Method
%
% Input:
%   point
%   order
%
% Output:
%   coeff
%
% Ex:
%   Find the first-order derivative of a function f(x) constructed by f(x-2), f(x), f(x + 1) and their coefficient c_(-2), c_0, c_1
%   f'(x) = c_(-2)*f(x-2) + c_0*f(x) + c_1*f(x + 1)
%   then
%       point = [-2 0 1]
%       order = 1

function coeff = FindFDC(point, order)

n = length(point);
if order >= n
    error('Order of the derivative needs less than number of points')
end

A = zeros(n);
for i = 1 : n
    for j = 1 : n
        A(j, i) = point(i)^(j-1)/factorial(j-1);
    end
end
b = zeros(n, 1); b(order + 1) = 1;

coeff = A\b;
end

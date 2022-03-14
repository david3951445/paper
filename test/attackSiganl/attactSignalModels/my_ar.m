function x = my_ar(y, n)
%my ar()
% min e, e = Ax-b
l = length(y);

A = zeros(l-n, n);
b = zeros(l-n, 1);
for i = n+1 : l
    A(i-n, :) = y(i-1:-1:i-n);
    b(i-n) = y(i);
end
% x = A\b;
x = lsqr(A, b);
end


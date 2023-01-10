function obj = getA_CT(obj)
%CONSTRUCT SMooth Model's A matrix
%
% method 1-1    - [FDC; 1- 1 0 ... 0; 0 1 -1 0 ... 0; ...]
% method 1-2    - [1 -1 0 ... 0; 1- 1 0 ... 0; 0 1 -1 0 ... 0; ...]
% method 1-3    - [a1-1 a2 ... an; 1- 1 0 ... 0; 0 1 -1 0 ... 0; ...], sum of "a1" to "an" = 1
% method 2      - [FDC]
% method 3      - zeros
% method 4      - with second order differential term
% method 5      - with integral term
%
% - FDC: Finite Difference Coefficient

WINDOW  = obj.WINDOW;
h      = obj.dt;
method  = obj.METHOD;
A       = zeros(WINDOW); % init

switch method
case '1-1' % finite differenciation
    point = 0 : -1 : -WINDOW+1;
    A(1, 1:WINDOW) = FindFDC(point, 1)'/h;
case '1-2' % 1 -1
    point = zeros(WINDOW, 1); point(1:2) = [1 -1];
    A(1, 1:WINDOW) = point/h;
case '1-3' % origin smooth coefficient
    coeff2 = 0.3.^(0:WINDOW-1);
    coeff2 = coeff2/sum(coeff2); % sum of coeff must be 1
    A(1, 1:WINDOW) = [-1 zeros(1, WINDOW-1)];
    A(1, 1:WINDOW) = (A(1, 1:WINDOW) + coeff2)/h;
end

switch method
case {'1-1', '1-2', '1-3'} % different in first column
    for i = 2 : WINDOW
        point = [1 0];
        A(i, i-1:i) = FindFDC(point, 1)'/h; % obtain coefficient
    end
case '2' % finite differenciation
    for i = 1 : WINDOW
        point = i-1 : -1 : -WINDOW+i;
        A(i, :) = FindFDC(point, 1)'/h;
    end
case '3' % zero    
case '4'
    A = [
        0 0 1 0
        0 0 0 1
        -6/h^2 6/h^2 6/h 2/h
        6/h^2 -6/h^2 -2/h -6/h
    ]; % test for WINDOW = 4 (trajectory there need to change manually too)
case '5'
    A = [
        1 -1 0 0
        1 -1 0 0
        % 6/h 2/h -6/h^2 6/h^2
        % -2/h -6/h 6/h^2 -6/h^2
        1 0 0 0
        0 1 0 0
    ]; % test for WINDOW = 4
otherwise
    error('no such method in getA_CT()')
end

%% extend to correct dimension
obj.A = kron(A, eye(obj.DIM));
end


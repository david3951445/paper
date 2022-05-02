function obj = getA_CT(obj)
%CONSTRUCT SMooth Model's A matrix

WINDOW  = obj.WINDOW;
dt      = obj.dt;
method  = obj.METHOD;
A       = zeros(WINDOW); % init

switch method
case '1-1'
    point = 0 : -1 : -WINDOW+1;
    A(1, 1:WINDOW) = FindFDC(point, 1)'/dt;
case '1-2'
    point = zeros(WINDOW, 1); point(1:2) = [1 -1];
    A(1, 1:WINDOW) = point/dt;
case '1-3'
    coeff2 = 0.5.^(0:WINDOW-1);
    coeff2 = coeff2/sum(coeff2);
    A(1, 1:WINDOW) = [-1 zeros(1, WINDOW-1)];
    A(1, 1:WINDOW) = (A(1, 1:WINDOW) + coeff2)/dt;
end

switch method
case {'1-1', '1-2', '1-3'}
    for i = 2 : WINDOW
        point = [1 0];
        A(i, i-1:i) = FindFDC(point, 1)'/dt; % obtain coefficient
    end
case '2'
    for i = 1 : WINDOW
    point = i-1 : -1 : -WINDOW+i;
    A(i, :) = FindFDC(point, 1)'/dt;
    end
case '3'

otherwise
    error('no such method in getA_CT()')
end

%% extend to correct dimension
obj.A = kron(A, eye(obj.DIM));
end


function obj = getA_CT(obj)
%CONSTRUCT SMooth Model's A matrix

WINDOW = obj.WINDOW;
dt = obj.dt;
A = zeros(WINDOW);

%% Method 1
coeff2 = 0.1.^(0:WINDOW-1);
A(1, 1:WINDOW) = coeff2/sum(coeff2);
A(2:WINDOW, 1:WINDOW-1) = eye(WINDOW-1);

%% extend to correct dimension
obj.A = kron(A, eye(obj.DIM));;
end


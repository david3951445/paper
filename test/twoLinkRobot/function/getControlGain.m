function K = getControlGain(fz, uav, Ar, Br)
%YALMIP method

% Hinf performance. Q, R, rho
rho   = 1*10^(2);
Q     = 10^(-2)*diag([1, 0.001, 1, 0.001, 1, 0.001, 0.1, 0, 0.1, 0, 1, 0.001]); % correspond to x - xr
E = 1*10^(0)*diag([0 1 0 1 0 1 0 1 0 1 0 1]); % disturbance matrix
O = zeros(12);  

LEN     = fz.num;
DIM_X   = uav.dim;
DIM_U   = uav.dim_u;
% Q1      = 0; % p.Q*10^(-3); % correspond to x (Q1 = 0 theoretically, if you want to set it to zero, remove M33)

K = cell(1, LEN);
for i = 1 : LEN
    fprintf('LMI iter: %d/%d\n', i, LEN)
    A = uav.A{i}; B = uav.B{i};

    K{i} = solveLMI1(A, B, E, Ar, Br, Q, rho);
end

%% local function
    function y = addSym(x)
        y = x + x';
    end
end
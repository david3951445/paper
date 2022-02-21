function K = getControlGain2(fz, uav, ref)
%YALMIP method

% Hinf performance. Q, R, rho
rho   = 10^(1);
Q     = 10^(-1)*diag([1, 0.001, 1.5, 0.002, 1, 0.001, 0.1, 0, 0.1, 0, 1, 0.001]); % correspond to x - xr
E = 10^(-1)*diag([0 1 0 1 0 1 0 1 0 1 0 1]); % disturbance matrix
O = zeros(12);  

LEN     = fz.num;
DIM_X   = uav.dim;
DIM_U   = uav.dim_u;
Ar      = ref.A;
Br      = ref.B;
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
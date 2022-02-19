function K = getControlGain2(fz, uav, ref, p, m)
%YALMIP method
%

LEN     = fz.num;
DIM_X   = uav.dim;
E       = uav.E;
DIM_U   = uav.dim_u;
Ar      = ref.A;
Br      = ref.B;
rho     = p.rho;
Q2      = p.Q; % correspond to x - xr
% Q1      = 0; % p.Q*10^(-3); % correspond to x (Q1 = 0 theoretically, if you want to set it to zero, remove M33)

K = cell(1, LEN);
for i = 1 : LEN
    fprintf('LMI iter: %d/%d\n', i, LEN)
    A = uav.A{i}; B = uav.B{i};

    K{i} = solveLMI2(A, B, E, Ar, Br, Q2, rho);
end

%% local function
    function y = addSym(x)
        y = x + x';
    end
end
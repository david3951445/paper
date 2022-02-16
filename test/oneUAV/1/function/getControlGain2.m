function m = getControlGain2(uav, fz, ref, p, m)
%YALMIP (spdvar method)
DIM_X   = uav.dim;
DIM_U   = uav.dim_u;
LEN     = fz.num;
I       = eye(DIM_X);
rho     = p.rho;
Q2      = p.Q; % correspond to x - xr
Q1      = 0; % p.Q*10^(-3); % correspond to x (Q1 = 0 theoretically, if you want to set it to zero, remove M33)
Ar      = ref.A;
Br      = ref.B;

for i = 1 : LEN
    fprintf('LMI iter: %d/%d\n', i, LEN)
    A = uav.A(:, :, i); B = uav.B(:, :, i);
    
    m.K(:, :, i) = solveLMI(A, B, I, Ar, Br, Q1, Q2, rho);
end

%% local function
    function y = addSym(x)
        y = x + x';
    end
end
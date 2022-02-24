function uav = getKL(uav)
%Solve LMI, YALMIP method

O = zeros(12);

LEN     = uav.AB.len;
DIM_X = uav.DIM_X;
DIM_U = uav.DIM_U;
% Q1      = 0; % p.Q*10^(-3); % correspond to x (Q1 = 0 theoretically, if you want to set it to zero, remove M33)

uav.K = cell(1, LEN);
for i = 1 : LEN
    fprintf('LMI iter: %d/%d\n', i, LEN)
    A = uav.AB.val{i}(:, 1 : DIM_X);
    B = uav.AB.val{i}(:, DIM_X+1 : DIM_X+DIM_U);
    
    % let A more negtive
    % uav.A{i} = uav.A{i} - 0.05*eye(obj.dim);
    uav.K{i} = solveLMI1(A, B, uav.E, uav.Ar, uav.Br, uav.Q, uav.rho);
end

end
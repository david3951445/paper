function uav = getKL(uav, fz, ref)
%Solve LMI, YALMIP method

O = zeros(12);  

LEN     = fz.num;
Ar      = ref.A;
Br      = ref.B;
% Q1      = 0; % p.Q*10^(-3); % correspond to x (Q1 = 0 theoretically, if you want to set it to zero, remove M33)

uav.K = cell(1, LEN);
for i = 1 : LEN
    fprintf('LMI iter: %d/%d\n', i, LEN)
    A = uav.A{i}; B = uav.B{i};

    uav.K{i} = solveLMI1(A, B, uav.E, Ar, Br, uav.Q, uav.rho);
end

end
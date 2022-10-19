function K = solveLMI1(A, B, E, Ar, Br, Q, rho)
%solution of "Qb + Pb(Ab+BbKb) + (Ab+BbKb)'Pb + PbEbEb'Pb/rho^2 < 0, Pb > 0"
%
% This function is used to solve a control problem defined below :
% (1) system
%       dx/dt = Ax + Bu + Ev
% (2) reference model
%       dxr/dt = Arx + Bru
% (3) augment system
%       dxb/dt = Abxb + Bbu + Ebvb
%       where xb = [x; xr], vb = [v; r]
%             Ab = [A 0; 0 Ar], Bb = [B; 0], Eb = [E 0; 0 Br]
% (4) control law
%       u = K(x-xr) = Kbxb
%       where Kb = [K -K]
% (5) H infinity performance
%       xb'Qbxb / vb'vb < rho^2
%       where Qb = [Q -Q; -Q Q]
% (6) Lyapunov function
%       V(x) = xb'Pbxb
%       where Pb = [P1 0; 0 P2]
%
% Given (1) ~ (6), form H infinity theorem, if
%       "Qb + Pb(Ab+BbKb) + (Ab+BbKb)'Pb + PbEbEb'Pb/rho^2 < 0, Pb > 0"
% hold then (3) achieve (5).

%% tunable parameter
d1 = 0*10^(-4); % LMI <= d1*I. if problem infeasible, try increasing d1

%% solve
[DIM_X, DIM_U]  = size(B);
O               = zeros(DIM_X);

options = sdpsettings('solver', 'sdpt3');
options = sdpsettings(options,'verbose', 0);
eqn = [];

W1 = sdpvar(DIM_X, DIM_X); % symmetric
Y1 = sdpvar(DIM_U, DIM_X); % full
eqn = [eqn, W1 >= 0];

M11 = addSym(A*W1 + B*Y1) + rho^(-2)*E*E';

M12 = W1*sqrtm(Q);
M22 = -eye(DIM_X);

LMI = [
    M11  M12
    M12' M22
];
eqn = [eqn, LMI <= d1*eye(2*DIM_X)];
% eq1 = M11 <= 0;
    
% If you want to limit size of K
% LMI2 = [
%     -10^(4)*eye(DIM_U)   Y2
%     Y2'                 -eye(DIM_X)
% ];
% eqn = [eqn, LMI2 <= 0];


sol = optimize(eqn, [], options);

% If you want to see solution information
% if sol.problem % Solve failed
    sol.info
% end
    
W1 = value(W1);
Y1 = value(Y1);

% P = eye(DIM_X)/W;
K = Y1/W1;

%% local function
function y = addSym(x)
    y = x + x';
end

end
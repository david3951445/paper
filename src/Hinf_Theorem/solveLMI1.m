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
%       where Pb = [P 0; 0 P]
%
% Given (1) ~ (6), form H infinity theorem, if
%       "Qb + Pb(Ab+BbKb) + (Ab+BbKb)'Pb + PbEbEb'Pb/rho^2 < 0, Pb > 0"
% hold then (3) achieve (5).

%% tunable parameter
d1 = 0*10^(-4); % LMI <= d1*I. if problem infeasible, try increasing d1

%% solve
[DIM_X, DIM_U]  = size(B);
O               = zeros(DIM_X);
EEBB            = E*E' + Br*Br';

options = sdpsettings('solver','sdpt3');
options = sdpsettings(options,'verbose', 0);

W = sdpvar(DIM_X, DIM_X); % symmetric
Y = sdpvar(DIM_U, DIM_X); % full
    
M11 = addSym(A*W + B*Y) + rho^(-2)*EEBB;

M12 = -B*Y - rho^(-2)*EEBB;
M22 = addSym(Ar*W) + rho^(-2)*EEBB;

M13 = W;
M23 = O;
M33 = -inv(2*Q);

M14 = O;
M24 = W;
M34 = O;
M44 = -inv(2*Q);

LMI = [
    M11  M12  M13  M14
    M12' M22  M23  M24
    M13' M23' M33  M34
    M14' M24' M34' M44
];
eq1 = LMI <= d1*eye(4*DIM_X)

% LMI = [
%     M11  M12
%     M12' M22
% ];
% eq1 = LMI <= 0;
    
% If you want to limit size of K
% LMI2 = [
%     -10^(4)*eye(DIM_U)   Y2
%     Y2'                 -eye(DIM_X)
% ];
% eqns = [LMI <= 10^(-4)*eye(4*DIM_X), LMI2 <= 0, W1 >= 0, W2 >= 0];

eqns = [eq1, W >= 0];
sol = optimize(eqns, [], options);

% If you want to see solution information
% if sol.problem % Solve failed
    sol.info
% end
    
W = value(W);
Y = value(Y);

% P = eye(DIM_X)/W;
K = Y/W;

%% local function
function y = addSym(x)
    y = x + x';
end

end
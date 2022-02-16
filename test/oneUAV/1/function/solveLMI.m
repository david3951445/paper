function K = solveLMI(A, B, E, Ar, Br, Q1, Q2, rho)
%solution of "Qb + Pb(Ab+BbKb) + (Ab+BbKb)'Pb + PbEbEb'Pb/rho^2 < 0, P1 > 0, P2 > 0"
%
% This function is used to solve a control problem defined below :
% (1) system
%       dx/dt = Ax + Bu + Ev
% (2) reference model
%       dxr/dt = Arx + Bru
% (3) augment system
%       dxb/dt = Abxb + Bbu + Ebvb
%       where xb = [x; x-xr], vb = [v; r]
%             Ab = [A 0; A-Ar Ar], Bb = [B; B], Eb = [E 0; E -Br]
% (4) control law
%       u = K(x-xr) = Kbxb
%       where Kb = [0 K]
% (5) H infinity performance
%       xb'Qbxb / vb'vb < rho^2
%       where Qb = [Q1 0; 0 Q2]
% (6) Lyapunov function
%       V(x) = xb'Pbxb
%       where Pb = [P1 0; 0 P2]
%
% Given (1) ~ (6), form H infinity theorem, if
%       "Qb + Pb(Ab+BbKb) + (Ab+BbKb)'Pb + PbEbEb'Pb/rho^2 < 0, P1 > 0, P2 > 0"
% hold then (3) achieve (5).

%% tunable parameter
d1 = 10^(-4); % LMI <= d1*I. if problem infeasible, try increasing d1

%% solve
[DIM_X, DIM_U] = size(B);

options = sdpsettings('solver','sdpt3');
options = sdpsettings(options,'verbose', 0);

W1 = sdpvar(DIM_X, DIM_X); % symmetric
W2 = sdpvar(DIM_X, DIM_X);
Y2 = sdpvar(DIM_U, DIM_X); % full
    
M11 = addSym(A*W1) + rho^(-2)*E*E';

M12 = B*Y2 + W1*(A-Ar)' + rho^(-2)*E*E';
M22 = addSym(Ar*W2 + B*Y2) + rho^(-2)*(E*E' + Br*Br');

M13 = zeros(DIM_X);
M23 = W2;
M33 = -inv(Q2);

if Q1 == 0
    LMI = [
        M11  M12  M13
        M12' M22  M23
        M13' M23' M33
    ];

    eq1 = LMI <= d1*eye(3*DIM_X)
else
    M14 = W1;
    M24 = zeros(DIM_X);
    M34 = zeros(DIM_X);
    M44 = -inv(Q1);

    LMI = [
        M11  M12  M13  M14
        M12' M22  M23  M24
        M13' M23' M33  M34
        M14' M24' M34' M44
    ];

    eq1 = LMI <= d1*eye(4*DIM_X)
end
    
% If you want to limit size of K
% LMI2 = [
%     -10^(4)*eye(DIM_U)   Y2
%     Y2'                 -eye(DIM_X)
% ];
% eqns = [LMI <= 10^(-4)*eye(4*DIM_X), LMI2 <= 0, W1 >= 0, W2 >= 0];

eqns = [eq1, W1 >= 0, W2 >= 0];
sol = optimize(eqns, [], options);

% If you want to see solution information
% if sol.problem
    sol.info
% end
    
W1 = value(W1);
W2 = value(W2);
Y2 = value(Y2);

P1 = eye(DIM_X)/W1;
P2 = eye(DIM_X)/W2;
K = Y2*P2;

%% local function
function y = addSym(x)
    y = x + x';
end

end
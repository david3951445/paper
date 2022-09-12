function [K, L] = solveLMI13(A, B, C, E, Q1, Q2, R, rho)
%solution of "Qb + Pb*Ab + Ab'*Pb + Kb'RKb + Pb*Pb/rho^2 < 0, Pb > 0"
%
% This function is used to solve a control problem defined below:
% (1) system
%       dxb/dt = Ab*xb + vb
%       where xb = [x; x-xh], Ab = [A+B*K -B*K; 0 A+L*C]
% (4) control law
%       u = Kb*xb, Kb = [K -K];
% (5) H infinity performance
%       xb'Qbxb + u'Ru/ vb'*vb < rho^2
%       Qb = [Q1 0; 0 Q2]
% (6) Lyapunov function
%       V(x) = xb'Pb*xb
%       where Pb = [P1 0; 0 P2]
% Infeasible, Kb'RKb couple, need two-step method
%% solve
[DIM_X, DIM_U]  = size(B);
[DIM_Y, ~]      = size(C);
O               = zeros(DIM_X);
I               = eye(DIM_X);
options = sdpsettings('solver', 'mosek');
options = sdpsettings(options,'verbose', 0);
eqn = [];

W1 = sdpvar(DIM_X, DIM_X);
P2 = sdpvar(DIM_X, DIM_X);
Y1 = sdpvar(DIM_U, DIM_X);
Y2 = sdpvar(DIM_X, DIM_Y);
Z  = sdpvar(DIM_U, DIM_U);

eqn = [eqn, W1 >= 0, P2 >= 0];

M11 = addSym(A*W1 + B*Y1) + rho^(-2)*E*E';

M12 = -B*Y1;
M22 = addSym(P2*A + Y2*C) + Q2 + 2*Z;

M13 = W1*sqrt(Q1);
M23 = O;
M33 = -I;

M14 = O;
M24 = P2*E*rho^(-1);
M34 = O;
M44 = -I;

M15 = I/W1;
M25 = zeros(DIM_X, DIM_U);
M35 = zeros(DIM_X, DIM_U);
M45 = zeros(DIM_X, DIM_U);
M55 = -eye(DIM_U)/Z/2;

LMI = [
    M11  M12  M13  M14  M15
    M12' M22  M23  M24  M25
    M13' M23' M33  M34  M35
    M14' M24' M34' M44  M45
    M15' M25' M35' M45' M55
];

eqn = [eqn, LMI <= 0];

LMI = [
    -Z  Y1'
    Y1  -R
];

eqn = [eqn, LMI <= 0];

sol = optimize(eqn, [], options);

% If you want to see solution information
% if sol.problem % Solve failed
    sol.info
% end
    
W1 = value(W1);
P2 = value(P2);
Y1 = value(Y1);
Y2 = value(Y2);

K = Y1/W1;
L = P2\Y2;

% Check eig value
% P1 = I/W1;
% Pb = [P1 O; O P2];
% Ab = [A+B*K -B*K; O A+L*C];
% eig(Pb*Ab + Ab'*Pb)

end

%% local function
function y = addSym(x)
    y = x + x';
end
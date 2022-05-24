function [K, L] = solveLMI12(sys, sys1, sys2)
%solution of "Qb + Pb*Ab + Ab'*Pb + Pb*Eb*Eb'*Pb/rho^2 < 0, Pb > 0"
%
% This function is used to solve a control problem defined below :
% (1) system
%       dxb/dt = Ab*xb + Eb*vb
%       xb = [x; x-xh], Ab = [A+B*K -B*K; 0 A+L*C]
%       Eb = [Ebs 0; 0 Ebe]
%       Ebs = blkdiag(0, Es1, Es2), Ebe = blkdiag(0, Ee1, Ee2)
% (4) control law
%       u = K*xh
%       K = [K1 Ka Ks], Ka = [-I 0 ... 0]
% (5) H infinity performance
%       (x-xh)'*Q2*(x-xh) + x*Q1*x / vb'*vb < rho^2
%       Qb = [Qbs 0; 0 Qbe]
%       Qbs = blkdiag(Qs, Qs1, Qs2), Qbe = blkdiag(Qe, Qe1, Qe2)
% (6) Lyapunov function
%       V(x) = xb'Pb*xb
%       Pb = [Pbs 0; 0 Pbe]
%       Pbs = blkdiag(Ps, Ps1, Ps2), Qbe = blkdiag(Pe, Pe1, Pe2)
%
% Since actuator fault state is always stable (corresponding terms in LMI do not affect the positive definiteness of LMI),
% it can be remove from LMI. That is, NO Ka, Qs1, Ps1, and 6*6 block matrix will become 5*5 block matrix.
%
% By testing, this method will let LMI more difficult to solve. The reason could be the increasing of conservative in Pbs, Pbe
% due to let Pbs = blkdiag(Ps, Ps1, Ps2) rather origin Pbs.

%% solve
I = eye(sys.DIM_X);
I1 = eye(sys1.DIM_X);
I2 = eye(sys2.DIM_X);
A = sys.A;
A1 = sys1.A;
A2 = sys2.A;
B = sys.B;
B1 = sys1.B;
B2 = sys2.B;
C = sys.C;
C1 = sys1.C;
C2 = sys2.C;
Es = sys.E;
Es1 = sys1.E;
Es2 = sys2.E;
Ee = sys.E;
Ee1 = sys1.E;
Ee2 = sys2.E;
rho_s = sys.rho;
rho_s1 = sys1.rho;
rho_s2 = sys2.rho;
rho_e = sys.rho;
rho_e1 = sys1.rho;
rho_e2 = sys2.rho;
Qs = sys.Q1;
Qs1 = sys1.Q1;
Qs2 = sys2.Q1;
Qe = sys.Q2;
Qe1 = sys1.Q2;
Qe2 = sys2.Q2;

options = sdpsettings('solver', 'mosek');
options = sdpsettings(options,'verbose', 0);
eqn = [];

Ws = sdpvar(sys.DIM_X, sys.DIM_X);
Ws1 = sdpvar(sys1.DIM_X, sys1.DIM_X);
Ws2 = sdpvar(sys2.DIM_X, sys2.DIM_X);
Pe = sdpvar(sys.DIM_X, sys.DIM_X);
Pe1 = sdpvar(sys1.DIM_X, sys1.DIM_X);
Pe2 = sdpvar(sys2.DIM_X, sys2.DIM_X);
Y = sdpvar(sys.DIM_U, sys.DIM_X);
% Y1 = sdpvar(sys.DIM_U, sys1.DIM_X);
K1 = -sys1.C;
Y2 = sdpvar(sys.DIM_U, sys2.DIM_X);
Z = sdpvar(sys.DIM_X, sys.DIM_Y);
Z1 = sdpvar(sys1.DIM_X, sys.DIM_Y);
Z2 = sdpvar(sys2.DIM_X, sys.DIM_Y);

eqn = [eqn, Ws >= 0, Ws1 >= 0, Ws2 >= 0, Pe >= 0, Pe1 >= 0, Pe2 >= 0];

M11 = addSym(A*Ws + B*Y) + rho_s^(-2)*(Es*Es');

M12 = zeros(sys.DIM_X, sys1.DIM_X);
M22 = addSym(A1*Ws1) + rho_s1^(-2)*(Es1*Es1');

M13 = B*Y2;
M23 = zeros(sys1.DIM_X, sys2.DIM_X);
M33 = addSym(A2*Ws2) + rho_s2^(-2)*(Es2*Es2');

M14 = -B*Y;
M24 = zeros(sys1.DIM_X, sys.DIM_X);
M34 = zeros(sys2.DIM_X, sys.DIM_X);
M44 = addSym(Pe*A + Z*C) + Qe;

M15 = -B*K1*Ws1;
M25 = zeros(sys1.DIM_X, sys1.DIM_X);
M35 = zeros(sys2.DIM_X, sys1.DIM_X);
M45 = Pe*B1*C1 + (Z1*C)';
M55 = addSym(Pe1*A1) + Qe1;

M16 = -B*Y2;
M26 = zeros(sys1.DIM_X, sys2.DIM_X);
M36 = zeros(sys2.DIM_X, sys2.DIM_X);
M46 = Z*B2*C2 + (Z2*C)';
M56 = Z1*B2*C2;
M66 = addSym(Pe2*A2 + Z2*B2*C2) + Qe2;

% without M22, M33
% N11 = [
%     M11  M14  M15  M16
%     M14' M44  M45  M46
%     M15' M45' M55  M56
%     M16' M46' M56' M66
% ];
% without M 2,2
% N11 = [
%     M11  M13  M14  M15  M16
%     M13' M33  M34  M35  M36
%     M14' M34' M44  M45  M46
%     M15' M35' M45' M55  M56
%     M16' M36' M46' M56' M66
% ];
N11 = [
    M11  M12  M13  M14  M15  M16
    M12' M22  M23  M24  M25  M26
    M13' M23' M33  M34  M35  M36
    M14' M24' M34' M44  M45  M46
    M15' M25' M35' M45' M55  M56
    M16' M26' M36' M46' M56' M66
];
N12 = blkdiag(Ws*sqrt(Qs), Ws1*sqrt(Qs1), Ws2*sqrt(Qs2), rho_e^(-1)*Pe*Ee, rho_e1^(-1)*Pe1*Ee1, rho_e2^(-1)*Pe2*Ee2);
N22 = blkdiag(-I, -I1, -I2, -I, -I1, -I2);

LMI = [
    N11  N12
    N12' N22
];

eqn = [eqn, N11 <= 0];  
% eqn = [eqn, LMI <= d1*eye(4*DIM_X)];  

% If you want to limit size of L, i.e., Y'*weight*Y <= Ub
% weight = eye(DIM_X); % weight of Y
% Ub = 10^(5)*eye(DIM_Y); % upper bound
% LMI = [
%     -Ub  Y2'
%     Y2  -weight
% ];
% eqn = [eqn, LMI <= 0];

sol = optimize(eqn, [], options);

% If you want to see solution information
% if sol.problem % Solve failed
    sol.info
% end

Ws = value(Ws);
Ws1 = value(Ws1);
Ws2 = value(Ws2);
Pe = value(Pe);
Pe1 = value(Pe1);
Pe2 = value(Pe2);
Y = value(Y);
% Y1 = value(Y1);
Y2 = value(Y2);
Z = value(Z);
Z1 = value(Z1);
Z2 = value(Z2);

K = Y/Ws;
% K1 = Y1/Ws1;
K2 = Y2/Ws2;
L = Pe\Z;
L1 = Pe1\Z1;
L2 = Pe2\Z2;

K = [K, K1, K2];
L = [L; L1; L2];
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
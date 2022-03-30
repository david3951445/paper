function [K, L] = solveLMI10(A, B, C, E, Q1, Q2, R, rho)
%solution of "Qb + Pb*Ab + Ab'*Pb + Pb*E*E'*Pb/rho^2 < 0, Pb > 0"
%
% This function is used to solve a control problem defined below :
% (1) system
%       dxb/dt = Ab*xb + vb
%       where xb = [x; x-xh], Ab = [A+BK -BK; 0 A+LC]
% (4) control law
%       u = K*xh
% (5) H infinity performance
%       (x-xh)'*Q2*(x-xh) + x*Q1*x/ vb'*vb < rho^2
%       Qb = [Q1 0; 0 Q2]
% (6) Lyapunov function
%       V(x) = xb'Pb*xb
%       where Pb = [P1 0; 0 P2]

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

eqn = [eqn, W1 >= 0, P2 >= 0];

M11 = addSym(A*W1 + B*Y1);% + rho^(-2)*E*E';

M12 = -B*Y1;
M22 = addSym(P2*A + Y2*C) + Q2;

M13 = W1*sqrt(Q1);
M23 = O;
M33 = -I;

M14 = O;
M24 = rho^(-1)*P2;%*E;
M34 = O;
M44 = -I;
if ~isempty(R)
    M15 = Y1'*sqrt(R);
    M25 = zeros(DIM_X, DIM_U);
    M35 = zeros(DIM_X, DIM_U);
    M45 = zeros(DIM_X, DIM_U);
    M55 = -eye(DIM_U);
    LMI = [
        M11  M12  M13  M14  M15
        M12' M22  M23  M24  M25
        M13' M23' M33  M34  M35
        M14' M24' M34' M44  M45
        M15' M25' M35' M45' M55
    ];
else
    LMI = [
        M11  M12  M13  M14
        M12' M22  M23  M24
        M13' M23' M33  M34
        M14' M24' M34' M44
    ];
end
% LMI = [
%     M11  M12  M13
%     M12' M22  M23
%     M13' M23' M33
% ];
% LMI = [
%     M11  M12
%     M12' M22
% ];

eqn = [eqn, LMI <= 0];  
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
    
W1 = value(W1);
P2 = value(P2);
Y1 = value(Y1);
Y2 = value(Y2);

% P = eye(DIM_X)/W;
K = Y1/W1;
L = P2\Y2;
end

%% local function
function y = addSym(x)
    y = x + x';
end
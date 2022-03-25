function [K, L] = solveLMI9(A, B, C, Q1, Q2, rho)
%solution of "Q + Pb*Ab + Ab'*Pb + Pb*Pb/rho^2 < 0, Pb > 0"
%
% This function is used to solve a control problem defined below :
% (1) system
%       dxb/dt = Ab*xb + vb
%       where xb = [x; xh], Ab = [A BK; -LC A+LC]
% (4) control law
%       u = K*x
% (5) H infinity performance
%       (x-xh)'*Q2*(x-xh) + x*Q1*x/ vb'*vb < rho^2
% (6) Lyapunov function
%       V(x) = x'Pb*x
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

M11 = addSym(A*W1) + rho^(-2)*I;

M12 = B*Y1 - C'*Y2' - W1*Q2;
M22 = addSym(P2*A + Y2*C) + Q2;

M13 = W1*sqrtm(Q1+Q2);
M23 = O;
M33 = -I;

M14 = O;
M24 = P2;
M34 = O;
M44 = -rho^2*I;

LMI = [
    M11  M12  M13  M14
    M12' M22  M23  M24
    M13' M23' M33  M34
    M14' M24' M34' M44
];

eqn = [eqn, LMI <= 0];  
% eqn = [eqn, LMI <= d1*eye(4*DIM_X)];  

% If you want to limit size of K, i.e., Y*weight*Y' <= Ub
% weight = -eye(DIM_X); % weight of Y
% Ub = -1000*eye(DIM_U); % upper bound
% LMI2 = [
%     Ub  Y
%     Y'  weight
% ];
% eqn = [eqn, LMI2 <= 0];

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
function [K, L] = solveLMI13(A, B, C, E, Q1, Q2, R, rho)
%solution of "Qb + Pb*Ab + Ab'*Pb + Kb'RKb + Pb*Pb/rho^2 < 0, Pb > 0" (Method in my paper)
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
%
% This method is currently not feasible. The term Kb'RKb cause the matrix inequality coupling and need two-step method.
% The version without Kb'RKb, solveLMI10(), is feasible.

%% solve
[DIM_X, DIM_U]  = size(B);
[DIM_Y, ~]      = size(C);
O               = zeros(DIM_X);
I               = eye(DIM_X);
options = sdpsettings('solver', 'mosek');
options = sdpsettings(options,'verbose', 0);

%% step 1
eqn = [];
W1 = sdpvar(DIM_X, DIM_X);
Y1 = sdpvar(DIM_U, DIM_X);

eqn = [eqn, W1 >= 0];

M11 = addSym(A*W1 + B*Y1) + rho^(-2)*I; % E for LMI solvable

M12 = W1*sqrt(Q1);
M22 = -I;

M13 = Y1';
M23 = zeros(DIM_X, DIM_U);
M33 = -eye(DIM_U)/R;

% LMI = [
%     M11  M12  M13  M14  M15
%     M12' M22  M23  M24  M25
%     M13' M23' M33  M34  M35
%     M14' M24' M34' M44  M45
%     M15' M25' M35' M45' M55
% ];

LMI = [
    M11  M12  M13
    M12' M22  M23
    M13' M23' M33
];

eqn = [eqn, LMI <= 0];  
% eqn = [eqn, LMI <= d1*eye(4*DIM_X)];  

sol = optimize(eqn, [], options);

% If you want to see solution information
% if sol.problem % Solve failed
    sol.info
% end
    
W1 = value(W1);
Y1 = value(Y1);
P1 = I/W1;
K = Y1*P1;

%% step 2
eqn = [];
P2 = sdpvar(DIM_X, DIM_X);
Y2 = sdpvar(DIM_X, DIM_Y);

eqn = [eqn, P2 >= 0];
M11 = Q1 + addSym(P1*(A + B*K)) + K'*R*K + rho^(-2)*P1*P1;
max(eig(M11))

M12 = -P1*B*K - K'*R*K;
M22 = Q2 + addSym(P2*A + Y2*C) + K'*R*K;

M13 = P2;
M23 = O;
M33 = -rho^(-2)*I;

LMI = [
    % M11  M13
    % M13' M33
    M11  M12  M13
    M12' M22  M23
    M13' M23' M33
];

eqn = [eqn, LMI <= 0];  
% eqn = [eqn, LMI <= d1*eye(4*DIM_X)];  

sol = optimize(eqn, [], options);

% If you want to see solution information
% if sol.problem % Solve failed
    sol.info
% end
    
P2 = value(P2);
Y2 = value(Y2);
L = I/P2*Y2;

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
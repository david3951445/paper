function K = solveLMI6(A, B, Q, R, rho)
%solution of "Q + P(A+BK) + (A+BK)'P + PP/rho^2 + KRK < 0, P > 0"
%
% This function is used to solve a control problem defined below :
% (1) system
%       dx/dt = Ax + Bu + Ev
% (4) control law
%       u = Kb*x
% (5) H infinity performance
%       x'Qx / vb'vb < rho^2
% (6) Lyapunov function
%       V(x) = x'Px

%% solve
[DIM_X, DIM_U]  = size(B);
O               = zeros(DIM_X);
I               = eye(DIM_X);
options = sdpsettings('solver', 'mosek');
options = sdpsettings(options,'verbose', 0);
eqn = [];

W = sdpvar(DIM_X, DIM_X);
Y = sdpvar(DIM_U, DIM_X);

eqn = [eqn, W >= 0];

M11 = addSym(A*W + B*Y) + rho^(-2)*I;

M12 = W*sqrt(Q);
M22 = -I;

LMI = [
    M11  M12
    M12' M22
];

% Consider R
if ~isempty(R)
    M13 = Y'*sqrt(R);
    % M13 = Y';
    M23 = zeros(size(B));
    M33 = -eye(DIM_U);
    % M33 = -inv(R);

    LMI = [
        M11  M12  M13
        M12' M22  M23
        M13' M23' M33
    ];
end

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
    
W = value(W);
Y = value(Y);

% P = eye(DIM_X)/W;
K = Y/W;
end

%% local function
function y = addSym(x)
    y = x + x';
end
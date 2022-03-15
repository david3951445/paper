function L = solveLMI7(A, C, Q, rho)
%solution of "Q + Pb*Ab + Ab*Pb + PbPb/rho^2 < 0, Pb > 0"
%
% This function is used to solve a control problem defined below :
% (1) system
%       dx/dt = Ax + v
% (2) observer
%       dxh/dt = Ax + L*C*(x - xh)
% (3) augment sys
%       dxb/dt = Ab*xb + vb
%       where xb = [x; x-xh], Ab = [A 0; 0 A-L*C], vb = [v; 0]
% (5) H infinity performance
%       x'Qbx / vb'vb < rho^2
% (6) Lyapunov function
%       V(x) = x'Pbx
%       where Pb = [P1 0; 0 P2]

%% solve
[DIM_U, DIM_X]  = size(C);
O               = zeros(DIM_X);
I               = eye(DIM_X);
options = sdpsettings('solver', 'mosek');
options = sdpsettings(options,'verbose', 0);
eqn = [];

P1 = sdpvar(DIM_X, DIM_X);
P2 = sdpvar(DIM_X, DIM_X);
Y2 = sdpvar(DIM_X, DIM_U);

eqn = [eqn, P1 >= 0, P2 >= 0];

M11 = addSym(P1*A);

M12 = O;
M22 = Q + addSym(P2*A - Y2*C);

M13 = P1;
M23 = O;
M33 = -rho^(-2)*eye(DIM_X);
% M33 = -inv(R);

M14 = O;
M24 = P2;
M34 = O;
M44 = -rho^(-2)*eye(DIM_X);

LMI = [
    M11  M12  M13 M14
    M12' M22  M23 M24
    M13' M23' M33 M34
    M14' M24' M34 M44
];

eqn = [eqn, LMI <= 0];   

sol = optimize(eqn, [], options);

% If you want to see solution information
% if sol.problem % Solve failed
    sol.info
% end
    
P2 = value(P2);
Y2 = value(Y2);

L = P2\Y2;
end

%% local function
function y = addSym(x)
    y = x + x';
end
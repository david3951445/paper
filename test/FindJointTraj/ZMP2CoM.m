function CoM = ZMP2CoM(zmp, dt, h)
%zmp trajectory to CoM trajectory
% This method is based on this paper:
% https://ieeexplore.ieee.org/document/1241826
% Here using LQ output tracking in optimal control, not MPC, to obtain control law.
% cost function:
%       2*J = (y(N)-r(N))'P(y(N)-r(N)) + Sigma_k{(y(k)-r(k))'Q(y(k)-r(k)) + u(k)Ru(k)}
% The trajectory is in R^2, i.e. x and y
%
% input:
% zmp   - zmp trajectory
% dt    - sampling time, i.e. time interval between two points in ZMP
%         trajectory
% h     - height of CoM of robot.
%
% output:
% CoM   - CoM trajectory

%% tunable parameters
P = 1000; % weighting of final tracking error
R = 1; % weighting of control effort
Q = P; % weighting of tracking error 

%% main algorithm
g           = 9.81; % gravity
N           = length(zmp);
A           = [1 dt dt^2/2; 0 1 dt; 0 0 1];
B           = [dt^3/6; dt^2/2; dt];
C           = [1 0 -h/g];
[DIM_X, ~]  = size(B);
[DIM_Y, ~]  = size(C);

CoM = zeros(2, N);
for i = 1 : 2 % x, y
    r       = zmp(i, :); % zmp
    S       = cell(1, N);
    v       = cell(1, N);
    y       = zeros(DIM_Y, N);
    x       = zeros(DIM_X, N);
    S{N}    = C'*P*C;
    v{N}    = C'*P*r(N);
    x(:, 1) = [0; 0; 0]; % position, velocity, acceleration
    y(:, 1) = C*x(:, 1);
    
    for k = N-1 : -1 : 1
        K{k} = (B'*S{k+1}*B + R)\B'*S{k+1}*A; % feedback gain
        S{k} = A'*S{k+1}*(A-B*K{k}) + C'*Q*C;
        v{k} = (A-B*K{k})'*v{k+1} + C'*Q*r(k);
        Kv{k} = (B'*S{k+1}*B + R)\B'; % feedforward gain
    end
    
    for k = 1 : N-1
        x(:, k+1) = (A - B*K{k})*x(:, k) + B*Kv{k}*v{k+1};
        y(:, k+1) = C*x(:, k+1); % output
    end

    CoM(i, :) = x(1, :);
end
end


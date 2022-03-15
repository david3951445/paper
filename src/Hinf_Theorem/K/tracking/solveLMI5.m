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
    %       where xb = [x; x-xr], vb = [v; r]
    %             Ab = [A 0; A-Ar Ar], Bb = [B; B], Eb = [E 0; E -Br]
    % (4) control law
    %       u = Kbxb
    %       where Kb = K
    % (5) H infinity performance
    %       xb'Qbxb / vb'vb < rho^2
    %       where Qb = [Q 0; 0 Q]
    % (6) Lyapunov function
    %       V(x) = xb'Pbxb
    %       where Pb = P
    %
    % Given (1) ~ (6), form H infinity theorem, if
    %       "Qb + Pb(Ab+BbKb) + (Ab+BbKb)'Pb + PbEbEb'Pb/rho^2 < 0, Pb > 0"
    % hold then (3) achieve (5).
    
    %% tunable parameter
    d1 = 0*10^(-4); % LMI <= d1*I. if problem infeasible, try increasing d1
    
    %% solve
    [DIM_X, DIM_U]  = size(B);
    O               = zeros(DIM_X);
    Ab = [A O; A-Ar Ar];
    Bb = [B; B];
    Eb = [E O; E -Br];
    Qb = [Q O; O Q];

    options = sdpsettings('solver', 'sdpt3');
    options = sdpsettings(options,'verbose', 0);
    eqn = [];

    W = sdpvar(DIM_X*2, DIM_X*2); % symmetric
    % W1 = sdpvar(DIM_X, DIM_X); % symmetric
    % W2 = sdpvar(DIM_X, DIM_X); % symmetric
    % W = blkdiag(W1, W2);

    Y = sdpvar(DIM_U, DIM_X*2); % full
    % Y2 = sdpvar(DIM_U, DIM_X); % full
    % Y = [zeros(DIM_U, DIM_X), Y2];

    eqn = [eqn, W >= 0];
    
    M11 = addSym(Ab*W + Bb*Y) + rho^(-2)*Eb*Eb';
    
    M12 = W;
    M22 = -inv(Qb);%eye(DIM_X*2);
    
    LMI = [
        M11  M12
        M12' M22
    ];

    eqn = [eqn, M11 <= d1*eye(2*DIM_X)];  
    % eqn = [eqn, LMI <= d1*eye(4*DIM_X)];  

    % If you want to limit size of K
    % LMI2 = [
    %     -10^(4)*eye(DIM_U)   Y2
    %     Y2'                 -eye(DIM_X)
    % ];
    % eqn = [LMI <= eq1, LMI2 <= 0, W >= 0;
    
    sol = optimize(eqn, [], options);
    
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
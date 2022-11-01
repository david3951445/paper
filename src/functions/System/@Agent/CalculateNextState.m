%
% input:
%   - xb    | matrix    : augment state
%   - d1    | matrix    : disturbance on acuator
%   - r     | scaler    : reference
%   - dr    | scaler    : first difference of reference
%   - ddr   | scaler    : second difference of  reference
%   - i     | scaler    : time step

function [ag, xb] = CalculateNextState(ag, xb, d1, r, dr, ddr, i)
    DIM_X3 = ag.sys_aug.DIM_X;
    DIM_F = ag.DIM_F;

    %% extract x, xh, X, dX form xb
    x       = xb(1 : DIM_X3, i);
    xh      = xb(DIM_X3 + (1:DIM_X3), i);
    X       = x(DIM_F + (1:DIM_F)); % position
    dX      = x(2*DIM_F + (1:DIM_F)); % velocity
    Xh      = xh(DIM_F + (1:DIM_F));
    dXh     = xh(2*DIM_F + (1:DIM_F));

    %% fault signal
    % feedback linearized term: Mh(), Hh(). By testing, using feedforward only ( Mh(r(t)) ) is more stable then feedback + feedforward ( Mh(xh(t)+r(t)) ).
    u = ag.u_PID(xh); % PID control

    M = ag.M(X+r); 
    Mh = ag.M(r); % feedforward compensation
    % Mh = ag.M(Xh+r); 
    % Mh = 0;
    
    H = ag.H(X+r, dX+dr);
    Hh = ag.H(r, dr); % feedforward compensation
    % Hh = ag.H(Xh+r, dXh+dr); % feedback compensation
    % Hh = 0;

    d1(:, i) = d1(:, i) + 1*x(DIM_F + (1:DIM_F)).*x(DIM_F + (1:DIM_F)); % Couple terms
    % fext1 = externalForce(ag.rbtree, 'body11', [0 0 0 0 0 ag.tr.GRF{1}(i)], X');
    % tau = inverseDynamics(ag.rbtree, X', [], [], fext1);
    ag.sys_a.fault(:, i) = -eye(DIM_F)/M*((M-Mh)*(ddr + u) + H-Hh - d1(:, i)); % Total fault
    % ag.sys_a.fault(:, i) = -eye(DIM_F)/M*((M-Mh)*(ddr + u) - d1(:, i));
    % ag.sys_a.fault(:, i) = -eye(DIM_F)/M*(-d1(:, i));
    % f = tau;

    %% control
    % ag.tr.u(:, i+1) = u;
    ag.tr.u(:, i+1) = Mh*(ddr + u) + Hh;

    % Show norm of error terms
    if mod(i, 100) == 0 
        % disp(['norm of Mh: ' num2str(norm(Mh))])
        % disp(['norm of M-Mh: ' num2str(norm(M-Mh))])
        % disp(['norm of H-Hh: ' num2str(norm(H-Hh))])
    end
    
    xb(:, i+1) = ODE_solver(@ag.f_aug, ag.tr.dt, [x; xh], ag.tr.t(i), 'RK4');
    % xb(:, i+1) = xb(:, i) + fun(ag.tr.t(i), xb(:, i))*ag.tr.dt;

    %% assign real fault signal
    xb = ag.sys_a.set_real_signal(xb, i);
    xb = ag.sys_s.set_real_signal(xb, i);
end
function uav = trajectory(uav, ref)
%calculate trajectory

dt          = uav.tr.dt;
t           = 0 : dt : uav.tr.T;
LEN         = length(t);

xb          = zeros(2*uav.DIM_X, LEN);
xb(:, 1)    = [uav.tr.x0; uav.tr.xr0];
u           = zeros(uav.DIM_U, LEN);

disp('Ploting trajectory ...')
for i = 1 : LEN - 1       
    if mod(i, 1/dt) == 0
        disp(['t = ' num2str(i*dt)])
    end
    
    k1 = RK4(uav, ref, xb(:, i), i*dt);
    if uav.tr.IS_RK4
        k2 = RK4(uav, ref, xb(:, i)+k1*dt/2, i*dt + dt/2);
        k3 = RK4(uav, ref, xb(:, i)+k2*dt/2, i*dt + dt/2);
        k4 = RK4(uav, ref, xb(:, i)+k3*dt, i*dt + dt); 

        xb(:, i+1)  = xb(:, i)  + (k1 + 2*k2 + 2*k3 + k4)*dt/6;
    end
    xb(:, i+1)  = xb(:, i) +  k1*dt;

    uav.tr.x(:, i+1) = xb(1 : uav.DIM_X, i+1);
    uav.tr.xr(:, i+1) = xb(uav.DIM_X+1 : 2*uav.DIM_X, i+1);

    % total_K = zeros(o.DIM_U, o.DIM_X);
    % for j = 1 : fz.num
    %     total_K = total_K + fz.mbfun(j, o.x(:, i+1))*uav.K{j};
    % end
    % o.u(:, i+1) = total_K*(o.x(:, i+1) - o.xr(:, i+1));
end
uav.tr.t = t;
end

%% Local function
function k = RK4(uav, ref, xb, t)
    O       = zeros(uav.DIM_X);
    v       = @(t) 0.01*randn(uav.DIM_X, 1) + 0;
    [r, F]  = ref.r_F(xb(1 : uav.DIM_X), t);
    x       = xb(1 : uav.DIM_X);
    xr      = xb(uav.DIM_X+1 : uav.DIM_X*2);

    total.A = O;
    total.B = zeros(uav.DIM_X, uav.DIM_U);
    total.K = total.B';

    if uav.tr.IS_LINEAR % linear
        ABK = zeros(uav.DIM_X*2);

        for i = 1 : uav.AB.len
            A = uav.AB.val{i}(:, 1 : DIM_X);
            B = uav.AB.val{i}(:, DIM_X+1 : DIM_X+DIM_U);
            K = uav.K{i};
            
            Ab = [A O; O ref.A];
            Bb = [B; zeros(uav.DIM_X, uav.DIM_U)];
            Kb = [K -K];
            % Kb = K;

            total.A = total.A + fz.mbfun(i, x)*A;
            total.B = total.B + fz.mbfun(i, x)*B;
            total.K = total.K + fz.mbfun(i, x)*K;

            ABK = ABK + fz.mf(x, i)*(Ab + Bb*Kb);
        end
        Eb = [eye(uav.DIM_X) O; O ref.B];

        feed = [uav.g(x)*[F; 0; 0; 0]; zeros(12, 1)]; % feedforward
        k = ABK*xb + Eb*[v(t); r] + 0*feed;

        % u = K*(x - xr);
        % k = [
        %     total.A*x + total.B*u + eye(o.DIM_X)*v(t)
        %     ref.A*xr + ref.B*r
        % ];
    else % nonlinear
        % control gain
        for i = 1 : uav.AB.len
            total.K = total.K + uav.AB.mf(x, i)*uav.K{i};
        end
        % fine-tune
        % K = K;
        
        U = total.K*(x - xr);

        k = [
            uav.f(x) + uav.g(x)*U + eye(uav.DIM_X)*v(t)
            ref.A*xr + ref.B*r
        ];
    end
end
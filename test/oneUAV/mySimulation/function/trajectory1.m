function tr = trajectory1(uav, fz, ref, p)
%Linear trajectory
dt = p.dt;
T = p.tf;
t = 0 : dt : T;

x0 = [0.54 0.55 0.57 0.57 2 0.5 0.51 0.59 0.52 0.52 0.55 0.52]';
xr0 = [0 1 0.5 0 0 0.8 0 0 0 0 0 0]';
v = @(t) 0.5*randn(12, 1) + 0;
    
%% Linear Trajectory
vb = @(x, t) [v(t); ref.r(x, [], t)];

x = zeros(uav.dim*2, length(t));
x(:, 1) = [x0; xr0];
for i = 1 : length(t) - 1       
    % if mod(i, 1/dt) == 0
    %     disp(['t = ' num2str(i*dt)])
    % end

    k1 = RK4(uav, fz, ref, x(:, i), vb,          i*dt);
    k2 = RK4(uav, fz, ref, x(:, i)+k1*dt/2, vb,  i*dt + dt/2);
    k3 = RK4(uav, fz, ref, x(:, i)+k2*dt/2, vb,  i*dt + dt/2);
    k4 = RK4(uav, fz, ref, x(:, i)+k3*dt, vb,    i*dt + dt);

    x(:, i+1)  = x(:, i)  + (k1 + 2*k2 + 2*k3 + k4)*dt/6;
    % obj.x(:, i+1)  = obj.x(:, i) +  k1*dt;
end

tr.t = t;
tr.x = x(1:12, :);
tr.xr = x(13:24, :);
end

%% Local function
function k = RK4(uav, fz, ref, rb, x, t)
    A = zeros(uav.dim);
    B = zeros(uav.dim, uav.dim_u);
    K = B';
    Ab = zeros(uav.dim*2);
    for i = 1 : fz.num
        A = A + uav.A(:, :, i);
        B = B + uav.B(:, :, i);
        K = K + uav.K(:, :, i);
        Ab = Ab + fz.mbfun(i, x)*[A+B*K -B*K; uav.O ref.Ar];
    end
    Eb = [uav.E uav.O; uav.O ref.B];

    k = Ab*x + Eb*vb(x, t);
end
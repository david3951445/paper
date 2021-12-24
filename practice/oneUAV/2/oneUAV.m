clc; clear; close all; tic
uav = UAV; fz = Fuzzy; ref = REF(uav);
p = struct; % rho, Q, A, B, K, P1, P2

% constant parameter

% tunable parameter
runID        = 0; % whether run inverse dynamics part
runLinearize = 1; % whether run linearize part
runLMI       = 1; % whether run LMI part

p.tf   = 4*pi;      % final time of trajectory
p.dt   = 0.005;     % time step of RK4
p.ampv = 0;           % amplitude of v
p.rho  = 10^(4);
p.Q    = 10^(-3)*diag([1000, 1000, 1000, 1000, 1, 1, 1, 0, 1, 0, 1, 1]);

% untunable
p.t = 0 : p.dt : p.tf;

%% inverse dynamic
if runID == 0
    load ID
else
    xd2 = diff(diff(ref.xd));
    yd2 = diff(diff(ref.yd));

    thd  = atan(xd2/uav.G);
    thd1 = diff(thd);
    thd2 = diff(thd1);

    phd  = atan(-yd2*cos(thd)/uav.G);
    phd1 = diff(phd);
    phd2 = diff(phd1);

    thd  = double(subs(thd, p.t));
    thd1 = double(subs(thd1, p.t));
    thd2 = double(subs(thd2, p.t));

    phd  = double(subs(phd, p.t));
    phd1 = double(subs(phd1, p.t));
    phd2 = double(subs(phd2, p.t));

    uo = [  uav.m*uav.G./cos(phd)./cos(thd);
            uav.Jx.*phd2;
            uav.Jy.*thd2;
            (uav.Jx - uav.Jy).*phd1.*thd1;  ]; % feedforward control
        
    save ID.mat uo
end
uav.uo = uo;

%% linearize
if runLinearize == 0
    load Matrix
else
    p = getLocalMatrix(uav, fz, p);
    save Matrix.mat p
end
uav.A = p.A; uav.B = p.B;

%% find K
% H infinity peformance : xQx/vv < rho^2
if runLMI == 0
    load Matrix
else
    p = getControlGain(uav, fz, ref, p);
    save Matrix.mat p
end
uav.K = p.K;

%% trajectory
% uav.uo = zeros(uav.dim_u, length(p.t));
% uav.K = zeros(uav.dim_u, uav.dim, fz.num);
tr = trajectory(uav, fz, ref, p, 'RK4');

%% plot
Plot(tr)

%% augment system parameter
% eigOfLMI(uav, fz, ref, p);

toc

%% functions
function p = getLocalMatrix(uav, fz, p)  
    s = sym('s', [uav.dim, 1]);
    
    % find A
    A = zeros(uav.dim, uav.dim, fz.num);
    for k = 1 : fz.num
        A(:, :, k) = 0.01*eye(uav.dim);
%         y = subs(uav.f(s), s(fz.PV), fz.set(:, k));
%         for i = 1 : uav.dim
%             [cf, u] = coeffs(y(i));
%             u = subs(u, s, (1:uav.dim)');
%             A(i, u, k) = cf;
%         end
    end

    % find B
    B = zeros(uav.dim, uav.dim_u, fz.num);
    for k = 1 : fz.num
        y = subs(uav.g(s), s(fz.PV), fz.set(:, k));
        B(:, :, k) = subs(y, s(11), 0); % let phi = 0
    end
    
    p.A = A; p.B = B;
end

function Plot(tr)
    % figure('units','normalized','outerposition',[0 0 1 1])
    state = [1:12];
    for i = 1 : length(state)
        figure(i)
    %     subplot(1, 2, i);
        plot(tr.t, tr.x(state(i), :), tr.t, tr.xr(state(i), :));
        title(['x_{' num2str(state(i)) '}']); legend("x", "x_r");
        xlabel("t"); %ylim([-2 2])
    end
end

function eigOfLMI(uav, fz, ref, p)
I = eye(uav.dim);
O = zeros(uav.dim);
Qb = [p.Q -p.Q; -p.Q p.Q];
Fb = [I O; O ref.B];
% Ab = zeros(2*uav.dim, 2*uav.dim, fz.num);
% Bb = zeros(2*uav.dim, uav.dim_u, fz.num);
% Kb = zeros(uav.dim_u, 2*uav.dim, fz.num);
Mb = zeros(2*uav.dim, 2*uav.dim);

% calculate if LMI is positive definite
for i = 1 : fz.num
    Ab = [uav.A(:, :, i) O; O ref.A];
    Bb = [uav.B(:, :, i); zeros(uav.dim, uav.dim_u)];
    Kb = [uav.K(:, :, i) -uav.K(:, :, i)];
    Pb = [p.P1(:, :, i) O; O p.P2(:, :, i)];
    
    Mb = Mb + Qb + symmetric((Ab + Bb*Kb)'*Pb) + p.rho^(-2)*Pb*(Fb*Fb')*Pb;
end
max(eig(Mb))
end

function y = symmetric(x)
    y = x + x';
end

% method 2 : https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7535919
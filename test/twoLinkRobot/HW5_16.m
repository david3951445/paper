clc; clear; close all
addpath(genpath('..\..\src'))
rb = robot();

% fixed parameters
I = eye(4); O = zeros(4);
Ar = [0 1 0 0; -6 -5 0 0; 0 0 0 1; 0 0 -6 -5];

% tunable parameters
d1 = 10^(-2); % scale of Q
d2 = 0.1; % scale of disturblance
d3 = 2;
dk = d3*diag([20 10 20 10], 0); % scale of K
dl = d3*diag([1 10 15 10], 0); % scale of L
Qe = d1*0*I; Q = d1*[1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]; rho = 10;
state = 'n'; % n : nonlinear, l : linear
dt = 0.002; tf = 10; % parameter of trajectory
r.amp = 6; r.freq = 1; % parameter of sine wave in reference input

rule.mf_num = 2; % # of membership function
rule.pv_num = 4;% # of premise variable
switch rule.mf_num
    case 2
        rule.x1 = 0.5*[-pi pi];
        rule.x2 = 0.4*[-pi pi];
        rule.x3 = 0.5*[-pi pi];
        rule.x4 = 0.4*[-pi pi];
    case 4        
        rule.x1 = 0.5*[-pi -pi/2 pi/2 pi];
        rule.x2 = 0.4*[-pi -pi/2 pi/2 pi];
        rule.x3 = 0.5*[-pi -pi/2 pi/2 pi];
        rule.x4 = 0.4*[-pi -pi/2 pi/2 pi];
end

rule.num = rule.mf_num^rule.pv_num;
for i = 1 : rule.num
    i0 = getIndex(i, rule.mf_num, rule.pv_num);
    rule.val(:, i) = [rule.x1(i0(1)); rule.x2(i0(2)); rule.x3(i0(3)); rule.x4(i0(4))];
end

%% find Af, Bf, C
Af = zeros(4, 4, rule.num); Bf = zeros(4, 2, rule.num);
for i = 1 : rule.num
    i0 = getIndex(i, rule.mf_num, rule.pv_num);
    var1 = [rule.x1(i0(1)) rule.x2(i0(2)) rule.x3(i0(3)) rule.x4(i0(4))];
    
    % A
    % 1, 3 row
    Af(1, 2, i) = 1; Af(3, 4, i) = 1;
    % 2, 4 row
    Af(2, :, i) = regress_2(var1, @rb.f1)';
    Af(4, :, i) = regress_2(var1, @rb.f2)';
    
    % B
    Bf(2, :, i) = [rb.g11(var1) rb.g12(var1)];
    Bf(4, :, i) = [rb.g12(var1) rb.g22(var1)];
end
% [Af, Bf] = para();
C = [1 0 0 0; 0 0 1 0];

%% LMI, find L, K
options = sdpsettings('solver','sdpt3');
options = sdpsettings(options,'verbose',0);

Kf = zeros(2, 4, rule.num); Lf = zeros(4, 2, rule.num);
for i = 1 : rule.num
    A = Af(:, :, i); B = Bf(:, :, i);
    
    % YALMIP method
    % STEP 1 : solve P2, L
    W22 = sdpvar(4, 4); % symmetric
    Y = sdpvar(2, 4); % full

    M11 = addSym(A*W22) + addSym(B*Y) + rho^(-2)*I;
    M12 = W22;
    M22 = -inv(Q);

    LMI = [M11  M12
           M12' M22];

    sol = optimize([LMI <= 0, W22 >= 0], [], options);

    if strcmp(sol.info, 'Infeasible problem (MOSEK)')
        disp('Infeasible problem (MOSEK)');
        break
    end
    
    W22 = value(W22);
    Y = value(Y);

    P22 = I/W22;
    K = Y*P22;
    
    % STEP 2 : solve W1, K
    P11 = sdpvar(4, 4);
    P33 = sdpvar(4, 4);
    Z = sdpvar(4, 2);

    % 4 4 2 4 4 4
    M11 = addSym(P11*A) - addSym(Z*C) + Qe;
    M12 = P11;
    M13 = Z;
    M14 = rho^(-2)*P11*P22 - P22*B*K;
    M15 = zeros(4);
    M16 = zeros(4);
    M22 = -rho^2*I;
    M23 = zeros(4, 2);
    M24 = zeros(4);
    M25 = zeros(4);
    M26 = zeros(4);
    M33 = -rho^2*eye(2);
    M34 = zeros(2, 4);
    M35 = zeros(2, 4);
    M36 = zeros(2, 4);
    M44 = addSym((A + B*K)'*P22) + rho^(-2)*P22*P22 + Q;
    M45 = -P22*B*K - Q;
    M46 = zeros(4);
    M55 = addSym(P33*Ar) + Q;
    M56 = P33;
    M66 = -rho^2*I;

    LMI = [
        M11  M12  M13  M14  M15  M16
        M12' M22  M23  M24  M25  M26
        M13' M23' M33  M34  M35  M36
        M14' M24' M34' M44  M45  M46
        M15' M25' M35' M45' M55  M56
        M16' M26' M36' M46' M56' M66
    ];
    
%     solvesdp([LMI <= 0, P11 >= 0, P33 >= 0])
    sol = optimize([LMI <= 0, P11 >= 0, P33 >= 0], [], options);

    P11 = value(P11);
    P33 = value(P33);
    Z = value(Z);

    L = P11\Z;
    
    Lf(:, :, i) = dl*L; Kf(:, :, i) = K*dk;
end

%% plot
t = 0 : dt : tf; t2 = 0 : dt/2 : tf;

r = r.amp*[zeros(1, length(t2)); sin(r.freq*t2); zeros(1, length(t2)); cos(r.freq*t2)]; % reference input
v = d2*wgn(2, length(t2), 0);
w = d2*wgn(4, length(t2), 0);
wb = [v; w; r];

xb = zeros(12, length(t));
xb(1, 1) = 0.5; xb(3, 1) = -0.5;

x = zeros(4, length(t)); xh = x; xr = x;
x(1, 1) = 0.5; x(3, 1) = -0.5;

% find x, u
for i = 1 : length(t) - 1
    j = 2*i-1;
    temp = zeros(12, 1); temp1 = zeros(4, 1); temp2 = zeros(4, 1);
    s = 0;
    
    for k = 1 : rule.num
        A = Af(:, :, k); B = Bf(:, :, k);
        K = Kf(:, :, k); L = Lf(:, :, k);
        i0 = getIndex(k, rule.mf_num, rule.pv_num); % index
        hh = h(xb(1, i), i0(1), rule.x1)*h(xb(2, i), i0(2), rule.x2)*h(xb(3, i), i0(3), rule.x3)*h(xb(4, i), i0(4), rule.x4);
        
        switch state
            case 'n'
                % x_hat, x, xr
                kh1 = fh(A, B, C, K, L, x(:, i), xh(:, i), xr(:, i)) - L*v(:, j);
                k1 = rb.f(x(:, i), K*(xh(:, i)-xr(:, i))) + w(j);
                kr1 = fl(j, xr(:, i), r, Ar, I);
                
                kh2 = fh(A, B, C, K, L, x(:, i)+0.5*k1*dt, xh(:, i)+0.5*kh1*dt, xr(:, i)) - L*v(:, j+1);
                k2 = rb.f(x(:, i)+0.5*k1*dt, K*(xh(:, i)-xr(:, i) + 0.5*(kh1-kr1)*dt)) + w(j+1);
                kr2 = fl(j+1, xr(:, i)+0.5*kr1*dt, r, Ar, I);
                
                kh3 = fh(A, B, C, K, L, x(:, i)+0.5*k2*dt, xh(:, i)+0.5*kh2*dt, xr(:, i)) - L*v(:, j+1);
                k3 = rb.f(x(:, i)+0.5*k2*dt, K*(xh(:, i)-xr(:, i) + 0.5*(kh2-kr2)*dt)) + w(j+1);
                kr3 = fl(j+1, xr(:, i)+0.5*kr2*dt, r, Ar, I);
                        
                kh4 = fh(A, B, C, K, L, x(:, i)+k3*dt, xh(:, i)+kh3*dt, xr(:, i)) - L*v(:, j+2);
                k4 = rb.f(x(:, i)+k3*dt, K*(xh(:, i)-xr(:, i) + (kh3-kr3)*dt)) + w(j+2);
                kr4 = fl(j+2, xr(:, i)+kr3*dt, r, Ar, I);
                
                temp1 = temp1 + hh.*(kh1+2*kh2+2*kh3+kh4)*dt/6;
                temp2 = temp2 + hh.*(k1+2*k2+2*k3+k4)*dt/6;
                xr(:, i+1) = xr(:, i) + (kr1+2*kr2+2*kr3+kr4)*dt/6;
                
            case 'l'
                Ab = [A-L*C, O, O; -B*K, A+B*K, -B*K; O, O, Ar];
                Eb = [-L, I, O; zeros(4, 2), I, O; zeros(4, 2), O, I];
                k1 = fl(j, xb(:, i), wb, Ab, Eb);
                k2 = fl(j+1, xb(:, i)+0.5*k1*dt, wb, Ab, Eb);
                k3 = fl(j+1, xb(:, i)+0.5*k2*dt, wb, Ab, Eb);
                k4 = fl(j+2, xb(:, i)+k3*dt, wb, Ab, Eb);
                temp = temp + hh.*(k1+2*k2+2*k3+k4)*dt/6;
        end
        
    end

    switch state
        case 'n'
            xh(:, i+1) = xh(:, i) + temp1;
            x(:, i+1) = x(:, i) + temp2;
            xb(:, i+1) = [xh(:, i+1); x(:, i+1); xr(:, i+1)];
        case 'l'
            xb(:, i+1) = xb(:, i) + temp;
    end
    
%         x_sum = x_sum + xb'*Qb*xb*dt;
    %     v_sum = v_sum + rho^2*vb(:, i)'*vb(:, i)*dt;
end
if state == 'l'
    xb(1:4, :) = xb(5:8, :) - xb(1:4, :);
end
% v_sum = v_sum + [xh(:, 1); x(:, 1) - xh(:, 1)]'*Pb*[xh(:, 1); x(:, 1) - xh(:, 1)];

Plot(t, xb);

%% controlabliliy
% for i = 1 : rule.num
%     A = Af(:, :, i); B=Bf(:, :, i);
%     if rank(ctrb(A,B)) ~= length(A)
%         disp('uncontrollable')
%     end
%     if rank(ctrb(A,C')) ~= length(A)
%         disp('unobservable')
%     end
% end

rmpath(genpath('..\..\src'))

%% functions
function y = getIndex(i, n, m) % transformation of index. ex: 1~27 => (1~3, 1~3, 1~3)
for j = m-1 : -1 : 1
    i0 = mod(i-1, n^j) + 1;
    y(m-j) = (i -i0)*n^(-j) +1;
    i = i0;
end
y(m) = i;
end 

function Plot(t, xb)
figure('units','normalized','outerposition',[0 0 1 1])
for i = 1 : 4
    subplot(2, 2, i); plot(t, xb(i, :), t, xb(i+4, :), t, xb(i+8, :));
    % hold on; plot(t2, vb(2, :), 'Color',[0.5 0.5 0.5]); hold off;
    title(['x_' num2str(i)]); legend("x_{hat}", "x", " xr");
    xlabel("t"); ylim([-4 4])
end
end

function y = fh(A, B, C, K, L, x, xh, xr) % dx_hat/dt
y = A*xh + B*K*(xh-xr) + L*C*(x-xh);
end

function y = fl(t, x, u, A, B) % for linear state
y = A*x + B*u(:, t);
end

function y = h(x, index, mid) % membership function of rule (i, j)
N = length(mid);
switch index
    case 1 % --\
        if x <= mid(index)
            y = 1;
        elseif x >= mid(index+1)
            y = 0;
        else
            y = 1 - (x - mid(index))/(mid(index+1) - mid(index));
        end
    case N % /--
        if x >= mid(index)
            y = 1;
        elseif x <= mid(index-1)
            y = 0;
        else
            y = 1 - (x - mid(index))/(mid(index-1) - mid(index));
        end
    otherwise % triangle /\
        if x <= mid(index-1) || x >= mid(index+1)
            y = 0;
        elseif x > mid(index-1) && x < mid(index)
            y = 1 - (x - mid(index))/(mid(index-1) - mid(index));
        else
            y = 1 - (x - mid(index))/(mid(index+1) - mid(index));
        end
end
end
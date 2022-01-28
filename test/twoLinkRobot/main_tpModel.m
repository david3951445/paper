clc; clear; close all
addpath(genpath('..\..\src'))
addpath(genpath('function'))
% fixed parameters
I = eye(4); O = zeros(4);
Ar = [0 1 0 0; -6 -5 0 0; 0 0 0 1; 0 0 -6 -5];

% tunable parameters
state = 'n'; % n : nonlinear, l : linear

d2 = 0; % scale of disturblance
d3 = 2;
dk = 1;%d3*diag([20 10 20 10], 0); % scale of K
dl = 1;%d3*diag([1 10 15 10], 0); % scale of L

d1  = 10^(-2); % scale of Q
Qe  = 0*d1*I;
Q   = d1*[1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
rho = 10;

dt      = 0.01;
tf      = 5; % parameter of trajectory
amp     = 6;
freq    = 1; % parameter of sine wave in reference input

%% find Af, Bf, C
rb = Robot();
if EXE.A
    rb.A = TPmodel(rb.Al);
    save('data/rb.mat', 'rb')
else
    load('data/rb.mat', 'rb')
end 

if EXE.B
    rb.B = TPmodel(rb.Bl);
    save('data/rb.mat', 'rb')
else
    load('data/rb.mat', 'rb')
end

Af = rb.A.val;
Bf = rb.B.val;
C = rb.C;

%% LMI, find L, K
if EXE.K_L
    options = sdpsettings('solver','sdpt3');
    options = sdpsettings(options,'verbose',0);

    Kf = cell(rb.A.len, rb.B.len); Lf = cell(rb.A.len, rb.B.len);
    for i = 1 : 1%rb.A.len
        for j = 1 : rb.B.len
            fprintf('LMI iter: %d/%d, %d/%d\n', i, rb.A.len, j, rb.B.len)
            A = Af{i}; B = Bf{j};

            % YALMIP method
            % STEP 1 : solve P2, L
            W22 = sdpvar(4, 4); % symmetric
            Y = sdpvar(2, 4); % full

            M11 = addSym(A*W22) + addSym(B*Y) + rho^(-2)*I;
            M12 = W22;
            M22 = -inv(Q);

            LMI = [
                M11  M12
                M12' M22
            ];

            % solvesdp([LMI <= 0, W22 >= 0])
            sol = optimize([LMI <= 0, W22 >= 0], [], options);

            % if strcmp(sol.info, 'Infeasible problem (MOSEK)')
            %     error('Infeasible problem (MOSEK)');
            % end
            if sol.problem == 0
                W22 = value(W22);
                Y = value(Y);
            else
                display('Hmm, something went wrong!');
                sol.info
                yalmiperror(sol.problem)
            end
            

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

            % solvesdp([LMI <= 0, P11 >= 0, P33 >= 0])
            sol = optimize([LMI <= 0, P11 >= 0, P33 >= 0], [], options);

            if sol.problem == 0
                P11 = value(P11);
                P33 = value(P33);
                Z = value(Z);
            else
                display('Hmm, something went wrong!');
                sol.info
                yalmiperror(sol.problem)
            end

            L = P11\Z;

            Lf{i, j} = dl*L; Kf{i, j} = K*dk;
        end
    end

    gain.L = Lf;
    gain.K = Kf;
    save('data/gain.mat', 'gain')
else
    load('data/gain.mat', 'gain')
end

%% plot
t = 0 : dt : tf; t2 = 0 : dt/2 : tf;

r = amp*[zeros(1, length(t2)); sin(freq*t2); zeros(1, length(t2)); cos(freq*t2)]; % reference input
v = d2*wgn(2, length(t2), 0);
w = d2*wgn(4, length(t2), 0);
wb = [v; w; r];

xb = zeros(12, length(t));
xb(1, 1) = 0.5; xb(3, 1) = -0.5;

x = zeros(4, length(t)); xh = x; xr = x;
x(1, 1) = 0.5; x(3, 1) = -0.5;


% find x, u
if EXE.TRAJ
    for i = 1 : length(t) - 1
        if isnan(norm(xb))
            error('diverge')
        end
        
        if mod(i, 0.1/dt) == 0 % print per 0.1s
            disp(['Calculate traj: t = ' num2str(t(i))]);
        end
        
        j = 2*i-1;
        temp = zeros(12, 1); temp1 = zeros(4, 1); temp2 = zeros(4, 1);
        s = 0;
        % sum_hh = 0;
        for k = 1 : rb.A.len
            for kk = 1 : rb.B.len
                A = rb.A.val{k}; B = rb.B.val{kk};
                K = gain.K{k, kk}; L = gain.L{k, kk};
                hh = rb.A.mf(xb(1:4, i), k)*rb.B.mf(xb([1, 3], i), kk);
                
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

                % sum_hh = sum_hh + hh;
            end
        end
        % sum_hh

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
end

if EXE.PLOT
    Plot(t, xb);
end

%% controlabliliy
% for i = 1 : rule.num
%     A = rb.A.val{i}; B = rb.B.val{i};
%     if rank(ctrb(A,B)) ~= length(A)
%         disp('uncontrollable')
%     end
%     if rank(ctrb(A,C')) ~= length(A)
%         disp('unobservable')
%     end
% end

rmpath(genpath('function'))
rmpath(genpath('..\..\src'))

%% if you want to check if sum of membership function is 1
sum = 0;
for i = 1 : rb.A.len
    sum = sum + rb.A.mf([0, 1, -1, 0], i);
end
disp(['sum of mbfun of A: ' num2str(sum)])
% sum = 0;
% for i = 1 : rb.B.len
%     sum = sum + rb.B.mf([0, 0, 0, 0], i);
% end
% disp(['sum of mbfun of B: ' num2str(sum)])

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
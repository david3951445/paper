%two-link robot, tpModel(A, B solve together), no observer, xb = [x x-xr]'.
clc; clear; close all
addpath(genpath('..\..\src'))
addpath(genpath('function'))

%% fixed parameters
I = eye(4); O = zeros(4);
Ar = [0 1 0 0; -6 -5 0 0; 0 0 0 1; 0 0 -6 -5];

%% tunable parameters
state = 'l'; % n : nonlinear, l : linear

d2 = 0; % scale of disturblance
d3 = 2;
dk = 1;%d3*diag([20 10 20 10], 0); % scale of K
dl = 1;%d3*diag([1 10 15 10], 0); % scale of L

d1  = 10^(-2); % scale of Q
Qe  = 0*d1*I;
Q   = d1*diag([1 1 1 1]);
Q1  = 10^(-6)*I;
Q2 = 10^(-3)*I;
rho = 10^(3);

dt      = 0.01;
tf      = 5; % parameter of trajectory
amp     = 6;
freq    = 1; % parameter of sine wave in reference input

%% find Af, Bf, C
rb = Robot();
% if ~EXE.A
%     load('data/rb.mat', 'rb')
% end
% if ~EXE.B
if EXE.A
    rb.A = TPmodel(rb.Al);
    rb.save('data/rb', 'A')
else
    rb.A = load('data/rb', 'A').A;
end 

if EXE.B
    rb.B = TPmodel(rb.Bl);
    rb.save('data/rb', 'B');
else
    rb.B = load('data/rb', 'B').B;
end

Af = rb.A.val;
Bf = rb.B.val;
C = rb.C;

%% LMI, find L, K
if EXE.K_L
    options = sdpsettings('solver','sdpt3');
    options = sdpsettings(options,'verbose',0);

    Kf = cell(rb.A.len, rb.B.len);
    for i = 1 : rb.A.len
        for j = 1 : rb.B.len
            fprintf('LMI iter: %d/%d, %d/%d\n', i, rb.A.len, j, rb.B.len)
            A = Af{i}; B = Bf{j};

            %%% METHOD 3, [Q1 0; 0 Q2], preposMulti [W1 0; 0 W2]
            W1 = sdpvar(4, 4); % symmetric
            W2 = sdpvar(4, 4);
            Y2 = sdpvar(2, 4); % full

            M11 = addSym(A*W1) + rho^(-2)*I;
            M12 = B*Y2 + (A*W1)';
            M13 = W1;
            M14 = zeros(4);
            M22 = addSym(-Ar*W2 + B*Y2) + 2*rho^(-2)*I;
            M23 = zeros(4);
            M24 = W2;
            M33 = -inv(Q1);
            M34 = zeros(4);
            M44 = -inv(Q2);

            LMI = [
                M11  M12  M13  M14
                M12' M22  M23  M24
                M13' M23' M33  M34
                M14' M24' M34' M44
            ];
                
            sol = optimize([LMI <= 10^(-4)*eye(16), W1 >= 0, W2 >= 0], [], options);

            if sol.problem
                sol.info
            end

            W1 = value(W1);
            W2 = value(W2);
            Y2 = value(Y2);
            
            P1 = I/W1;
            P2 = I/W2;
            K = Y2*P2;

            Kf{i, j} = K*dk;
            
            %% eig test
            % Qb = [Q1 O; O Q2];
            % Ab = [A O; A -Ar];
            % Bb = [B; B];
            % Kb = [zeros(2, 4) K];
            % P = [P1 O; O P2];
            % Eb = [I O;I -I];
            % LMI_ = Qb + addSym(P*(Ab + Bb*Kb)) + P*(Eb*Eb')*P/rho^(2);
            % eig(LMI_)
            % eig(P1)
            % eig(P2)
            % eig(M)
        end
    end

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
wb = [w; r];

xb = zeros(8, length(t));
xb(1, 1) = 0.5; xb(3, 1) = -0.5;

x = zeros(4, length(t)); xh = x; xr = x;
x(1, 1) = 0.5; x(3, 1) = -0.5;


% find x, u
if EXE.TRAJ
    for i = 1 : length(t) - 1
        if isnan(norm(xb))
            error('diverge')
        end
        
        if mod(i - 1, 0.1/dt) == 0 % print per 0.1s
            disp(['Calculate traj: t = ' num2str(t(i))]);
        end
        
        j = 2*i-1;
        temp = zeros(8, 1);
        temp1 = zeros(4, 1);
        temp2 = zeros(4, 1);

        % sum_hh = 0;
        for k = 1 : rb.A.len
            for kk = 1 : rb.B.len
                A = rb.A.val{k}; B = rb.B.val{kk};
                K = gain.K{k, kk};
                hh = rb.A.mf(x(:, i), k)*rb.B.mf(x([1, 3], i), kk);
                
                switch state
                    case 'n'
                        % x_hat, x, xr
                        k1 = rb.f(x(:, i), K*(x(:, i)-xr(:, i))) + w(j);
                        kr1 = fl(j, xr(:, i), r, Ar, I);
                        
                        k2 = rb.f(x(:, i)+0.5*k1*dt, K*(x(:, i)-xr(:, i) + 0.5*(k1-kr1)*dt)) + w(j+1);
                        kr2 = fl(j+1, xr(:, i)+0.5*kr1*dt, r, Ar, I);
                        
                        k3 = rb.f(x(:, i)+0.5*k2*dt, K*(x(:, i)-xr(:, i) + 0.5*(k2-kr2)*dt)) + w(j+1);
                        kr3 = fl(j+1, xr(:, i)+0.5*kr2*dt, r, Ar, I);
                                
                        k4 = rb.f(x(:, i)+k3*dt, K*(x(:, i)-xr(:, i) + (k3-kr3)*dt)) + w(j+2);
                        kr4 = fl(j+2, xr(:, i)+kr3*dt, r, Ar, I);
                        
                        temp2 = temp2 + hh.*(k1+2*k2+2*k3+k4)*dt/6;
                        xr(:, i+1) = xr(:, i) + (kr1+2*kr2+2*kr3+kr4)*dt/6;
                        
                    case 'l'
                        Ab = [A B*K; A -Ar + B*K];
                        Eb = [I O;I -I];
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
                x(:, i+1) = x(:, i) + temp2;
                xb(:, i+1) = [x(:, i+1); x(:, i+1) - xr(:, i+1)];
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
sum_A = zeros(4);
for i = 1 : rb.A.len
    h = rb.A.mf([0, 1, -1, 0], i);
    sum = sum + h;
    sum_A = sum_A + h.*rb.A.val{i};
end
disp(['sum of mbfun of A: ' num2str(sum)])
disp('sum of of A: ')
disp(sum_A)

sum = 0;
sum_B = zeros(4, 2);
for i = 1 : rb.B.len
    h = rb.B.mf([0, 0, 0, 0], i);
    sum = sum + h;
    sum_B = sum_B + h.*rb.B.val{i};
end
disp(['sum of mbfun of B: ' num2str(sum)])
disp('sum of of B: ')
disp(sum_B)

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
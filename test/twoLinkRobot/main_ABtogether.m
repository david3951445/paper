%two-link robot, tpModel(A, B solve together), no observer, xb = [x x-xr]'.
clc; clear; close all
addpath(genpath('..\..\src'))
addpath(genpath('function'))

%% fixed parameters
I = eye(4); O = zeros(4);
Ar = [0 1 0 0; -6 -5 0 0; 0 0 0 1; 0 0 -6 -5];
Br = -Ar;

%% tunable parameters
state = 'l'; % n : nonlinear, l : linear
E = I;
d2 = 0; % scale of disturblance

Q   = 10^(-2)*diag([1 0.001 1 0.001]);
rho = 10^(3);
Q1  = 10^(-3)*I;
Q2 = 10^(-6)*I;

dt      = 0.002;
tf      = 5; % parameter of trajectory
amp     = 6;
freq    = 1; % parameter of sine wave in reference input

rb = Robot();

%% find Af, Bf, C
if EXE.A_B
    rb.AB = TPmodel(rb.ABl);
    rb.save('data/rb.mat', 'AB')
end
 
%% LMI
LEN = rb.AB.len;
if EXE.LMI
    rb.K = cell(1, LEN);
    for i = 1 : LEN
        fprintf('LMI iter: %d/%d\n', i, LEN)
        A = rb.AB.val{i}(:, 1:4); B = rb.AB.val{i}(:, 5:6);

        rb.K{i} = solveLMI1(A, B, E, Ar, Br, Q, rho);
    end
    rb.save('data/rb.mat', 'K')
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
        for k = 1 : rb.AB.len
            A = rb.AB.val{k}(:, 1:4); B = rb.AB.val{k}(:, 5:6);
            K = rb.K{k};
            p = [x(:, i)'];
            hh = rb.AB.mf(p, k);
            
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
        % sum_hh

        switch state
            case 'n'
                x(:, i+1) = x(:, i) + temp2;
                xb(:, i+1) = [x(:, i+1); x(:, i+1) - xr(:, i+1)];
            case 'l'
                xb(:, i+1) = xb(:, i) + temp;
        end
    end

    if state == 'l'
        xb(1:4, :) = xb(5:8, :) - xb(1:4, :);
    end
end

if EXE.PLOT
    Plot(t, xb);
end

%% if you want to check if sum of membership function is 1
sum = 0;
sum_AB = zeros(4, 6);
for i = 1 : rb.AB.len
    h = rb.AB.mf([0, 0.5, -0.5, 0], i);
    sum = sum + h;
    sum_AB = sum_AB + h.*rb.AB.val{i};
end
disp(['sum of mbfun of AB: ' num2str(sum)])
disp('sum of of AB: ')
disp(sum_AB)

%% if u want to check norm of Matrix
for i = 1 : rb.AB.len
    disp(['norm of K: ' num2str(norm(rb.K{i}))]);
end

%% Remove path
rmpath(genpath('function'))
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
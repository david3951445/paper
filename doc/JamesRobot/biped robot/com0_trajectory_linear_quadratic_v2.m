clc;clear;close all;
A = [1,1];
B = [3,2];
C = [4.5,1];
D = [6.3,2];
E = [5.2,3.5];
F = [8,5];
T = 20;
hh = 5e-4; % 配合Runge Kutta 
tacc = 10;
%% a to a' (quadratic_start)
t1 = tacc;
t = 0:hh:t1;
dB = B-A;
dA1 = dB*tacc/(2*T); %% A to A1: 限制 q(1) = a4 + a3 + a0 = A + dA  
h = t/tacc;
 
for i = 1:2
    for j = 1:length(t)
        [p(j,i),v(j,i),a(j,i)] = q_s(A(i),dB(i),tacc,h(j),T);
    end
end
A1 = p(end,:);
t_pass = length(t) - 1;
%% a' to a'' (linear_start)
Ts = T+tacc/2; %% 因為0秒到tacc的加速時間所走的距離較短,在Linear的部分要多tacc/2秒去補齊
t2 = Ts-tacc; 
t = t1:hh:t2;
h = (t-t1)/(Ts-tacc); %% 把(A+dA1)當作h的原點

for i = 1:2
    for j = 1:length(t)
        [p(j+t_pass,i),v(j+t_pass,i),a(j+t_pass,i)] = l_s(A(i),dA1(i),dB(i),h(j),Ts,tacc);
    end
end
A2 = p(end,:);
t_pass = t_pass + length(t) - 1;
%% a'' to b' (quadratic)
t3 = t2 + 2*tacc;
t = t2:hh:t3;
h = (t-t2)/(2*tacc); %% 把(B-dB1)當作h的原點
dB1 = B-A2;
dC = C-B;

for i = 1:2
    for j = 1:length(t)
        [p(j+t_pass,i),v(j+t_pass,i),a(j+t_pass,i)] = q(B(i),dB1(i),dC(i),tacc,h(j),T);
    end
end
B1 = p(end,:);
t_pass = t_pass + length(t) - 1;
%% b' to b'' (linear)
t4 = T + t3 - 2*tacc;
t = t3:hh:t4;
h = (t-t3+tacc)/T;

for i = 1:2
    for j = 1:length(t)
        [p(j+t_pass,i),v(j+t_pass,i),a(j+t_pass,i)] = l(B(i),dC(i),h(j),T);
    end
end
B2 = p(end,:);
t_pass = t_pass + length(t) - 1;
%% b'' to c' (quadratic)
t5 = t4 + 2*tacc;
t = t4:hh:t5;
h = (t-t4)/(2*tacc);
dC1 = C-B2;
dD = D-C;

for i = 1:2
    for j = 1:length(t)
        [p(j+t_pass,i),v(j+t_pass,i),a(j+t_pass,i)] = q(C(i),dC1(i),dD(i),tacc,h(j),T);
    end
end
C1 = p(end,:);
t_pass = t_pass + length(t) - 1;
%% c' to c'' (linear)
t6 = T + t5 - 2*tacc;
t = t5:hh:t6;
h = (t-t5+tacc)/T;

for i = 1:2
    for j = 1:length(t)
        [p(j+t_pass,i),v(j+t_pass,i),a(j+t_pass,i)] = l(C(i),dD(i),h(j),T);
    end
end
C2 = p(end,:);
t_pass = t_pass + length(t) - 1;
%% c'' to d' (quadratic)
t7 = t6 + 2*tacc;
t = t6:hh:t7;
h = (t-t6)/(2*tacc);
dD1 = D-C2;
dE = E-D;

for i = 1:2
    for j = 1:length(t)
        [p(j+t_pass,i),v(j+t_pass,i),a(j+t_pass,i)] = q(D(i),dD1(i),dE(i),tacc,h(j),T);
    end
end
D1 = p(end,:);
t_pass = t_pass + length(t) - 1;
%% d' to d'' (linear)
t8 = T + t7 - 2*tacc;
t = t7:hh:t8;
h = (t-t7+tacc)/T;

for i = 1:2
    for j = 1:length(t)
        [p(j+t_pass,i),v(j+t_pass,i),a(j+t_pass,i)] = l(D(i),dE(i),h(j),T);
    end
end
D2 = p(end,:);
t_pass = t_pass + length(t) - 1;
%% d'' to e' (quadratic)
t9 = t8 + 2*tacc;
t = t8:hh:t9;
h = (t-t8)/(2*tacc);
dE1 = E-D2;
dF = F-E;

for i = 1:2
    for j = 1:length(t)
        [p(j+t_pass,i),v(j+t_pass,i),a(j+t_pass,i)] = q(E(i),dE1(i),dF(i),tacc,h(j),T);
    end
end
E1 = p(end,:);
t_pass = t_pass + length(t) - 1;
%% e' to e'' (linear_end)
t10 = T + t9 - 2*tacc + tacc/2; %% 多(tacc/2)秒
t = t9:hh:t10;
h = (t-t9+tacc)/T;

for i = 1:2
    for j = 1:length(t)
        [p(j+t_pass,i),v(j+t_pass,i),a(j+t_pass,i)] = l(E(i),dF(i),h(j),T);
    end
end
E2 = p(end,:);
t_pass = t_pass + length(t) - 1;
%% e'' to f (quadratic_end) 
t11 = t10 + tacc;
t = t10:hh:t11;
h = (t-t10)/tacc;
dF1 = F-E2;
disp(dF1)
for i = 1:2
    for j = 1:length(t)
        [p(j+t_pass,i),v(j+t_pass,i),a(j+t_pass,i)] = q_e(F(i),dF(i),dF1(i),tacc,h(j),Ts);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 為了處理 num of q 的問題, xr 要多兩個 sample 點 --> 這邊要多四個 sample 點 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

t_end = t11+4*hh;
t = 0:hh:t_end;
for i = 1:4
    p(end+1,:) = p(end,:);
    v(end+1,:) = v(end,:);
    a(end+1,:) = a(end,:);
end


save com0_trajectory.mat
%% debug 
figure; 
plot(t,p);
figure; 
plot(t,v);
figure; 
plot(t,a);
figure; 
plot(p(:,1),p(:,2));

%%
figure; 
plot(t(1:end-1),diff(a(:,1))/hh);

figure; 
plot(t(1:end-1),diff(a(:,2))/hh);
%% function 
function [p,v,a] = q_s(A,dB,tacc,h,T)
 a4 = tacc*dB/(-2*T);
 a3 = tacc*dB/T;
 a2 = 0;
 a1 = 0;
 a0 = A;
 p =  a4*h^4    + a3*h^3   + a2*h^2 + a1*h     +a0  ;
 v = (4*a4*h^3  + 3*a3*h^2 + 2*a2*h + a1)/tacc      ;
 a = (12*a4*h^2 + 6*a3*h   + 2*a2       )/(tacc^2)  ;
end

function [p,v,a] = l_s(A,dA1,dB,h,Ts,tacc)
 p = (dB-dA1)*h + (A+dA1);
 v = (dB-dA1)/(Ts-tacc);
 a = 0;
end

function [p,v,a] = q(B,dB1,dC,tacc,h,T)
 a4 = - (tacc*dC/T - dB1);
 a3 = 2*(tacc*dC/T - dB1);
 a2 = 0;
 a1 = 2*dB1;
 a0 = B-dB1;
 p =  a4*h^4    + a3*h^3   + a2*h^2 + a1*h     +a0   ;
 v = (4*a4*h^3  + 3*a3*h^2 + 2*a2*h + a1)/(2*tacc)   ;
 a = (12*a4*h^2 + 6*a3*h   + 2*a2       )/(2*tacc)^2 ;
end

function [p,v,a] = l(B,dC,h,T)
p = dC*h+B;
v = dC/T;
a = 0;
end

function [p,v,a] = q_e(D,dD,dD1,tacc,h,Ts)
 a4 = (dD-dD1)/(2*(Ts-tacc)/tacc);
 a3 = -(dD-dD1)/((Ts-tacc)/tacc);
 a2 = 0;
 a1 = (dD-dD1)/((Ts-tacc)/tacc);
 a0 = D-dD1;
 p =  a4*h^4    + a3*h^3   + a2*h^2 + a1*h   +a0  ;
 v = (4*a4*h^3  + 3*a3*h^2 + 2*a2*h + a1)/tacc    ;
 a = (12*a4*h^2 + 6*a3*h   + 2*a2       )/tacc^2  ;
%  if abs(p) < 1e-15
%      p = 0;
%  end
%   if abs(v) < 1e-15
%      v = 0;
%   end
%   if abs(a) < 1e-15
%      a = 0;
%  end
end
